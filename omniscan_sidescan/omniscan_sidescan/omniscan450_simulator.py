#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""A fake Omniscan 450 that speaks ping-protocol over TCP.

Lets the driver, the nodes and the launch files be run and tested with no
sonar attached.  It answers device information and protocol version requests,
accepts the parameter messages, and streams synthetic os_mono_profile messages
once pinging is enabled.

Run one directly::

    ros2 run omniscan_sidescan omniscan450_simulator --port 51200

or start a set matching the four-sonar launch file::

    ros2 run omniscan_sidescan omniscan450_simulator --bind 127.0.0.1 \\
        --port 51200 --port 51201 --port 51202 --port 51203
"""

import argparse
import math
import socket
import struct
import threading
import time

from .brping import definitions, PingMessage, PingParser

# Values reported in the device information reply.
DEVICE_TYPE = 3
DEVICE_REVISION = 1
FIRMWARE_VERSION = (1, 2, 3)

PROTOCOL_VERSION = (1, 0, 0)
# Independent API value: do not derive the simulated command ID from the
# driver's generated definitions, which previously hid a protocol mismatch.
WIRE_SET_SPEED_OF_SOUND = 116


class Omniscan450Simulator(object):
    """Serves one simulated sonar on a TCP port."""

    def __init__(self, bind='127.0.0.1', port=51200, channel_number=0,
                 ping_interval=0.05, seed=0.0):
        """Create a simulator; call start() to begin serving.

        Args:
            bind: address to listen on.
            port: TCP port to listen on.
            channel_number: reported in every profile.
            ping_interval: seconds between streamed profiles.
            seed: shifts the synthetic return pattern so several simulators
                produce visibly different data.
        """
        self.bind = bind
        self.port = port
        self.channel_number = channel_number
        self.ping_interval = ping_interval
        self.seed = seed

        # Defaults matching the sonar's own power-up state.
        self.start_mm = 0
        self.length_mm = 5000
        self.msec_per_ping = 0
        self.pulse_len_percent = 0.002
        self.filter_duration_percent = 0.0015
        self.gain_index = -1
        self.num_results = 600
        self.speed_of_sound_mm = 1482000
        self.enabled = False

        self.ping_number = 0
        self._power_on = time.monotonic()

        self._server = None
        self._thread = None
        self._running = threading.Event()

    @property
    def address(self):
        """Return the (host, port) bound; the port is real after start()."""
        return self._server.getsockname() if self._server else (self.bind, self.port)

    def start(self):
        """Bind the socket and serve in a background thread."""
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self.bind, self.port))
        self._server.listen(1)
        self._server.settimeout(0.2)

        self._running.set()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        """Stop serving and close the listening socket."""
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._server is not None:
            self._server.close()
            self._server = None

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

    def _serve(self):
        while self._running.is_set():
            try:
                connection, _ = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                self._handle_connection(connection)
            finally:
                connection.close()

    def _handle_connection(self, connection):
        """Serve a single client until it disconnects or we are stopped."""
        connection.setblocking(False)
        parser = PingParser()
        next_ping = time.monotonic()

        while self._running.is_set():
            try:
                data = connection.recv(4096)
                if not data:
                    return
                for byte in data:
                    if parser.parse_byte(byte) == PingParser.NEW_MESSAGE:
                        self._handle_message(connection, parser.rx_msg)
            except BlockingIOError:
                pass
            except OSError:
                return

            now = time.monotonic()
            if self.enabled and now >= next_ping:
                configured_interval = self.msec_per_ping / 1000.0
                next_ping = now + max(
                    self.ping_interval, configured_interval)
                try:
                    connection.sendall(self._build_profile())
                except OSError:
                    return

            time.sleep(0.002)

    def _handle_message(self, connection, msg):
        """Act on one decoded request from the client."""
        if msg.message_id == definitions.COMMON_GENERAL_REQUEST:
            self._handle_request(connection, msg.requested_id)
        elif msg.message_id == definitions.OMNISCAN450_OS_PING_PARAMS:
            self.start_mm = msg.start_mm
            self.length_mm = msg.length_mm
            self.msec_per_ping = msg.msec_per_ping
            self.pulse_len_percent = msg.pulse_len_percent
            self.filter_duration_percent = msg.filter_duration_percent
            self.gain_index = msg.gain_index
            self.num_results = msg.num_results
            self.enabled = bool(msg.enable)
        elif msg.message_id == WIRE_SET_SPEED_OF_SOUND:
            self.speed_of_sound_mm = msg.speed_of_sound

    def _handle_request(self, connection, requested_id):
        if requested_id == definitions.COMMON_DEVICE_INFORMATION:
            reply = PingMessage(definitions.COMMON_DEVICE_INFORMATION)
            reply.device_type = DEVICE_TYPE
            reply.device_revision = DEVICE_REVISION
            (reply.firmware_version_major,
             reply.firmware_version_minor,
             reply.firmware_version_patch) = FIRMWARE_VERSION
            reply.reserved = 0
        elif requested_id == definitions.COMMON_PROTOCOL_VERSION:
            reply = PingMessage(definitions.COMMON_PROTOCOL_VERSION)
            (reply.version_major,
             reply.version_minor,
             reply.version_patch) = PROTOCOL_VERSION
            reply.reserved = 0
        elif requested_id == definitions.OMNISCAN450_OS_MONO_PROFILE:
            connection.sendall(self._build_profile())
            return
        else:
            reply = PingMessage(definitions.COMMON_NACK)
            reply.nacked_id = requested_id
            reply.nack_message = b'unsupported'

        reply.pack_msg_data()
        connection.sendall(reply.msg_data)

    def _build_profile(self):
        """Build one synthetic os_mono_profile packet."""
        msg = PingMessage(definitions.OMNISCAN450_OS_MONO_PROFILE)

        msg.ping_number = self.ping_number
        msg.start_mm = self.start_mm
        msg.length_mm = self.length_mm
        msg.timestamp_ms = int((time.monotonic() - self._power_on) * 1000) & 0xffffffff
        msg.ping_hz = 450000
        msg.gain_index = max(self.gain_index, 0)
        msg.num_results = self.num_results
        msg.sos_dmps = self.speed_of_sound_mm // 100
        msg.channel_number = self.channel_number
        msg.reserved = 0
        msg.pulse_duration_sec = 0.0002
        msg.analog_gain = 1.0
        msg.max_pwr_db = 0.0
        msg.min_pwr_db = -90.0
        msg.transducer_heading_deg = 90.0 * self.channel_number
        msg.vehicle_heading_deg = 12.5

        msg.pwr_results = self._synthetic_returns()
        msg.pack_msg_data()

        self.ping_number = (self.ping_number + 1) & 0xffffffff
        return msg.msg_data

    def _synthetic_returns(self):
        """Build a decaying sweep with a bright band, as little endian u16."""
        results = []
        for i in range(self.num_results):
            fraction = i / max(self.num_results - 1, 1)
            # Geometric spreading, plus a moving bright return so successive
            # pings are distinguishable.
            envelope = math.exp(-3.0 * fraction)
            band = math.exp(-200.0 * (fraction - self._band_position()) ** 2)
            value = 0.15 * envelope + 0.85 * band
            results.append(max(0, min(65535, int(value * 65535))))
        return struct.pack('<%dH' % self.num_results, *results)

    def _band_position(self):
        """Where the bright return sits, cycling over successive pings."""
        return 0.2 + 0.6 * (0.5 + 0.5 * math.sin(
            self.seed + self.ping_number * 0.05))


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Simulate one or more Omniscan 450 sonars over TCP.')
    parser.add_argument('--bind', default='127.0.0.1',
                        help='address to listen on (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, action='append', dest='ports',
                        help='TCP port to listen on; repeat for several sonars '
                             '(default: 51200)')
    parser.add_argument('--num-results', type=int, default=600,
                        help='initial profile length (default: 600)')
    parser.add_argument('--ping-interval', type=float, default=0.05,
                        help='seconds between profiles (default: 0.05)')
    parsed = parser.parse_args(args)

    ports = parsed.ports or [51200]

    simulators = []
    for index, port in enumerate(ports):
        simulator = Omniscan450Simulator(
            bind=parsed.bind, port=port, channel_number=index,
            ping_interval=parsed.ping_interval, seed=index * 1.7)
        simulator.num_results = parsed.num_results
        simulators.append(simulator.start())
        print('Omniscan 450 simulator listening on %s:%d' % (parsed.bind, port))

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for simulator in simulators:
            simulator.stop()


if __name__ == '__main__':
    main()
