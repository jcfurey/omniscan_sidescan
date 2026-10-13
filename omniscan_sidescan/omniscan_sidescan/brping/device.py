# -*- coding: utf-8 -*-
"""Base class for devices implementing the Blue Robotics ping-protocol.

Vendored and trimmed from ping-python (Blue Robotics, MIT) -- see LICENSE in
this directory.  Two deliberate changes from upstream:

* ``_input_buffer`` is a per-instance attribute rather than a class attribute.
  Upstream declares it at class scope, so every device object in a process
  shares one receive buffer; with several sonars open at once the bytes of one
  device get fed to another device's parser and profiles are attributed to the
  wrong sonar.  This driver opens two or four sonars in a single process, so
  the buffer has to be per-instance.
* Serial transport is dropped, along with the pyserial dependency.  The
  Omniscan 450 is reached over TCP (or UDP) on the vehicle network.
"""

import socket
import select
import time
from collections import deque

from . import definitions
from . import pingmessage


class PingDevice(object):
    """Speaks ping-protocol over a socket."""

    def __init__(self):
        ## A helper class to take care of decoding the input stream
        self.parser = pingmessage.PingParser()

        ## Per-instance receive buffer; see the module docstring.
        self._input_buffer = deque()

        ## device id of this object, used for dst_device_id in outgoing messages
        self.my_id = 255

        # IO device
        self.iodev = None
        self.server_address = None
        self._is_stream = False
        self._write_timeout = 5.0

    def connect_tcp(self, host=None, port=51200, timeout=5.0):
        """Connect to the device over TCP.

        Args:
            host: server address (IPv4) or name.
            port: TCP port the device listens on.
            timeout: connect timeout, in seconds.
        """
        if host is None:
            host = '0.0.0.0'

        self.close()
        self.server_address = (host, port)
        sock = None
        try:
            # create_connection handles DNS as well as IPv4/IPv6 and closes
            # failed address attempts internally.
            sock = socket.create_connection(self.server_address, timeout)
            sock.setblocking(False)
        except socket.timeout:
            raise ConnectionError(
                "Connection to %s:%d timed out after %s seconds"
                % (host, port, timeout))
        except OSError as exception:
            if sock is not None:
                sock.close()
            raise ConnectionError(
                "Failed to open TCP connection to %s:%d: %s"
                % (host, port, exception))

        self.iodev = sock
        self.server_address = self.iodev.getpeername()
        self._is_stream = True
        self._write_timeout = timeout
        self._reset_input()

    def connect_udp(self, host=None, port=51200):
        """Connect to the device over UDP."""
        if host is None:
            host = '0.0.0.0'

        self.close()
        self.server_address = (host, port)
        sock = None
        try:
            addresses = socket.getaddrinfo(
                host, port, type=socket.SOCK_DGRAM)
            if not addresses:
                raise OSError('no addresses found')
            family, socktype, protocol, _, address = addresses[0]
            sock = socket.socket(family, socktype, protocol)
            sock.connect(address)
            sock.setblocking(False)
        except OSError as exception:
            if sock is not None:
                sock.close()
            raise ConnectionError(
                "Failed to open UDP connection to %s:%d: %s"
                % (host, port, exception))

        self.iodev = sock
        self.server_address = self.iodev.getpeername()
        self._is_stream = False
        self._reset_input()

    def _reset_input(self):
        """Discard parser state and bytes left over from an old connection."""
        self.parser = pingmessage.PingParser()
        self._input_buffer.clear()

    def read_io(self):
        """Read whatever is available from the io device into the buffer."""
        if self.iodev is None:
            raise ConnectionError(
                "IO device is null, configure a connection before reading.")

        buffer_size = 4096
        while True:
            try:
                data = self.iodev.recv(buffer_size)

                if not data:
                    # An empty recv() on a stream socket means the peer closed.
                    if self._is_stream:
                        raise ConnectionError("TCP connection closed by peer.")
                    break

                # extendleft() reverses the chunk and pop() takes from the
                # right, so the two together preserve arrival order.
                self._input_buffer.extendleft(data)

                if len(data) < buffer_size:
                    break

            except BlockingIOError:
                # Nothing available yet; normal for a non-blocking socket.
                break
            except OSError as exception:
                raise ConnectionError(
                    "Failed to read from socket: %s" % exception)

    def read(self):
        """Consume rx buffer data until a message is decoded.

        Returns the new PingMessage as soon as one is parsed -- there may be
        data left in the buffer, so subsequent calls may return further
        messages -- or None if the buffer runs dry first.
        """
        self.read_io()
        while len(self._input_buffer):
            b = self._input_buffer.pop()

            if self.parser.parse_byte(b) == pingmessage.PingParser.NEW_MESSAGE:
                # A successful read depends on successful handling.
                if not self.handle_message(self.parser.rx_msg):
                    return None
                return self.parser.rx_msg
        return None

    def write(self, data, timeout=None):
        """Write all bytes to the device and return the bytes written.

        ``socket.send`` may accept only part of a packet, especially when a
        non-blocking socket is under backpressure.  Ping packets must remain
        whole on the stream, so retry until all bytes are accepted or the
        write deadline expires.
        """
        if self.iodev is None:
            raise ConnectionError(
                "IO device is null, configure a connection before writing.")

        write_timeout = self._write_timeout if timeout is None else timeout
        deadline = time.monotonic() + write_timeout
        view = memoryview(data)
        written = 0

        while view:
            try:
                sent = self.iodev.send(view)
            except BlockingIOError:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ConnectionError(
                        "Timed out while writing to the device.")
                _, writable, _ = select.select(
                    [], [self.iodev], [], remaining)
                if not writable:
                    raise ConnectionError(
                        "Timed out while writing to the device.")
                continue
            except OSError as exception:
                raise ConnectionError(
                    "Failed to write to socket: %s" % exception)

            if sent == 0:
                raise ConnectionError("Socket connection closed while writing.")
            written += sent
            view = view[sent:]

        return written

    def wait_readable(self, timeout):
        """Wait until the socket can be read, returning False on timeout."""
        if self.iodev is None:
            raise ConnectionError(
                "IO device is null, configure a connection before reading.")
        try:
            readable, _, exceptional = select.select(
                [self.iodev], [], [self.iodev], max(0.0, timeout))
        except (OSError, ValueError) as exception:
            raise ConnectionError(
                "Failed while waiting for socket data: %s" % exception)
        if exceptional:
            raise ConnectionError("Socket reported an exceptional condition.")
        return bool(readable)

    def initialize(self):
        """Return True if the device replies to a protocol version request."""
        return self.request(definitions.COMMON_PROTOCOL_VERSION) is not None

    def request(self, m_id, timeout=0.5):
        """Request the given message id.

        Returns the device reply if it arrives within the timeout, else None.
        """
        msg = pingmessage.PingMessage(definitions.COMMON_GENERAL_REQUEST)
        msg.requested_id = m_id
        msg.pack_msg_data()
        self.write(msg.msg_data)

        return self.wait_message([m_id], timeout)

    def wait_message(self, message_ids, timeout=0.5):
        """Wait up to timeout seconds for a message with one of these ids."""
        deadline = time.monotonic() + timeout
        while True:
            msg = self.read()
            if msg is not None and msg.message_id in message_ids:
                return msg

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None

            # Continue immediately when read() left complete or partial data
            # in the userspace queue.  Otherwise block in select instead of
            # polling every 5 ms.
            if not self._input_buffer:
                self.wait_readable(remaining)

    def handle_message(self, msg):
        """Extract the fields of an incoming message into self attributes.

        Returns True if the message was handled successfully.
        """
        setattr(self, "_src_device_id", msg.src_device_id)
        setattr(self, "_dst_device_id", msg.dst_device_id)

        if msg.message_id not in pingmessage.payload_dict:
            return False

        try:
            for attr in pingmessage.payload_dict[msg.message_id]["field_names"]:
                setattr(self, "_" + attr, getattr(msg, attr))
        except AttributeError:
            return False

        return True

    def get_device_information(self):
        """Get a device_information message from the device.

        Returns None if there is no reply, otherwise a dict with the keys
        device_type, device_revision, firmware_version_major,
        firmware_version_minor, firmware_version_patch and reserved.
        """
        if self.request(definitions.COMMON_DEVICE_INFORMATION) is None:
            return None
        return {
            "device_type": self._device_type,
            "device_revision": self._device_revision,
            "firmware_version_major": self._firmware_version_major,
            "firmware_version_minor": self._firmware_version_minor,
            "firmware_version_patch": self._firmware_version_patch,
            "reserved": self._reserved,
        }

    def get_protocol_version(self):
        """Get a protocol_version message from the device.

        Returns None if there is no reply, otherwise a dict with the keys
        version_major, version_minor, version_patch and reserved.
        """
        if self.request(definitions.COMMON_PROTOCOL_VERSION) is None:
            return None
        return {
            "version_major": self._version_major,
            "version_minor": self._version_minor,
            "version_patch": self._version_patch,
            "reserved": self._reserved,
        }

    def close(self):
        """Close the underlying socket, if one is open."""
        if self.iodev is not None:
            try:
                self.iodev.close()
            finally:
                self.iodev = None
                self._is_stream = False
                self._reset_input()
