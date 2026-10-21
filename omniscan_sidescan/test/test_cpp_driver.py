"""Exercise the installed C++ executables over TCP and actual ROS services."""

from collections import deque
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time
import uuid

from ament_index_python.packages import get_package_prefix
from composition_interfaces.srv import LoadNode, UnloadNode
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from omniscan_sidescan.brping import definitions, PingMessage
from omniscan_sidescan.omniscan450_simulator import Omniscan450Simulator
from omniscan_sidescan_msgs.msg import (
    SideScanSonar,
    SideScanSonarAll,
    SideScanSonarAllRaw,
    SideScanSonarRaw,
    SonarProfile,
)
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data


class FragmentingConnection:
    """Split real TCP writes across header, metadata, and sample boundaries."""

    def __init__(self, connection, owner):
        self.connection, self.owner = connection, owner

    def __getattr__(self, name):
        return getattr(self.connection, name)

    def recv(self, size):
        """Optionally keep an uncooperative peer open after the client half-closes."""
        data = self.connection.recv(size)
        if not data and self.owner.linger_on_eof:
            raise BlockingIOError()
        return data

    def sendall(self, data):
        """Forward either a normal packet or deliberately fragmented bytes."""
        if not self.owner.fragment_responses:
            self.connection.sendall(data)
        else:
            cursor = 0
            for size in (1, 2, 5, 13, 37, 211, 1021, len(data)):
                chunk = data[cursor:cursor + size]
                if not chunk:
                    break
                self.connection.sendall(chunk)
                cursor += len(chunk)
                time.sleep(0.0002)
        if self.owner.end_stream_after_profile and data[4:6] == b'\x96\x08':
            self.owner.end_stream_after_profile = False
            self.owner.final_ping = struct.unpack_from('<I', data, 8)[0]
            self.owner.enabled = False
            self.owner.ignore_information = True
            self.connection.shutdown(socket.SHUT_WR)


class ControlledSimulator(Omniscan450Simulator):
    """Inject faults independently of the native protocol implementation."""

    rejected_length = None
    rejection_mode = 'nack'
    reject_stop_commands = False
    ignore_stop_commands = False
    linger_on_eof = False
    freeze_counter = False
    invalid_profile = False
    bad_checksum = False
    ignore_information = False
    ignore_information_after_enable = False
    connections = 0
    profile_gate = None
    last_requested_length = None
    fragment_responses = False
    noise_only = False
    truncate_next_profile = False
    truncate_at_length = None
    truncated_profile_ping = None
    single_profile_after_barrier = False
    pause_next_profile = False
    confirmation_ping = None
    end_stream_after_profile = False
    final_ping = None
    reject_sync = False
    ignore_sync_readback = False
    wrong_sync_readback = False

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ping_commands = []
        self.sync_commands = []

    def _handle_connection(self, connection):
        self.connections += 1
        super()._handle_connection(FragmentingConnection(connection, self))

    def _handle_message(self, connection, message):
        if message.message_id == 170:
            self.sync_commands.append((message.channel_number, message.number_of_channels))
            if self.reject_sync:
                response = PingMessage(2)
                response.nacked_id, response.nack_message = 170, b'sync unsupported'
                connection.sendall(response.pack_msg_data())
                return
        if message.message_id == 2197:
            self.ping_commands.append((self.connections, bool(message.enable), message.length_mm))
            if not message.enable and self.ignore_stop_commands:
                return
        if message.message_id == 2197 and message.enable:
            self.last_requested_length = message.length_mm
        if message.message_id == 2197 and (message.enable or self.reject_stop_commands) \
                and message.length_mm == self.rejected_length:
            if self.rejection_mode == 'nack':
                response = PingMessage(definitions.COMMON_NACK)
                response.nacked_id = 2197
                response.nack_message = b'unsupported range'
                connection.sendall(response.pack_msg_data())
            return
        super()._handle_message(connection, message)

    def _handle_request(self, connection, requested_id):
        if requested_id == 169:
            if self.ignore_sync_readback:
                return
            if self.wrong_sync_readback:
                response = PingMessage(169)
                response.channel_number, response.number_of_channels = 255, 0
                connection.sendall(response.pack_msg_data())
                return
        if not self.ignore_information and not (self.ignore_information_after_enable
                                                and self.enabled):
            super()._handle_request(connection, requested_id)
            if requested_id == 4 and self.enabled and self.single_profile_after_barrier:
                self.pause_next_profile = True
            if requested_id == 4 and self.length_mm == self.truncate_at_length:
                # Inject after the ordered command barrier; earlier profiles
                # cannot confirm a configuration and are intentionally ignored.
                self.truncate_next_profile = True
                self.truncate_at_length = None

    def _build_profile(self):
        if self.noise_only:
            self.ping_number = (self.ping_number + 1) & 0xffffffff
            return b'BR\xff\xff\x96\x08\x00\x00' + bytes(16384)
        if self.profile_gate is not None:
            while self._running.is_set():
                if self.profile_gate.acquire(timeout=0.01):
                    break
        counter = self.ping_number
        packet = bytearray(super()._build_profile())
        if self.freeze_counter:
            self.ping_number = counter
        if self.invalid_profile:
            struct.pack_into('<f', packet, 8 + 32, float('nan'))
            struct.pack_into('<H', packet, len(packet) - 2, sum(packet[:-2]) & 0xffff)
        if self.bad_checksum:
            packet[-1] ^= 1
        if self.pause_next_profile:
            self.pause_next_profile = False
            self.enabled = False
            self.confirmation_ping = counter
        if self.truncate_next_profile:
            self.truncate_next_profile = False
            self.enabled = False
            self.truncated_profile_ping = counter
            # A plausible incomplete NACK can hide a complete profile. Stop
            # sending so recovery must follow the frame timer, not socket I/O.
            return b'BR\x00\x10\x02\x00\x00\x00' + packet
        return packet


class NativeHarness:
    """Own simulator processes, DDS subscriptions, and lifecycle clients."""

    def __init__(self, tmp_path, count=2, missing=None, component=False,
                 device_faults=None, driver_env=None, **overrides):
        self.names = ('port', 'starboard', 'bow', 'stern')[:count]
        self.namespace = '/cpp_' + uuid.uuid4().hex[:10]
        self.node = rclpy.create_node('native_observer', namespace=self.namespace)
        self.devices = {}
        self.profiles = {name: deque(maxlen=1000) for name in self.names}
        self.combined, self.raw, self.diagnostics = (deque(maxlen=1000) for _ in range(3))
        self.process = None
        self.log_path = tmp_path / 'native.log'
        self.log = self.log_path.open('w')
        self.parameters = {
            'sensor_number': 999, 'read_timeout': 0.1,
            'configuration_timeout': 0.3, 'connect_timeout': 0.2,
            'max_consecutive_failures': 2,
            'reconnect_initial_delay': 0.02, 'reconnect_max_delay': 0.1,
            'num_results': 1200,
        }
        for index, name in enumerate(self.names, 1):
            device = ControlledSimulator(port=0, channel_number=index, ping_interval=0.025)
            for flag, value in (device_faults or {}).get(name, {}).items():
                setattr(device, flag, value)
            device.start()
            self.devices[name] = device
            self.parameters[name + '_ip_address'] = '127.0.0.1'
            self.parameters[name + '_port'] = device.address[1]
            self.node.create_subscription(
                SonarProfile, self.namespace + '/omniscan999/' + name + '/profile',
                lambda message, name=name: self.profiles[name].append(message),
                qos_profile_sensor_data)
        if missing is not None:
            self.devices[missing].stop()
        self.node.create_subscription(
            SideScanSonar if count == 2 else SideScanSonarAll,
            self.namespace + '/omniscan999/range', self.combined.append, qos_profile_sensor_data)
        self.node.create_subscription(
            SideScanSonarRaw if count == 2 else SideScanSonarAllRaw,
            self.namespace + '/omniscan999/range_raw', self.raw.append, qos_profile_sensor_data)
        self.node.create_subscription(
            DiagnosticArray, self.namespace + '/diagnostics', self.diagnostics.append, 10)
        self.parameters.update(overrides)
        executable = 'sidescansonar_node' if count == 2 else 'sidescansonarall_node'
        binary = Path(get_package_prefix('omniscan_sidescan')) / 'lib/omniscan_sidescan'
        command = [str(binary / executable), '--ros-args',
                   '-r', '__ns:=' + self.namespace, '-r', '__node:=native']
        for name, value in self.parameters.items():
            command.extend(['-p', name + ':=' + str(value)])
        if component:
            container = (Path(get_package_prefix('rclcpp_components'))
                         / 'lib/rclcpp_components/component_container')
            executor = 'multi-threaded' if component == 'multi-threaded' else 'single-threaded'
            command = [str(container), '--executor-type', executor]
            if executor == 'single-threaded':
                command.append('--isolated')
            command.extend(['--ros-args', '-r', '__ns:=' + self.namespace,
                            '-r', '__node:=container'])
            if executor == 'multi-threaded':
                command.extend(['-p', 'thread_num:=4'])
        environment = dict(os.environ, **(driver_env or {}))
        if component and os.environ.get('OMNISCAN_COMPONENT_PRELOAD'):
            environment['LD_PRELOAD'] = ':'.join(filter(None, [
                os.environ['OMNISCAN_COMPONENT_PRELOAD'], environment.get('LD_PRELOAD')]))
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        env=environment)
        self.client = AsyncParameterClient(self.node, self.namespace + '/native')
        if component:
            try:
                self.load_component(count)
            except BaseException:
                self.close()
                raise

    def load_component(self, count):
        """Load through the real component manager with intra-process enabled."""
        client = self.node.create_client(
            LoadNode, self.namespace + '/container/_container/load_node')
        try:
            assert client.wait_for_service(timeout_sec=3)
            request = LoadNode.Request()
            request.package_name = 'omniscan_sidescan'
            request.plugin_name = 'omniscan_sidescan::' + (
                'SonarNode' if count == 2 else 'SonarAllNode')
            request.node_name, request.node_namespace = 'native', self.namespace
            request.parameters = [Parameter(name, value=value).to_parameter_msg()
                                  for name, value in self.parameters.items()]
            request.extra_arguments = [
                Parameter('use_intra_process_comms', value=True).to_parameter_msg()]
            future = client.call_async(request)
            self.wait(future.done)
            result = future.result()
            assert result.success, result.error_message
            self.component_id = result.unique_id
        finally:
            self.node.destroy_client(client)

    def unload_component(self):
        """Unload an active or configuring node through its component manager."""
        client = self.node.create_client(
            UnloadNode, self.namespace + '/container/_container/unload_node')
        try:
            request = UnloadNode.Request()
            request.unique_id = self.component_id
            future = client.call_async(request)
            self.wait(future.done, timeout=2)
            assert future.result().success
        finally:
            self.node.destroy_client(client)

    def wait(self, predicate, timeout=5.0, allow_exit=False):
        """Spin DDS until a condition holds; include the native log on failure."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if not allow_exit and self.process.poll() is not None:
                break
            rclpy.spin_once(self.node, timeout_sec=0.005)
        assert predicate(), self.log_path.read_text()

    def pump(self, seconds=0.15):
        """Drain DDS during a bounded observation interval."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def update(self, **values):
        """Set desired parameters through the external atomic service."""
        assert self.client.wait_for_services(timeout_sec=2)
        future = self.client.set_parameters_atomically([
            Parameter(name, value=value) for name, value in values.items()])
        self.wait(future.done, timeout=1)
        return future.result().result

    def transition(self, transition):
        """Request a standard ROS lifecycle transition."""
        client = self.node.create_client(ChangeState, self.namespace + '/native/change_state')
        try:
            assert client.wait_for_service(timeout_sec=2)
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            self.wait(future.done, timeout=2)
            assert future.result().success, self.log_path.read_text()
        finally:
            self.node.destroy_client(client)

    def status(self, name):
        """Return the latest diagnostic status for a named channel."""
        if self.diagnostics:
            return next(s for s in self.diagnostics[-1].status if s.name.endswith('/' + name))
        return None

    def resources(self):
        """Inspect live descriptors, threads, and resident memory on Linux."""
        root = Path('/proc') / str(self.process.pid)
        pages = int((root / 'statm').read_text().split()[1])
        return (len(list((root / 'fd').iterdir())), len(list((root / 'task').iterdir())),
                pages * os.sysconf('SC_PAGE_SIZE') / (1024 * 1024))

    def close(self):
        """Stop native workers before destroying DDS and simulated devices."""
        try:
            if self.process is not None and self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
                    pytest.fail('C++ driver did not stop within three seconds')
            if self.process is not None:
                assert self.process.returncode == 0, self.log_path.read_text()
        finally:
            self.log.close()
            self.node.destroy_node()
            for device in self.devices.values():
                device.stop()


@pytest.fixture
def native_factory(tmp_path):
    """Keep each installed native driver and its ROS context isolated."""
    rclpy.init()
    harnesses = []

    def create(**options):
        harness = NativeHarness(tmp_path, **options)
        harnesses.append(harness)
        return harness

    yield create
    try:
        for harness in harnesses:
            harness.close()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('count', [2, 4])
def test_cpp_profiles_and_legacy_outputs_match_wire_data(native_factory, count):
    h = native_factory(count=count, length_mm=6000)
    h.wait(lambda: all(h.profiles.values()) and len(h.combined) >= 4 and h.raw)
    for index, name in enumerate(h.names, 1):
        profile = h.profiles[name][-1]
        assert profile.configuration_generation == 0
        assert profile.channel_number == index
        assert profile.header.frame_id == 'sonar_' + name
        assert profile.length_mm == 6000
        assert profile.transducer_heading_deg == 90.0 * index
        assert profile.vehicle_heading_deg == 12.5
        assert profile.ping_hz == 450000
        assert profile.sos_dmps == 14820
        assert len(profile.pwr_data) == len(profile.scaled_data) == 1200
        assert list(profile.scaled_data) == pytest.approx([
            -90.0 + value * 90.0 / 65535 for value in profile.pwr_data], abs=0.00001)
    message = h.combined[-2]
    # Every combined sample must correspond to a real per-head sample, preserve
    # its data, and carry the oldest contributing host receipt stamp.

    def corresponding():
        return {name: next((p for p in h.profiles[name]
                           if p.ping_number == getattr(message, name + '_ping_number')), None)
                for name in h.names}
    h.wait(lambda: all(corresponding().values()))
    heads = corresponding()

    def stamp(p):
        return p.header.stamp.sec * 1000000000 + p.header.stamp.nanosec
    assert stamp(message) == min(stamp(p) for p in heads.values())
    for name, profile in heads.items():
        assert list(getattr(message, name + '_scaled_data')) == list(profile.scaled_data)
        pings = [getattr(m, name + '_ping_number') for m in h.combined]
        assert len(pings) == len(set(pings))


def test_cpp_four_head_outage_keeps_healthy_heads_and_services_alive(native_factory):
    h = native_factory(count=4)
    h.wait(lambda: all(h.profiles.values()) and h.combined and h.diagnostics)
    failed_port = h.devices['bow'].address[1]
    h.devices['bow'].stop()
    before = {name: len(messages) for name, messages in h.profiles.items()}
    started = time.monotonic()
    assert h.update(length_mm=7000, frame_id='rig_link').successful
    assert time.monotonic() - started < 0.5
    healthy = ('port', 'starboard', 'stern')
    h.wait(lambda: all(
        len(h.profiles[name]) >= before[name] + 5
        and h.profiles[name][-1].configuration_generation == 1 for name in healthy))
    h.wait(lambda: h.status('bow').level == DiagnosticStatus.ERROR)
    h.pump()
    combined_count = len(h.combined)
    h.pump(0.15)
    assert len(h.combined) == combined_count
    assert all(h.devices[name].connections == 1 for name in healthy)
    h.devices['bow'] = ControlledSimulator(
        port=failed_port, channel_number=3, ping_interval=0.025).start()
    h.wait(lambda: len(h.combined) > combined_count and h.combined[-1].length_mm == 7000)
    assert h.combined[-1].header.frame_id == 'rig_link'
    assert h.profiles['bow'][-1].configuration_generation == 1
    assert all(h.devices[name].connections == 1 for name in healthy)


@pytest.mark.parametrize('mode', ['nack', 'ignore'])
def test_cpp_rejected_configuration_is_visible_and_recovers(native_factory, mode):
    h = native_factory()
    h.wait(lambda: all(h.profiles.values()) and h.combined)
    faulty = h.devices['starboard']
    faulty.rejection_mode, faulty.rejected_length = mode, 7000
    assert h.update(length_mm=7000).successful
    h.wait(lambda: h.profiles['port'][-1].configuration_generation == 1)
    h.wait(lambda: h.status('starboard') is not None
           and h.status('starboard').level == DiagnosticStatus.ERROR)
    values = {item.key: item.value for item in h.status('starboard').values}
    assert values['desired_generation'] == '1'
    assert values['applied_generation'] == '-1'
    assert values['desired.length_mm'] == '7000'
    h.pump()
    before = len(h.combined)
    h.pump(0.15)
    assert len(h.combined) == before
    faulty.rejected_length = None
    h.wait(lambda: len(h.combined) > before and h.combined[-1].length_mm == 7000)
    assert h.devices['port'].connections == 1


def test_cpp_missing_head_and_invalid_parameters_do_not_block(native_factory):
    h = native_factory(missing='starboard')
    h.wait(lambda: h.profiles['port'])
    assert not h.combined
    assert not h.update(length_mm=7000, frame_id='').successful
    assert not h.update(num_results=1201).successful
    assert not h.update(port_port=1234).successful
    assert not h.update(pulse_len_percent=float('nan')).successful
    assert h.update(length_mm=6000).successful
    h.wait(lambda: h.profiles['port'][-1].length_mm == 6000)
    assert h.profiles['port'][-1].configuration_generation == 1
    assert h.devices['port'].connections == 1


def test_cpp_rejected_reconfiguration_stops_with_last_confirmed_settings(native_factory):
    h = native_factory(reconnect_initial_delay=60.0, reconnect_max_delay=60.0)
    h.wait(lambda: h.combined)
    faulty = h.devices['starboard']
    faulty.rejected_length, faulty.reject_stop_commands = 7000, True
    assert h.update(length_mm=7000).successful
    h.wait(lambda: h.status('starboard') is not None and any(
        item.key == 'state' and item.value == 'retrying'
        for item in h.status('starboard').values))
    assert not faulty.enabled
    assert faulty.length_mm == 5000
    assert faulty.connections == h.devices['port'].connections == 1
    assert h.profiles['port'][-1].configuration_generation == 1
    faulty.rejected_length = None
    assert h.update(length_mm=8000).successful
    h.wait(lambda: h.combined[-1].length_mm == 8000)
    assert faulty.connections == 2
    assert h.devices['port'].connections == 1


def test_cpp_lifecycle_releases_connections_and_reactivates(native_factory):
    h = native_factory()
    h.wait(lambda: h.combined)
    h.transition(Transition.TRANSITION_DEACTIVATE)
    h.pump()
    before = len(h.combined)
    connections = [device.connections for device in h.devices.values()]
    h.pump(0.15)
    assert len(h.combined) == before
    assert not any(device.enabled for device in h.devices.values())
    assert h.update(length_mm=7000).successful
    h.transition(Transition.TRANSITION_ACTIVATE)
    h.wait(lambda: len(h.combined) > before and h.combined[-1].length_mm == 7000)
    assert [device.connections for device in h.devices.values()] == [n + 1 for n in connections]
    h.transition(Transition.TRANSITION_DEACTIVATE)
    h.transition(Transition.TRANSITION_CLEANUP)
    h.transition(Transition.TRANSITION_CONFIGURE)
    h.transition(Transition.TRANSITION_ACTIVATE)
    h.wait(lambda: all(device.connections == 3 for device in h.devices.values()))
    h.wait(lambda: h.combined[-1].length_mm == 7000)


def test_cpp_new_settings_interrupt_a_silent_long_read(native_factory):
    h = native_factory(read_timeout=60.0)
    h.wait(lambda: h.combined)
    h.devices['starboard'].enabled = False
    h.pump(0.15)
    assert h.update(length_mm=7000).successful
    h.wait(lambda: all(messages[-1].length_mm == 7000
                       and messages[-1].configuration_generation == 1
                       for messages in h.profiles.values()), timeout=0.8)
    assert all(device.connections == 1 for device in h.devices.values())


@pytest.mark.parametrize('stage', ['activation', 'reconfiguration'])
def test_cpp_confirmation_profile_is_delivered_once_when_stream_pauses(native_factory, stage):
    h = native_factory(read_timeout=60.0, max_profile_age=5.0, max_profile_skew=0.2)
    h.wait(lambda: len(h.combined) >= 3)
    if stage == 'activation':
        h.transition(Transition.TRANSITION_DEACTIVATE)
        h.pump(0.05)
    for device in h.devices.values():
        device.single_profile_after_barrier = True
    if stage == 'activation':
        h.transition(Transition.TRANSITION_ACTIVATE)
    else:
        assert h.update(length_mm=7000).successful
    h.wait(lambda: all(device.confirmation_ping is not None for device in h.devices.values()))
    h.wait(lambda: all(any(message.ping_number == h.devices[name].confirmation_ping
                           for message in messages)
                       for name, messages in h.profiles.items()), timeout=0.8)
    generation = 0 if stage == 'activation' else 1
    for name, messages in h.profiles.items():
        delivered = [message for message in messages
                     if message.ping_number == h.devices[name].confirmation_ping]
        assert len(delivered) == 1
        assert delivered[0].configuration_generation == generation
        assert delivered[0].length_mm == (5000 if generation == 0 else 7000)
    h.wait(lambda: all(getattr(h.combined[-1], name + '_ping_number')
                       == h.devices[name].confirmation_ping for name in h.names))
    before = len(h.combined)
    h.pump(0.15)
    assert len(h.combined) == before
    assert all(device.connections == (2 if stage == 'activation' else 1)
               for device in h.devices.values())


def test_cpp_read_timeout_update_interrupts_the_previous_deadline(native_factory):
    h = native_factory(read_timeout=60.0)
    h.wait(lambda: h.combined)
    faulty = h.devices['starboard']
    faulty.enabled = False
    h.pump(0.15)
    assert h.update(read_timeout=0.05).successful
    h.wait(lambda: faulty.connections == 2, timeout=0.8)
    h.wait(lambda: h.combined[-1].starboard_ping_number > 0)
    assert h.devices['port'].connections == 1
    assert h.profiles['port'][-1].configuration_generation == 0


@pytest.mark.parametrize('read_timeout', [0.05, 60.0])
@pytest.mark.parametrize('max_profile_age', [0.5, 5.0])
def test_cpp_truncated_frame_expires_during_a_silent_read(
        native_factory, read_timeout, max_profile_age):
    h = native_factory(read_timeout=read_timeout, max_profile_age=max_profile_age,
                       max_consecutive_failures=0)
    h.wait(lambda: h.combined)
    faulty = h.devices['starboard']
    faulty.truncate_next_profile = True
    h.wait(lambda: faulty.truncated_profile_ping is not None)
    if max_profile_age > 1.0:
        h.wait(lambda: any(message.ping_number == faulty.truncated_profile_ping
                           for message in h.profiles['starboard']), timeout=2.0)
        recovered = next(message for message in h.profiles['starboard']
                         if message.ping_number == faulty.truncated_profile_ping)
        stamp = recovered.header.stamp.sec * 1000000000 + recovered.header.stamp.nanosec
        age = (h.node.get_clock().now().nanoseconds - stamp) / 1e9
        assert 0.8 < age < 2.5  # Recovery must preserve original receipt time.
    h.wait(lambda: h.status('starboard') is not None and any(
        item.key == 'parser_errors' and int(item.value) > 0
        for item in h.status('starboard').values), timeout=2.5)
    if max_profile_age < 1.0:
        assert not any(message.ping_number == faulty.truncated_profile_ping
                       for message in h.profiles['starboard'])
        assert h.status('starboard').level == DiagnosticStatus.ERROR
    assert all(device.connections == 1 for device in h.devices.values())
    before = len(h.combined)
    faulty.enabled = True
    h.wait(lambda: len(h.combined) > before)


def test_cpp_truncated_profile_can_confirm_configuration_during_silence(native_factory):
    h = native_factory(configuration_timeout=60.0, read_timeout=60.0)
    h.wait(lambda: h.combined)
    faulty = h.devices['starboard']
    faulty.truncate_at_length = 7000
    assert h.update(length_mm=7000).successful
    h.wait(lambda: faulty.truncated_profile_ping is not None)
    h.wait(lambda: h.status('starboard') is not None and any(
        item.key == 'applied_generation' and item.value == '1'
        for item in h.status('starboard').values), timeout=2.5)
    assert all(device.connections == 1 for device in h.devices.values())
    assert not faulty.enabled
    faulty.enabled = True
    h.wait(lambda: h.combined[-1].length_mm == 7000)
    # DDS does not order delivery across independent topics/publication threads.
    h.wait(lambda: all(messages[-1].configuration_generation == 1
                       for messages in h.profiles.values()))


def test_cpp_new_settings_supersede_unconfirmed_configuration(native_factory):
    h = native_factory(configuration_timeout=60.0)
    h.wait(lambda: h.combined)
    faulty = h.devices['starboard']
    faulty.rejection_mode, faulty.rejected_length = 'ignore', 7000
    assert h.update(length_mm=7000).successful
    h.wait(lambda: faulty.last_requested_length == 7000
           and h.profiles['port'][-1].configuration_generation == 1)
    assert h.update(length_mm=5000).successful
    h.wait(lambda: all(messages[-1].configuration_generation == 2
                       and messages[-1].length_mm == 5000
                       for messages in h.profiles.values()), timeout=1.0)
    assert h.devices['port'].connections == 1
    assert faulty.connections == 2


def test_cpp_new_settings_wake_long_failure_backoff(native_factory):
    h = native_factory(reconnect_initial_delay=60.0, reconnect_max_delay=60.0)
    h.wait(lambda: h.combined)
    faulty = h.devices['starboard']
    faulty.rejected_length = 7000
    assert h.update(length_mm=7000).successful
    h.wait(lambda: h.status('starboard') is not None
           and h.status('starboard').level == DiagnosticStatus.ERROR)
    assert h.update(length_mm=5000).successful
    h.wait(lambda: all(messages[-1].configuration_generation == 2
                       for messages in h.profiles.values()), timeout=1.0)
    assert h.devices['port'].connections == 1
    assert faulty.connections == 2


def test_cpp_fragmented_streams_survive_bursts_of_settings_updates(native_factory):
    h = native_factory(count=4, configuration_timeout=1.0)
    h.wait(lambda: h.combined)
    for device in h.devices.values():
        device.fragment_responses = True
    for index in range(1, 21):
        assert h.update(length_mm=5000 + index * 100).successful
    h.wait(lambda: all(messages[-1].configuration_generation == 20
                       and messages[-1].length_mm == 7000
                       for messages in h.profiles.values()), timeout=3.0)
    h.wait(lambda: h.combined[-1].length_mm == 7000)
    for messages in h.profiles.values():
        generations = [message.configuration_generation for message in messages]
        assert generations == sorted(generations)
        assert len(messages[-1].pwr_data) == 1200


def test_cpp_continuous_malformed_traffic_stays_bounded_and_isolated(native_factory):
    h = native_factory(max_consecutive_failures=0)
    h.wait(lambda: h.combined)
    before = h.resources()
    faulty = h.devices['starboard']
    faulty.noise_only = True
    h.wait(lambda: h.status('starboard') is not None
           and h.status('starboard').level == DiagnosticStatus.ERROR)
    values = {item.key: item.value for item in h.status('starboard').values}
    assert int(values['parser_errors']) > 0
    assert h.status('port').level == DiagnosticStatus.OK
    assert h.resources()[2] < before[2] + 8
    assert all(device.connections == 1 for device in h.devices.values())
    faulty.noise_only = False
    previous = len(h.combined)
    h.wait(lambda: len(h.combined) > previous)


def test_cpp_lifecycle_churn_releases_threads_and_descriptors(native_factory):
    h = native_factory(count=4)
    h.wait(lambda: h.combined)
    baseline = h.resources()
    for index in range(12):
        h.transition(Transition.TRANSITION_DEACTIVATE)
        h.transition(Transition.TRANSITION_CLEANUP)
        inactive = h.resources()
        assert inactive[0] <= baseline[0] - 4
        assert inactive[1] <= baseline[1] - 4
        assert h.update(length_mm=6000 + index * 100).successful
        h.transition(Transition.TRANSITION_CONFIGURE)
        h.transition(Transition.TRANSITION_ACTIVATE)
        previous = len(h.combined)
        h.wait(lambda: len(h.combined) > previous
               and h.combined[-1].length_mm == 6000 + index * 100)
        resources = h.resources()
        assert resources[0] <= baseline[0] + 1
        assert resources[1] <= baseline[1] + 1
        assert resources[2] < baseline[2] + 12


def test_cpp_deactivation_during_reconnect_delivers_stop_commands(native_factory):
    h = native_factory(count=4)
    h.wait(lambda: h.combined)
    for index in range(12):
        failed_name = h.names[index % 4]
        faulty = h.devices[failed_name]
        healthy = [name for name in h.names if name != failed_name]
        pings = {name: h.profiles[name][-1].ping_number for name in healthy}
        previous = faulty.connections
        faulty.bad_checksum = True
        h.wait(lambda: faulty.connections > previous)
        h.wait(lambda: all(h.profiles[name][-1].ping_number > pings[name] for name in healthy))
        h.transition(Transition.TRANSITION_DEACTIVATE)
        h.wait(lambda: not any(device.enabled for device in h.devices.values()), timeout=0.5)
        faulty.bad_checksum = False
        old = {name: messages[-1].ping_number for name, messages in h.profiles.items()}
        h.transition(Transition.TRANSITION_ACTIVATE)
        h.wait(lambda: all(h.profiles[name][-1].ping_number > old[name] for name in h.names))


def test_cpp_duplicate_counter_reconnects_only_affected_head(native_factory):
    h = native_factory()
    h.wait(lambda: h.combined)
    h.devices['starboard'].freeze_counter = True
    h.wait(lambda: h.devices['starboard'].connections >= 2)
    assert h.devices['port'].connections == 1
    h.devices['starboard'].freeze_counter = False
    before = len(h.combined)
    h.wait(lambda: len(h.combined) >= before + 3)


def test_cpp_counter_wrap_and_device_reboot_are_distinguished(native_factory):
    h = native_factory()
    h.wait(lambda: len(h.combined) >= 3)
    faulty = h.devices['starboard']
    # A small backwards jump is a device reboot and must reconnect that head.
    faulty.ping_number = 0
    h.wait(lambda: faulty.connections == 2)
    # Unsigned counter wrap is normal, including independent uptime wrap.
    h.transition(Transition.TRANSITION_DEACTIVATE)
    faulty.ping_number = 0xfffffff9
    faulty._power_on = time.monotonic() - (0xffffffff - 180) / 1000.0
    h.transition(Transition.TRANSITION_ACTIVATE)
    before = len(h.profiles['starboard'])
    h.wait(lambda: len(h.profiles['starboard']) >= before + 12)
    assert faulty.connections == 3
    assert h.devices['port'].connections == 2


def test_cpp_invalid_metadata_and_configuration_drift_recover(native_factory):
    h = native_factory()
    h.wait(lambda: h.combined)
    faulty = h.devices['starboard']
    faulty.invalid_profile = True
    h.wait(lambda: faulty.connections >= 2)
    before = len(h.profiles['port'])
    h.wait(lambda: len(h.profiles['port']) >= before + 4)
    assert h.devices['port'].connections == 1
    faulty.invalid_profile = False
    before = len(h.combined)
    h.wait(lambda: len(h.combined) >= before + 3)
    connections = faulty.connections
    faulty.length_mm = 8000
    h.wait(lambda: faulty.connections > connections)
    h.wait(lambda: faulty.length_mm == 5000)
    assert h.devices['port'].connections == 1


def test_cpp_profile_age_uses_monotonic_time_when_ros_clock_is_paused(native_factory):
    h = native_factory(read_timeout=60.0, max_consecutive_failures=0,
                       max_profile_age=0.05, use_sim_time=True)
    h.wait(lambda: h.combined)
    assert h.combined[-1].header.stamp.sec == 0
    assert h.combined[-1].header.stamp.nanosec == 0
    h.devices['starboard'].enabled = False
    h.wait(lambda: h.status('starboard') is not None
           and h.status('starboard').level == DiagnosticStatus.ERROR)
    values = {item.key: item.value for item in h.status('starboard').values}
    assert values['state'] == 'streaming'  # The long read has not timed out.
    assert float(values['profile_age_sec']) > 0.05
    assert h.status('port').level == DiagnosticStatus.OK
    h.pump()
    before = len(h.combined)
    h.pump(0.1)
    assert len(h.combined) == before
    assert all(device.connections == 1 for device in h.devices.values())
    h.devices['starboard'].enabled = True
    h.wait(lambda: len(h.combined) > before)


def test_cpp_parser_errors_remain_visible_without_failure_reconnects(native_factory):
    h = native_factory(max_consecutive_failures=0)
    h.wait(lambda: h.combined)
    h.devices['starboard'].bad_checksum = True
    h.wait(lambda: h.status('starboard') is not None
           and h.status('starboard').level == DiagnosticStatus.ERROR)
    values = {item.key: item.value for item in h.status('starboard').values}
    assert int(values['parser_errors']) > 0
    assert all(device.connections == 1 for device in h.devices.values())
    h.devices['starboard'].bad_checksum = False
    before = len(h.combined)
    h.wait(lambda: len(h.combined) > before)
    assert all(device.connections == 1 for device in h.devices.values())


def test_cpp_skew_limits_and_complete_set_consumption(native_factory):
    h = native_factory(read_timeout=60.0, max_profile_age=60.0)
    h.wait(lambda: len(h.combined) >= 3)
    for device in h.devices.values():
        device.profile_gate = threading.Semaphore(0)
    h.pump(0.1)
    assert h.update(max_profile_skew=0.005).successful
    before = len(h.combined)
    port_count, starboard_count = (len(h.profiles[name]) for name in h.names)
    h.devices['port'].profile_gate.release()
    h.wait(lambda: len(h.profiles['port']) > port_count)
    h.pump(0.03)
    h.devices['starboard'].profile_gate.release()
    h.wait(lambda: len(h.profiles['starboard']) > starboard_count)
    h.pump(0.03)
    assert len(h.combined) == before
    assert h.update(max_profile_skew=0.2).successful
    h.devices['port'].profile_gate.release()
    h.wait(lambda: len(h.combined) == before + 1)
    # Another port profile cannot reuse the already consumed starboard sample.
    port_count = len(h.profiles['port'])
    h.devices['port'].profile_gate.release()
    h.wait(lambda: len(h.profiles['port']) > port_count)
    h.pump(0.03)
    assert len(h.combined) == before + 1


@pytest.mark.parametrize('count', [2, 4])
@pytest.mark.parametrize('executor', [True, 'multi-threaded'])
def test_cpp_components_wait_for_activation_and_unload_cleanly(native_factory, count, executor):
    h = native_factory(component=executor, count=count)
    client = h.node.create_client(GetState, h.namespace + '/native/get_state')
    try:
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(GetState.Request())
        h.wait(future.done)
        assert future.result().current_state.id == State.PRIMARY_STATE_UNCONFIGURED
    finally:
        h.node.destroy_client(client)
    h.pump(0.1)
    assert all(device.connections == 0 for device in h.devices.values())
    h.transition(Transition.TRANSITION_CONFIGURE)
    h.pump(0.1)
    assert all(device.connections == 0 for device in h.devices.values())
    h.transition(Transition.TRANSITION_ACTIVATE)
    h.wait(lambda: h.combined)
    # Unloading an active component must also join every native worker.
    h.unload_component()
    h.pump()
    assert not any(device.enabled for device in h.devices.values())
    assert h.process.poll() is None


@pytest.mark.parametrize('invalid', [False, True])
def test_cpp_final_profile_before_eof_is_validated_and_delivered_once(native_factory, invalid):
    h = native_factory()
    h.wait(lambda: len(h.combined) >= 4)
    peer = h.devices['port']
    peer.invalid_profile = invalid
    peer.end_stream_after_profile = True
    h.wait(lambda: peer.final_ping is not None)
    if not invalid:
        h.wait(lambda: any(p.ping_number == peer.final_ping for p in h.profiles['port']))
    h.wait(lambda: peer.connections >= 2)
    h.pump(0.1)
    assert sum(p.ping_number == peer.final_ping for p in h.profiles['port']) == int(not invalid)
    assert h.devices['starboard'].connections == 1
    before = h.profiles['starboard'][-1].ping_number
    h.wait(lambda: h.profiles['starboard'][-1].ping_number > before)


@pytest.mark.parametrize('count', [2, 4])
def test_cpp_sync_assignments_are_verified_and_reapplied_after_reconnect(native_factory, count):
    h = native_factory(count=count, sync_enabled=True)
    h.wait(lambda: len(h.combined) >= 4 and h.diagnostics)
    for index, name in enumerate(h.names):
        assert h.devices[name].sync_commands == [(index, count)]
        assert h.profiles[name][-1].channel_number == index
        values = {v.key: v.value for v in h.status(name).values}
        assert values['sync_assignment_verified'] == '1'
        assert values['acoustic_timing_verified'] == '0'
    peer = h.devices['port']
    peer.bad_checksum = True
    h.wait(lambda: len(peer.sync_commands) >= 2)
    peer.bad_checksum = False
    before = h.profiles['port'][-1].ping_number
    h.wait(lambda: h.profiles['port'][-1].ping_number > before)
    assert all(command == (0, count) for command in peer.sync_commands)
    assert all(h.devices[name].connections == 1 for name in h.names[1:])


@pytest.mark.parametrize('fault', ['reject_sync', 'ignore_sync_readback', 'wrong_sync_readback'])
def test_cpp_unverified_sync_never_starts_pinging_and_only_retries_failed_head(
        native_factory, fault):
    h = native_factory(sync_enabled=True, device_faults={'port': {fault: True}})
    peer = h.devices['port']
    h.wait(lambda: peer.connections >= 2 and len(h.profiles['starboard']) >= 4)
    assert not peer.enabled and not peer.ping_commands
    assert not h.profiles['port'] and not h.combined
    assert h.devices['starboard'].connections == 1
    setattr(peer, fault, False)
    h.wait(lambda: h.combined)


def test_cpp_sync_disabled_preserves_existing_device_assignments(native_factory):
    h = native_factory()
    h.wait(lambda: len(h.combined) >= 4)
    assert all(not peer.sync_commands for peer in h.devices.values())


def test_cpp_health_reports_survive_tight_profile_age_limits(native_factory, tmp_path):
    library = os.environ.get('OMNISCAN_PUBLICATION_STALL_LIBRARY')
    assert library, 'Run through colcon test or set OMNISCAN_PUBLICATION_STALL_LIBRARY'
    gate, ready = tmp_path / 'health-gate', tmp_path / 'health-ready'
    gate.touch()
    environment = {'OMNISCAN_STALL_GATE': str(gate), 'OMNISCAN_STALL_READY': str(ready),
                   'OMNISCAN_STALL_TOPIC': '/diagnostics', 'LD_PRELOAD': library}
    if os.environ.get('ASAN_OPTIONS'):
        runtime = subprocess.check_output(
            ['c++', '-print-file-name=libasan.so'], text=True).strip()
        environment['LD_PRELOAD'] = runtime + ':' + library
    try:
        h = native_factory(max_profile_age=0.001, driver_env=environment)
        h.wait(ready.exists)
        # One report is in flight and the next is pending. The latter should
        # survive waiting longer than the measurement freshness limit.
        h.pump(1.15)
        gate.unlink()
        h.wait(lambda: len(h.diagnostics) >= 2, timeout=0.4)
    finally:
        gate.unlink(missing_ok=True)


def test_cpp_ping_period_supports_ten_hz_and_checks_timeout_relationship(native_factory):
    h = native_factory(read_timeout=0.3, msec_per_ping=100)
    h.wait(lambda: len(h.combined) >= 5)
    assert all(peer.msec_per_ping == 100 for peer in h.devices.values())
    for messages in h.profiles.values():
        deltas = [b.timestamp_ms - a.timestamp_ms for a, b in zip(messages, list(messages)[1:])]
        assert deltas and all(delta >= 90 for delta in deltas)
    assert not h.update(msec_per_ping=300).successful
    assert not h.update(read_timeout=0.05).successful
    assert h.update(msec_per_ping=150, read_timeout=0.25).successful
    h.wait(lambda: all(peer.msec_per_ping == 150 for peer in h.devices.values()))
    assert all(peer.connections == 1 for peer in h.devices.values())


@pytest.mark.parametrize('component', [False, 'multi-threaded'])
def test_cpp_stalled_publication_keeps_other_heads_acquisition_and_services_alive(
        native_factory, tmp_path, component):
    library = os.environ.get('OMNISCAN_PUBLICATION_STALL_LIBRARY')
    assert library, 'Run through colcon test or set OMNISCAN_PUBLICATION_STALL_LIBRARY'
    gate, ready = tmp_path / 'stall-gate', tmp_path / 'stall-ready'
    gate.touch()
    environment = {'OMNISCAN_STALL_GATE': str(gate), 'OMNISCAN_STALL_READY': str(ready),
                   'LD_PRELOAD': library}
    if os.environ.get('ASAN_OPTIONS'):
        runtime = subprocess.check_output(
            ['c++', '-print-file-name=libasan.so'], text=True).strip()
        environment['LD_PRELOAD'] = runtime + ':' + library
    try:
        h = native_factory(count=4, component=component, driver_env=environment)
        if component:
            h.transition(Transition.TRANSITION_CONFIGURE)
            h.transition(Transition.TRANSITION_ACTIVATE)
        h.wait(lambda: ready.exists() and len(h.combined) >= 4 and h.diagnostics)
        assert not h.profiles['port']
        before = {v.key: v.value for v in h.status('port').values}
        started = time.monotonic()
        assert h.update(length_mm=7000).successful
        assert time.monotonic() - started < 0.5
        h.wait(lambda: all(peer.length_mm == 7000 for peer in h.devices.values()))
        h.wait(lambda: all(h.profiles[name][-1].configuration_generation == 1
                           for name in h.names[1:]))

        def profiles_received():
            return int(next(v.value for v in h.status('port').values
                            if v.key == 'profiles_received'))
        h.wait(lambda: profiles_received() > int(before['profiles_received']))
        values = {v.key: v.value for v in h.status('port').values}
        assert int(values['publication_coalesced']) > 0
        assert h.status('port').level == DiagnosticStatus.OK
        assert all(peer.connections == 1 for peer in h.devices.values())
        gate.unlink()
        h.wait(lambda: h.profiles['port']
               and h.profiles['port'][-1].configuration_generation == 1)
        assert h.devices['port'].ping_number - h.profiles['port'][-1].ping_number < 5
        # Device stop commands must arrive before deactivation joins the
        # blocked DDS call. The lifecycle reply follows once DDS is released.
        ready.unlink()
        gate.touch()
        h.wait(ready.exists)
        client = h.node.create_client(ChangeState, h.namespace + '/native/change_state')
        try:
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_DEACTIVATE
            future = client.call_async(request)
            h.wait(lambda: not any(peer.enabled for peer in h.devices.values()), timeout=0.5)
            assert not future.done()
            gate.unlink()
            h.wait(future.done)
            assert future.result().success
        finally:
            h.node.destroy_client(client)
    finally:
        gate.unlink(missing_ok=True)


@pytest.mark.parametrize('action', ['deactivate', 'shutdown', 'unload', 'sigint', 'sigterm'])
@pytest.mark.parametrize('fault', ['bad_checksum', 'ignore_information_after_enable'])
def test_cpp_unconfirmed_startup_stops_pinging_on_cleanup(native_factory, action, fault):
    h = native_factory(component=action == 'unload', configuration_timeout=60.0,
                       device_faults={'starboard': {fault: True}})
    faulty = h.devices['starboard']
    if action == 'unload':
        h.transition(Transition.TRANSITION_CONFIGURE)
        h.transition(Transition.TRANSITION_ACTIVATE)
    h.wait(lambda: faulty.enabled and faulty.last_requested_length == 5000
           and len(h.profiles['port']) >= 3)
    assert not h.profiles['starboard']
    started = time.monotonic()
    if action == 'unload':
        h.unload_component()
    elif action in ('sigint', 'sigterm'):
        h.process.send_signal(signal.SIGINT if action == 'sigint' else signal.SIGTERM)
    else:
        h.transition(Transition.TRANSITION_DEACTIVATE if action == 'deactivate'
                     else Transition.TRANSITION_ACTIVE_SHUTDOWN)
    h.wait(lambda: not any(device.enabled for device in h.devices.values()),
           timeout=0.5, allow_exit=True)
    assert time.monotonic() - started < 1.0
    if action in ('shutdown', 'sigint', 'sigterm'):
        h.process.wait(timeout=1)
        assert h.process.returncode == 0
    else:
        assert h.process.poll() is None
    if action == 'deactivate':
        setattr(faulty, fault, False)
        h.transition(Transition.TRANSITION_ACTIVATE)
        h.wait(lambda: h.combined)
        assert all(device.connections == 2 for device in h.devices.values())


@pytest.mark.parametrize('fault', ['bad_checksum', 'ignore_information_after_enable'])
def test_cpp_failed_startup_stops_pinging_during_retry_backoff(native_factory, fault):
    h = native_factory(configuration_timeout=0.2,
                       device_faults={'starboard': {fault: True}},
                       reconnect_initial_delay=60.0, reconnect_max_delay=60.0)
    faulty = h.devices['starboard']
    h.wait(lambda: faulty.last_requested_length == 5000 and h.profiles['port'])
    h.wait(lambda: h.status('starboard') is not None and any(
        item.key == 'state' and item.value == 'retrying'
        for item in h.status('starboard').values))
    assert not faulty.enabled
    assert faulty.connections == h.devices['port'].connections == 1
    before = len(h.profiles['port'])
    h.wait(lambda: len(h.profiles['port']) >= before + 3)
    setattr(faulty, fault, False)
    assert h.update(length_mm=7000).successful
    h.wait(lambda: h.combined and h.combined[-1].length_mm == 7000)
    assert faulty.connections == 2
    assert h.devices['port'].connections == 1


def test_cpp_lifecycle_shutdown_finishes_standalone_process(native_factory):
    h = native_factory()
    h.wait(lambda: h.combined)
    h.transition(Transition.TRANSITION_ACTIVE_SHUTDOWN)
    h.process.wait(timeout=1)
    assert h.process.returncode == 0
    assert not any(device.enabled for device in h.devices.values())
    assert 'LifecycleNode is not shut down' not in h.log_path.read_text()


@pytest.mark.parametrize('ignore_stop', [False, True])
def test_cpp_shutdown_bounds_cleanup_when_peer_ignores_half_close(native_factory, ignore_stop):
    faults = {'linger_on_eof': True, 'ignore_stop_commands': ignore_stop}
    h = native_factory(device_faults={name: faults for name in ('port', 'starboard')},
                       read_timeout=60.0)
    h.wait(lambda: h.combined)
    started = time.monotonic()
    h.process.send_signal(signal.SIGTERM)
    h.process.wait(timeout=1)
    assert time.monotonic() - started < 0.5
    assert h.process.returncode == 0
    assert all(device.ping_commands[-1][1] is False for device in h.devices.values())
    assert all(device.enabled == ignore_stop for device in h.devices.values())


def test_cpp_shutdown_cancels_long_configuration_deadline(native_factory):
    # Reserve a port so a missing endpoint cannot be reused by another process.
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        h = native_factory(starboard_port=reservation.getsockname()[1],
                           configuration_timeout=60.0, connect_timeout=60.0,
                           read_timeout=60.0)
        h.devices['port'].ignore_information = True
        h.pump(0.25)
        started = time.monotonic()
        h.process.send_signal(signal.SIGINT)
        h.process.wait(timeout=1)
        assert time.monotonic() - started < 0.5
        assert h.process.returncode == 0
        assert 'LifecycleNode is not shut down' not in h.log_path.read_text()
