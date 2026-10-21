"""Exercise the installed C++ executables over TCP and actual ROS services."""

from collections import deque
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


class ControlledSimulator(Omniscan450Simulator):
    """Inject faults independently of the native protocol implementation."""

    rejected_length = None
    rejection_mode = 'nack'
    freeze_counter = False
    invalid_profile = False
    bad_checksum = False
    ignore_information = False
    connections = 0
    profile_gate = None

    def _handle_connection(self, connection):
        self.connections += 1
        super()._handle_connection(connection)

    def _handle_message(self, connection, message):
        if message.message_id == 2197 and message.enable \
                and message.length_mm == self.rejected_length:
            if self.rejection_mode == 'nack':
                response = PingMessage(definitions.COMMON_NACK)
                response.nacked_id = 2197
                response.nack_message = b'unsupported range'
                connection.sendall(response.pack_msg_data())
            return
        super()._handle_message(connection, message)

    def _handle_request(self, connection, requested_id):
        if not self.ignore_information:
            super()._handle_request(connection, requested_id)

    def _build_profile(self):
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
        return packet


class NativeHarness:
    """Own simulator processes, DDS subscriptions, and lifecycle clients."""

    def __init__(self, tmp_path, count=2, missing=None, component=False, **overrides):
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
                         / 'lib/rclcpp_components/component_container_isolated')
            command = [str(container), '--ros-args', '-r', '__ns:=' + self.namespace,
                       '-r', '__node:=container']
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        self.client = AsyncParameterClient(self.node, self.namespace + '/native')
        if component:
            self.load_component(count)

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

    def wait(self, predicate, timeout=5.0):
        """Spin DDS until a condition holds; include the native log on failure."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if self.process.poll() is not None:
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
def test_cpp_components_wait_for_activation_and_unload_cleanly(native_factory, count):
    h = native_factory(component=True, count=count)
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
    client = h.node.create_client(
        UnloadNode, h.namespace + '/container/_container/unload_node')
    try:
        request = UnloadNode.Request()
        request.unique_id = h.component_id
        future = client.call_async(request)
        h.wait(future.done, timeout=2)
        assert future.result().success
    finally:
        h.node.destroy_client(client)
    h.pump()
    assert not any(device.enabled for device in h.devices.values())
    assert h.process.poll() is None


def test_cpp_lifecycle_shutdown_finishes_standalone_process(native_factory):
    h = native_factory()
    h.wait(lambda: h.combined)
    h.transition(Transition.TRANSITION_ACTIVE_SHUTDOWN)
    h.process.wait(timeout=1)
    assert h.process.returncode == 0
    assert not any(device.enabled for device in h.devices.values())
    assert 'LifecycleNode is not shut down' not in h.log_path.read_text()


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
