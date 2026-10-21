"""ROS publication, parameter semantics, and independent connection recovery."""

from dataclasses import replace
import socket
import time

from diagnostic_msgs.msg import DiagnosticStatus
from omniscan_sidescan.brping import definitions, PingMessage
from omniscan_sidescan.omniscan450_simulator import Omniscan450Simulator
from omniscan_sidescan.sidescansonar_node import SonarNode
from omniscan_sidescan.sidescansonarall_node import SonarAllNode
from omniscan_sidescan_msgs.msg import SideScanSonar, SonarProfile
import pytest
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


class FaultSimulator(Omniscan450Simulator):
    """Reject or silently ignore a selected scan length after startup."""

    rejected_length = None
    rejection_mode = 'nack'

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


def _wait_for(predicate, timeout=3.0, nodes=()):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.005)
        time.sleep(0.005)
    return predicate()


def _streaming(node):
    return all(s.snapshot().state == 'streaming' for s in node.sonars.values())


@pytest.fixture
def node_factory():
    """Keep workers, ROS resources, and simulator lifetimes in a safe order."""
    rclpy.init()
    nodes, simulators = [], []

    def create(count=2, initial_ping=None, **overrides):
        node_class = SonarNode if count == 2 else SonarAllNode
        devices = {}
        parameters = {
            'sensor_number': 999, 'read_timeout': 0.05,
            'configuration_timeout': 0.2, 'max_consecutive_failures': 2,
            'reconnect_initial_delay': 0.02, 'reconnect_max_delay': 0.08,
        }
        for index, channel in enumerate(node_class.CHANNELS, 1):
            simulator = FaultSimulator(
                port=0, channel_number=index, ping_interval=0.02)
            if initial_ping is not None:
                simulator.ping_number = initial_ping
            simulator.start()
            simulators.append(simulator)
            devices[channel] = simulator
            parameters[channel + '_ip_address'] = simulator.address[0]
            parameters[channel + '_port'] = simulator.address[1]
        parameters.update(overrides)
        node = node_class(
            'omniscan_test', parameter_overrides=[
                Parameter(name, value=value) for name, value in parameters.items()])
        nodes.append(node)
        return node, devices

    yield create
    for node in nodes:
        node.destroy_node()
    for simulator in simulators:
        simulator.stop()
    if rclpy.ok():
        rclpy.shutdown()


def test_complete_and_independent_profiles_use_sensor_qos(node_factory):
    node, _ = node_factory()
    listener = rclpy.create_node('profile_listener')
    received, profiles = [], []
    try:
        listener.create_subscription(
            SideScanSonar, '/omniscan999/range', received.append,
            qos_profile_sensor_data)
        listener.create_subscription(
            SonarProfile, '/omniscan999/starboard/profile', profiles.append,
            qos_profile_sensor_data)
        assert _wait_for(lambda: received and profiles, nodes=(node, listener))
        message, profile = received[-1], profiles[-1]
        assert message.header.frame_id == 'sonar'
        assert len(message.port_scaled_data) == message.num_results == 600
        assert len(message.starboard_scaled_data) == 600
        assert profile.header.frame_id == 'sonar_starboard'
        assert profile.channel_number == 2
        assert profile.transducer_heading_deg == pytest.approx(180.0)
        assert len(profile.pwr_data) == len(profile.scaled_data) == 600
        assert profile.configuration_generation == 0
    finally:
        listener.destroy_node()


def test_committed_parameter_updates_are_confirmed_by_each_worker(node_factory):
    node, devices = node_factory()
    assert _wait_for(lambda: _streaming(node))
    result = node.set_parameters_atomically([
        Parameter('length_mm', value=7000),
        Parameter('frame_id', value='sidescan_link'),
    ])
    assert result.successful, result.reason
    assert _wait_for(lambda: all(
        s.snapshot().applied_generation == 1 and s.snapshot().state == 'streaming'
        for s in node.sonars.values()))
    assert all(s.length_mm == 7000 for s in devices.values())
    assert node._frame_id == 'sidescan_link'
    assert _wait_for(node.read_and_publish)
    rejected = node.set_parameters_atomically([Parameter('frame_id', value='')])
    assert not rejected.successful
    assert node._frame_id == 'sidescan_link'
    assert node._generation == 1


def test_a_later_parameter_veto_has_no_hardware_or_runtime_side_effects(node_factory):
    node, devices = node_factory()
    assert _wait_for(lambda: _streaming(node))
    node.remove_on_set_parameters_callback(node._on_set_parameters)
    node.add_on_set_parameters_callback(
        lambda parameters: SetParametersResult(successful=False, reason='test veto'))
    node.add_on_set_parameters_callback(node._on_set_parameters)
    rejected = node.set_parameters_atomically([
        Parameter('length_mm', value=7000),
        Parameter('frame_id', value='rejected_frame'),
        Parameter('read_timeout', value=0.2),
    ])
    assert not rejected.successful
    assert node.get_parameter('length_mm').value == 5000
    assert node._frame_id == 'sonar'
    assert node._read_timeout == 0.05
    assert node._generation == 0
    assert all(s.snapshot().desired_generation == 0 for s in node.sonars.values())
    assert all(s.length_mm == 5000 for s in devices.values())


def test_missing_device_does_not_block_startup_or_ros_callbacks(node_factory):
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        started = time.monotonic()
        node, _ = node_factory(starboard_port=reservation.getsockname()[1])
        assert time.monotonic() - started < 0.5
        assert _wait_for(lambda: node.sonars['port'].snapshot().state == 'streaming')
        ticks = []
        timer = node.create_timer(0.01, lambda: ticks.append(time.monotonic()))
        assert _wait_for(lambda: len(ticks) >= 5, timeout=0.5, nodes=(node,))
        node.destroy_timer(timer)
        result = node.set_parameters_atomically([Parameter('length_mm', value=7000)])
        assert result.successful
        assert _wait_for(lambda: node.sonars['port'].snapshot().applied_generation == 1)
        assert node.sonars['starboard'].snapshot().applied_generation == -1


def test_healthy_profiles_continue_and_only_failed_connection_restarts(node_factory):
    node, devices = node_factory()
    listener = rclpy.create_node('outage_listener')
    received = []
    replacement = None
    try:
        listener.create_subscription(
            SonarProfile, '/omniscan999/port/profile', received.append,
            qos_profile_sensor_data)
        assert _wait_for(lambda: received and _streaming(node), nodes=(node, listener))
        healthy = node.sonars['port'].snapshot()
        failed_port = devices['starboard'].address[1]
        devices['starboard'].stop()
        assert _wait_for(lambda: node.sonars['starboard'].snapshot().state == 'retrying')
        prior_count = len(received)
        assert _wait_for(lambda: len(received) >= prior_count + 4, nodes=(node, listener))
        assert node.sonars['port'].snapshot().connections == healthy.connections
        assert not node.read_and_publish()
        replacement = Omniscan450Simulator(
            port=failed_port, channel_number=2, ping_interval=0.02).start()
        assert _wait_for(lambda: _streaming(node))
        assert _wait_for(node.read_and_publish)
        assert node.sonars['port'].snapshot().connections == healthy.connections
        assert node.sonars['starboard'].snapshot().connections >= 2
    finally:
        listener.destroy_node()
        if replacement is not None:
            replacement.stop()


@pytest.mark.parametrize('mode', ['nack', 'ignore'])
def test_failed_configuration_is_reported_and_recovers_independently(
        node_factory, monkeypatch, mode):
    node, devices = node_factory()
    assert _wait_for(lambda: _streaming(node))
    healthy_connections = node.sonars['port'].snapshot().connections
    rejected = devices['starboard']
    rejected.rejected_length = 7000
    rejected.rejection_mode = mode
    result = node.set_parameters_atomically([Parameter('length_mm', value=7000)])
    assert result.successful  # Acceptance of desired state, not hardware confirmation.
    assert _wait_for(lambda: node.sonars['port'].snapshot().applied_generation == 1)
    assert _wait_for(lambda: node.sonars['starboard'].snapshot().state == 'retrying')
    assert not node.read_and_publish()
    observed = []
    monkeypatch.setattr(node.diagnostics_pub, 'publish', observed.append)
    node.publish_diagnostics()
    status = next(s for s in observed[-1].status if s.name.endswith('/starboard'))
    values = {item.key: item.value for item in status.values}
    assert status.level == DiagnosticStatus.ERROR
    assert values['desired_generation'] == '1'
    assert values['applied_generation'] == '-1'
    assert values['desired.length_mm'] == '7000'
    rejected.rejected_length = None
    assert _wait_for(lambda: _streaming(node) and all(
        s.snapshot().applied_generation == 1 for s in node.sonars.values()))
    assert _wait_for(node.read_and_publish)
    assert node.sonars['port'].snapshot().connections == healthy_connections


def test_stale_and_skewed_sets_are_dropped_and_receive_stamps_are_preserved(
        node_factory, monkeypatch):
    node, _ = node_factory()
    assert _wait_for(lambda: _streaming(node))
    snapshots = {name: worker.snapshot() for name, worker in node.sonars.items()}
    now = time.monotonic()
    stamp = node.get_clock().now().nanoseconds

    def frozen(name, age, sequence):
        original = snapshots[name]
        return replace(original, sample=replace(
            original.sample, sequence=sequence,
            received_monotonic=now - age, timestamp_ns=stamp - int(age * 1e9)))

    current = {'port': frozen('port', 0.15, 10000),
               'starboard': frozen('starboard', 0.01, 10000)}
    for name, worker in node.sonars.items():
        monkeypatch.setattr(worker, 'snapshot', lambda name=name: current[name])
    combined = []
    monkeypatch.setattr(node.range_pub, 'publish', combined.append)
    assert not node.read_and_publish()
    assert 'skew' in node._combined_status
    current['port'] = frozen('port', 0.8, 10001)
    assert not node.read_and_publish()
    current['port'] = frozen('port', 0.02, 10002)
    current['starboard'] = frozen('starboard', 0.01, 10002)
    assert node.read_and_publish()
    message = combined[-1]
    actual_stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
    assert actual_stamp == current['port'].sample.timestamp_ns
    assert not node.read_and_publish()  # A profile is never reused in another set.


def test_four_channels_keep_their_own_frames_and_metadata(node_factory, monkeypatch):
    node, _ = node_factory(count=4)
    assert _wait_for(lambda: _streaming(node))
    received = {channel: [] for channel in node.CHANNELS}
    for channel in node.CHANNELS:
        monkeypatch.setattr(node.profile_pubs[channel], 'publish', received[channel].append)
    assert _wait_for(node.read_and_publish)
    for index, channel in enumerate(node.CHANNELS, 1):
        profile = received[channel][-1]
        assert profile.channel_number == index
        assert profile.header.frame_id == 'sonar_' + channel


def test_shutdown_cancels_long_configuration_waits(node_factory):
    node, devices = node_factory(configuration_timeout=60.0, read_timeout=60.0)
    assert _wait_for(lambda: _streaming(node))
    # Force a configuration that is silently ignored, hence would otherwise
    # wait for the full 60-second confirmation deadline in each worker.
    for simulator in devices.values():
        simulator.rejected_length = 7000
        simulator.rejection_mode = 'ignore'
    node.set_parameters_atomically([Parameter('length_mm', value=7000)])
    assert _wait_for(lambda: all(
        worker.snapshot().state == 'configuring' for worker in node.sonars.values()))
    workers = list(node.sonars.values())
    started = time.monotonic()
    node.close_sonars()
    assert time.monotonic() - started < 0.5
    assert all(not worker._thread.is_alive() for worker in workers)


def test_ping_counter_wrap_does_not_restart_connections(node_factory):
    node, _ = node_factory(initial_ping=0xfffffffd)
    assert _wait_for(lambda: _streaming(node) and all(
        s.snapshot().sample.data.ping_number < 10 for s in node.sonars.values()))
    assert all(s.snapshot().connections == 1 for s in node.sonars.values())


def test_duplicate_pings_trigger_only_the_affected_channel_recovery(node_factory, monkeypatch):
    node, devices = node_factory()
    assert _wait_for(lambda: _streaming(node))
    healthy_connections = node.sonars['port'].snapshot().connections
    starboard = node.sonars['starboard']
    counter = starboard.snapshot().sample.data.ping_number
    original = devices['starboard']._build_profile

    def duplicate():
        packet = PingMessage(msg_data=original())
        packet.ping_number = counter
        return packet.pack_msg_data()

    monkeypatch.setattr(devices['starboard'], '_build_profile', duplicate)
    assert _wait_for(lambda: starboard.snapshot().duplicate_profiles >= 2
                     and starboard.snapshot().connections >= 2)
    assert node.sonars['port'].snapshot().connections == healthy_connections


def test_diagnostics_detect_age_expiry_without_another_receive_event(node_factory, monkeypatch):
    node, _ = node_factory()
    assert _wait_for(lambda: _streaming(node))
    for worker in node.sonars.values():
        old = worker.snapshot()
        stale = replace(old, sample=replace(
            old.sample, received_monotonic=time.monotonic() - 1.0))
        monkeypatch.setattr(worker, 'snapshot', lambda stale=stale: stale)
    received = []
    monkeypatch.setattr(node.diagnostics_pub, 'publish', received.append)
    node._combined_status = 'complete profiles within age and receive skew limits'
    node.publish_diagnostics()
    assert all(status.level != DiagnosticStatus.OK for status in received[-1].status)
