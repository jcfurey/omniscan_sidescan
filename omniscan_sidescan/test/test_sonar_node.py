"""ROS-level tests for the two-channel side-scan node."""

import time

from omniscan_sidescan.omniscan450_driver import SonarConnectionError
from omniscan_sidescan.omniscan450_simulator import Omniscan450Simulator
from omniscan_sidescan.sidescansonar_node import SonarNode
from omniscan_sidescan_msgs.msg import SideScanSonar
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


def _wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


@pytest.fixture
def sonar_node():
    """Create a ROS node connected to two simulated sonars."""
    port = Omniscan450Simulator(port=0, channel_number=1).start()
    starboard = Omniscan450Simulator(port=0, channel_number=2).start()
    rclpy.init()

    node = listener = None
    try:
        node = SonarNode(
            'omniscan_test',
            parameter_overrides=[
                Parameter('sensor_number', value=999),
                Parameter('port_ip_address', value=port.address[0]),
                Parameter('port_port', value=port.address[1]),
                Parameter(
                    'starboard_ip_address', value=starboard.address[0]),
                Parameter('starboard_port', value=starboard.address[1]),
            ],
        )
        listener = rclpy.create_node('omniscan_test_listener')
        yield node, listener, port, starboard
    finally:
        if node is not None:
            node.close_sonars()
            node.destroy_node()
        if listener is not None:
            listener.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        port.stop()
        starboard.stop()


def test_node_publishes_complete_profiles_with_sensor_qos(sonar_node):
    node, listener, _, _ = sonar_node
    received = []
    listener.create_subscription(
        SideScanSonar, '/omniscan999/range', received.append,
        qos_profile_sensor_data)

    assert _wait_for(lambda: node.range_pub.get_subscription_count() == 1)
    for _ in range(10):
        assert node.read_and_publish()
        rclpy.spin_once(listener, timeout_sec=0.1)
        if received:
            break

    assert received
    msg = received[-1]
    assert msg.header.frame_id == 'sonar'
    assert len(msg.port_scaled_data) == msg.num_results == 600
    assert len(msg.starboard_scaled_data) == msg.num_results
    assert msg.port_transducer_heading_deg == pytest.approx(90.0)
    assert msg.starboard_transducer_heading_deg == pytest.approx(180.0)


def test_parameter_updates_reach_every_sonar_atomically(sonar_node):
    node, _, port, starboard = sonar_node

    result = node.set_parameters_atomically([
        Parameter('length_mm', value=7000),
        Parameter('frame_id', value='sidescan_link'),
    ])

    assert result.successful, result.reason
    assert _wait_for(
        lambda: port.length_mm == 7000 and starboard.length_mm == 7000)
    assert node._frame_id == 'sidescan_link'

    rejected = node.set_parameters_atomically([
        Parameter('frame_id', value=''),
    ])
    assert not rejected.successful
    assert node._frame_id == 'sidescan_link'


def test_persistent_profile_timeouts_trigger_restart(sonar_node):
    node, _, port, _ = sonar_node
    node._read_timeout = 0.05
    node._max_consecutive_failures = 2
    assert _wait_for(lambda: port.enabled)
    port.enabled = False

    with pytest.raises(SonarConnectionError):
        for _ in range(5):
            node.read_and_publish()
