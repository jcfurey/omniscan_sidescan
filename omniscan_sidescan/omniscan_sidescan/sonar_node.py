# -*- coding: utf-8 -*-
"""Shared node implementation for the Omniscan 450 side scan sonars.

One base class drives both the two-sonar and the four-sonar node; they differ
only in which channels they own and which message types they publish.  Message
fields follow the convention ``<channel>_<field>`` (for example
``port_ping_number``), so the messages are filled from a table rather than by
hand.

Originally authored for ROS 1 by Monika Roznere (Binghamton University).
"""

from array import array

from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
import rclpy
from rclpy.node import Node

from .omniscan450_driver import Omniscan450Driver, SonarError, SonarParams

# Topic naming, kept identical to the ROS 1 package.
SENSOR_NAME = 'omniscan'
SONAR_TOPIC_NAME = 'range'
SONAR_RAW_TOPIC_NAME = 'range_raw'

DEFAULT_SENSOR_NUMBER = 450
DEFAULT_SONAR_PORT = 51200
DEFAULT_FRAME_ID = 'sonar'
DEFAULT_READ_TIMEOUT = 0.5

# Profile fields that are recorded once per sonar.
PER_CHANNEL_FIELDS = (
    'ping_number',
    'timestamp_ms',
    'ping_hz',
    'gain_index',
    'pulse_duration_sec',
    'analog_gain',
    'max_pwr_db',
    'min_pwr_db',
    'transducer_heading_deg',
)

# Profile fields recorded once per message, taken from the first channel.
# These describe the commanded scan region and the vehicle, so they are the
# same across sonars that share a configuration.
SHARED_FIELDS = (
    'start_mm',
    'length_mm',
    'num_results',
    'sos_dmps',
    'channel_number',
    'vehicle_heading_deg',
)

# Tunable parameters, mirroring the ROS 1 cfg/Omniscan450.cfg.  Each entry is
# (name, default, minimum, maximum, description); ints and floats are told
# apart by the type of the default.
TUNABLE_PARAMETERS = (
    ('speed_of_sound_mm', 1482000, 1482000, 1500000,
     'speed of sound in water, in mm/s'),
    ('start_mm', 0, 0, 5000,
     'start of ping range, in mm'),
    ('length_mm', 5000, 0, 300000,
     'length of returned profile; end of region = start_mm + length_mm'),
    ('msec_per_ping', 0, 0, 10,
     'set to limit the ping rate; 0 for the best ping rate'),
    ('pulse_len_percent', 0.002, 0.002, 0.004,
     '%% of total ping time for the current range'),
    ('filter_duration_percent', 0.0015, 0.0015, 0.0016,
     'typically 0.0015'),
    ('gain_index', -1, -1, 7,
     '-1 for auto gain, otherwise 0-7 sets the gain'),
    ('num_results', 600, 200, 1200,
     'number of signal data points in the returned profile'),
)


class SideScanSonarNode(Node):
    """Reads a set of Omniscan 450 sonars and publishes raw and scaled data.

    Subclasses set CHANNELS, SONAR_MSG and SONAR_RAW_MSG.
    """

    #: Sonar channels this node owns, in message field order.
    CHANNELS = ()
    #: Default address per channel, overridable with the <channel>_ip_address
    #: parameter.  Values come from the deployed BlueBoat, see notes.
    DEFAULT_IP_ADDRESSES = {}
    #: Message type carrying the scaled (dB) profiles.
    SONAR_MSG = None
    #: Message type carrying the raw u16 profiles.
    SONAR_RAW_MSG = None

    def __init__(self, node_name):
        super().__init__(node_name)

        self._declare_parameters()

        sensor_number = self.get_parameter('sensor_number').value
        self._frame_id = self.get_parameter('frame_id').value
        self._read_timeout = self.get_parameter('read_timeout').value

        sensor_name = '%s%d' % (SENSOR_NAME, sensor_number)
        self.range_raw_pub = self.create_publisher(
            self.SONAR_RAW_MSG, '%s/%s' % (sensor_name, SONAR_RAW_TOPIC_NAME),
            10)
        self.range_pub = self.create_publisher(
            self.SONAR_MSG, '%s/%s' % (sensor_name, SONAR_TOPIC_NAME), 10)

        params = self._current_sonar_params()

        # Connect every sonar before publishing anything; a partial set of
        # sonars would silently produce messages with stale channels.
        self.sonars = {}
        try:
            for channel in self.CHANNELS:
                ip_address = self.get_parameter('%s_ip_address' % channel).value
                port = self.get_parameter('%s_port' % channel).value
                self.get_logger().info(
                    'Connecting to %s sonar at %s:%d' % (channel, ip_address, port))
                self.sonars[channel] = Omniscan450Driver(
                    ip_address=ip_address, port=port, params=params,
                    name=channel, logger=self.get_logger())
        except SonarError:
            self.close_sonars()
            raise

        # Parameter updates arrive through this callback, which is the ROS 2
        # replacement for the ROS 1 dynamic_reconfigure server.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._reported_read_failure = False

    def _declare_parameters(self):
        """Declare every parameter this node accepts."""
        self.declare_parameter(
            'sensor_number', DEFAULT_SENSOR_NUMBER,
            ParameterDescriptor(
                description='sensor number used in the topic names',
                read_only=True))
        self.declare_parameter(
            'frame_id', DEFAULT_FRAME_ID,
            ParameterDescriptor(
                description='frame_id stamped on published messages'))
        self.declare_parameter(
            'read_timeout', DEFAULT_READ_TIMEOUT,
            ParameterDescriptor(
                description='seconds to wait for a profile from each sonar'))

        for channel in self.CHANNELS:
            self.declare_parameter(
                '%s_ip_address' % channel,
                self.DEFAULT_IP_ADDRESSES[channel],
                ParameterDescriptor(
                    description='address of the %s sonar' % channel,
                    read_only=True))
            self.declare_parameter(
                '%s_port' % channel, DEFAULT_SONAR_PORT,
                ParameterDescriptor(
                    description='TCP port of the %s sonar' % channel,
                    read_only=True))

        for name, default, minimum, maximum, description in TUNABLE_PARAMETERS:
            if isinstance(default, int):
                descriptor = ParameterDescriptor(
                    description=description,
                    integer_range=[IntegerRange(
                        from_value=minimum, to_value=maximum, step=1)])
            else:
                descriptor = ParameterDescriptor(
                    description=description,
                    floating_point_range=[FloatingPointRange(
                        from_value=minimum, to_value=maximum, step=0.0)])
            self.declare_parameter(name, default, descriptor)

    def _current_sonar_params(self, overrides=None):
        """Build a SonarParams from the declared parameters.

        Args:
            overrides: optional {name: value} taking precedence, used to
                preview a pending parameter change.
        """
        overrides = overrides or {}
        values = {}
        for name in SonarParams.field_names():
            values[name] = (overrides[name] if name in overrides
                            else self.get_parameter(name).value)
        return SonarParams(**values)

    def _on_set_parameters(self, parameters):
        """Push changed tunables to every sonar.

        Called before the new values are stored, so the pending values are
        merged over the current ones to build the parameter set to send.
        """
        tunable_names = set(SonarParams.field_names())
        overrides = {p.name: p.value for p in parameters
                     if p.name in tunable_names}

        for parameter in parameters:
            if parameter.name == 'frame_id':
                self._frame_id = parameter.value
            elif parameter.name == 'read_timeout':
                self._read_timeout = parameter.value

        if not overrides:
            return SetParametersResult(successful=True)

        params = self._current_sonar_params(overrides)
        try:
            for sonar in self.sonars.values():
                sonar.set_parameters(params)
        except (ConnectionError, OSError) as exception:
            return SetParametersResult(
                successful=False,
                reason='failed to send parameters to the sonars: %s' % exception)

        return SetParametersResult(successful=True)

    def read_and_publish(self):
        """Read one profile from every sonar and publish both messages.

        Returns True if a set of messages was published.
        """
        try:
            readings = {
                channel: sonar.get_data(timeout=self._read_timeout)
                for channel, sonar in self.sonars.items()
            }
        except SonarError as exception:
            # Throttle: the sonars drop the occasional profile, and logging
            # every miss at the ping rate would swamp the log.
            if not self._reported_read_failure:
                self.get_logger().error(
                    'Failed to read Omniscan 450 sonar data: %s' % exception)
                self._reported_read_failure = True
            return False

        if self._reported_read_failure:
            self.get_logger().info('Omniscan 450 sonar data recovered')
            self._reported_read_failure = False

        stamp = self.get_clock().now().to_msg()

        raw_msg = self.SONAR_RAW_MSG()
        msg = self.SONAR_MSG()

        for target in (raw_msg, msg):
            target.header.stamp = stamp
            target.header.frame_id = self._frame_id

        # Shared fields come from the first channel, as in the ROS 1 nodes.
        primary_data = readings[self.CHANNELS[0]][0]
        for field in SHARED_FIELDS:
            value = getattr(primary_data, field)
            setattr(raw_msg, field, value)
            setattr(msg, field, value)

        for channel in self.CHANNELS:
            data, scaled_results = readings[channel]

            for field in PER_CHANNEL_FIELDS:
                value = getattr(data, field)
                setattr(raw_msg, '%s_%s' % (channel, field), value)
                setattr(msg, '%s_%s' % (channel, field), value)

            # array.array lets rclpy take the fast path instead of validating
            # every one of the several hundred elements.
            setattr(raw_msg, '%s_pwr_data' % channel,
                    array('H', data.pwr_results))
            setattr(msg, '%s_scaled_data' % channel,
                    array('f', scaled_results))

        self.range_raw_pub.publish(raw_msg)
        self.range_pub.publish(msg)
        return True

    def close_sonars(self):
        """Stop pinging and disconnect every sonar."""
        for sonar in self.sonars.values():
            sonar.close_connection()
        self.sonars = {}


def run_sonar_node(node_class, node_name, args=None):
    """Spin a sonar node until shutdown, then disconnect the sonars.

    The read loop runs in the main thread and services callbacks between
    reads, so parameter updates never touch a sonar socket concurrently with
    a read.
    """
    rclpy.init(args=args)

    node = None
    try:
        node = node_class(node_name)
        while rclpy.ok():
            node.read_and_publish()
            # Service parameter callbacks without blocking the read loop.
            rclpy.spin_once(node, timeout_sec=0.0)
    except SonarError as exception:
        # Launch files respawn the node, matching the ROS 1 behaviour.
        if node is not None:
            node.get_logger().error(str(exception))
        else:
            print('Omniscan 450: %s' % exception)
        raise SystemExit(1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_sonars()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
