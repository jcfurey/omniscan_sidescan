# -*- coding: utf-8 -*-
"""Publish independent sonar profiles and compatible complete combined sets."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from omniscan_sidescan_msgs.msg import SonarProfile
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from .omniscan450_driver import PARAMETER_LIMITS, SonarError, SonarParams
from .sonar_channel import SonarChannel

SENSOR_NAME = 'omniscan'
DEFAULT_SENSOR_NUMBER = 450
DEFAULT_SONAR_PORT = 51200
DEFAULT_FRAME_ID = 'sonar'

PER_CHANNEL_FIELDS = (
    'ping_number', 'timestamp_ms', 'ping_hz', 'gain_index',
    'pulse_duration_sec', 'analog_gain', 'max_pwr_db', 'min_pwr_db',
    'transducer_heading_deg',
)
SHARED_FIELDS = (
    'start_mm', 'length_mm', 'num_results', 'sos_dmps', 'channel_number',
    'vehicle_heading_deg',
)
CONSISTENT_FIELDS = ('start_mm', 'length_mm', 'num_results', 'sos_dmps')

TUNABLE_PARAMETERS = (
    ('speed_of_sound_mm', 1482000, 'speed of sound in water, in mm/s'),
    ('start_mm', 0, 'start of ping range, in mm'),
    ('length_mm', 5000, 'length of returned profile, in mm'),
    ('msec_per_ping', 0, 'minimum ping period in milliseconds; zero is fastest'),
    ('pulse_len_percent', 0.002, 'pulse length as a fraction of ping time'),
    ('filter_duration_percent', 0.0015, 'filter duration fraction'),
    ('gain_index', -1, '-1 for auto gain, otherwise 0-7'),
    ('num_results', 600, 'number of signal data points in each profile'),
)


class SideScanSonarNode(Node):
    """Keep each channel available independently of its peers."""

    CHANNELS = ()
    DEFAULT_IP_ADDRESSES = {}
    SONAR_MSG = None
    SONAR_RAW_MSG = None

    def __init__(self, node_name, **kwargs):
        """Create ROS resources, then start independent connection workers."""
        super().__init__(node_name, **kwargs)
        self.sonars = {}
        self._closed = False
        try:
            self._initialize()
        except Exception:
            self.destroy_node()
            raise

    def _initialize(self):
        self._declare_parameters()
        self._frame_id = self.get_parameter('frame_id').value
        if not self._frame_id.strip():
            raise ValueError('frame_id must not be empty')
        initial_delay = self.get_parameter('reconnect_initial_delay').value
        max_delay = self.get_parameter('reconnect_max_delay').value
        if initial_delay > max_delay:
            raise ValueError('reconnect_initial_delay must not exceed reconnect_max_delay')
        self._desired_params = self._current_sonar_params()
        self._generation = 0
        self._read_timeout = self.get_parameter('read_timeout').value
        self._max_profile_age = self.get_parameter('max_profile_age').value
        self._max_profile_skew = self.get_parameter('max_profile_skew').value
        self._last_profile_sequences = dict.fromkeys(self.CHANNELS, -1)
        self._last_combined_sequences = dict.fromkeys(self.CHANNELS, -1)
        self._combined_status = 'waiting for complete profiles'

        sensor_name = '%s%d' % (
            SENSOR_NAME, self.get_parameter('sensor_number').value)
        self.range_raw_pub = self.create_publisher(
            self.SONAR_RAW_MSG, sensor_name + '/range_raw', qos_profile_sensor_data)
        self.range_pub = self.create_publisher(
            self.SONAR_MSG, sensor_name + '/range', qos_profile_sensor_data)
        self.profile_pubs = {
            channel: self.create_publisher(
                SonarProfile, '%s/%s/profile' % (sensor_name, channel),
                qos_profile_sensor_data)
            for channel in self.CHANNELS
        }
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self._profile_guard = self.create_guard_condition(self.read_and_publish)
        self.create_timer(1.0, self.publish_diagnostics)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.add_post_set_parameters_callback(self._post_set_parameters)
        clock = self.get_clock()
        for channel in self.CHANNELS:
            ip_address = self.get_parameter(channel + '_ip_address').value
            if not ip_address.strip():
                raise ValueError(channel + '_ip_address must not be empty')
            if not self.get_parameter(channel + '_frame_id').value.strip():
                raise ValueError(channel + '_frame_id must not be empty')
            self.sonars[channel] = SonarChannel(
                channel, ip_address.strip(),
                self.get_parameter(channel + '_port').value, self._desired_params,
                read_timeout=self._read_timeout,
                connect_timeout=self.get_parameter('connect_timeout').value,
                configuration_timeout=self.get_parameter('configuration_timeout').value,
                max_consecutive_failures=self.get_parameter(
                    'max_consecutive_failures').value,
                reconnect_initial_delay=initial_delay, reconnect_max_delay=max_delay,
                logger=self.get_logger(), clock_ns=lambda: clock.now().nanoseconds,
                notify=self._notify_profiles)
        for sonar in self.sonars.values():
            sonar.start()

    def _notify_profiles(self):
        # Workers only trigger the guard. All ROS publication stays in the
        # executor, and workers are joined before the guard is destroyed.
        if not self._closed and self.context.ok():
            self._profile_guard.trigger()

    def _declare_parameters(self):
        self.declare_parameter(
            'sensor_number', DEFAULT_SENSOR_NUMBER,
            ParameterDescriptor(
                description='sensor number used in topic names', read_only=True,
                integer_range=[IntegerRange(from_value=0, to_value=65535, step=1)]))
        self.declare_parameter(
            'frame_id', DEFAULT_FRAME_ID,
            ParameterDescriptor(description='rig frame for the combined messages'))
        durations = (
            ('read_timeout', 0.5, False, 'profile read deadline for one channel'),
            ('connect_timeout', 5.0, True, 'TCP connection deadline for one channel'),
            ('configuration_timeout', 2.0, True, 'profile configuration confirmation deadline'),
            ('reconnect_initial_delay', 0.25, True, 'initial reconnect backoff'),
            ('reconnect_max_delay', 10.0, True, 'maximum reconnect backoff'),
            ('max_profile_age', 0.5, False, 'oldest host receive age permitted for publication'),
            ('max_profile_skew', 0.05, False, 'maximum host receive skew of a combined set'),
        )
        for name, default, read_only, description in durations:
            self.declare_parameter(
                name, default, ParameterDescriptor(
                    description=description + ', in seconds', read_only=read_only,
                    floating_point_range=[FloatingPointRange(
                        from_value=0.001, to_value=60.0, step=0.0)]))
        self.declare_parameter(
            'max_consecutive_failures', 10,
            ParameterDescriptor(
                description='profile failures before reconnecting this channel; zero disables',
                read_only=True,
                integer_range=[IntegerRange(from_value=0, to_value=100000, step=1)]))
        for channel in self.CHANNELS:
            self.declare_parameter(
                channel + '_ip_address', self.DEFAULT_IP_ADDRESSES[channel],
                ParameterDescriptor(description='device address', read_only=True))
            self.declare_parameter(
                channel + '_port', DEFAULT_SONAR_PORT,
                ParameterDescriptor(
                    description='device TCP port', read_only=True,
                    integer_range=[IntegerRange(from_value=1, to_value=65535, step=1)]))
            self.declare_parameter(
                channel + '_frame_id', DEFAULT_FRAME_ID + '_' + channel,
                ParameterDescriptor(description='physical sonar frame', read_only=True))
        for name, default, description in TUNABLE_PARAMETERS:
            minimum, maximum = PARAMETER_LIMITS[name]
            if isinstance(default, int):
                descriptor = ParameterDescriptor(
                    description=description, integer_range=[IntegerRange(
                        from_value=minimum, to_value=maximum, step=1)])
            else:
                descriptor = ParameterDescriptor(
                    description=description, floating_point_range=[FloatingPointRange(
                        from_value=minimum, to_value=maximum, step=0.0)])
            self.declare_parameter(name, default, descriptor)

    def _current_sonar_params(self, overrides=None):
        overrides = overrides or {}
        return SonarParams(**{
            name: overrides[name] if name in overrides else self.get_parameter(name).value
            for name in SonarParams.field_names()
        })

    def _on_set_parameters(self, parameters):
        """Validate proposed values without touching hardware or cached state."""
        overrides = {p.name: p.value for p in parameters
                     if p.name in SonarParams.field_names()}
        try:
            self._current_sonar_params(overrides)
            for parameter in parameters:
                if parameter.name == 'frame_id' and not parameter.value.strip():
                    raise ValueError('frame_id must not be empty')
                if parameter.name in ('read_timeout', 'max_profile_age', 'max_profile_skew'):
                    if not math.isfinite(parameter.value) or parameter.value <= 0:
                        raise ValueError(parameter.name + ' must be finite and positive')
        except (TypeError, ValueError) as exception:
            return SetParametersResult(successful=False, reason=str(exception))
        return SetParametersResult(successful=True)

    def _post_set_parameters(self, parameters):
        """Queue only committed desired settings; device status is asynchronous."""
        params = self._current_sonar_params()
        if params != self._desired_params:
            self._generation += 1
            self._desired_params = params
        self._frame_id = self.get_parameter('frame_id').value
        self._read_timeout = self.get_parameter('read_timeout').value
        self._max_profile_age = self.get_parameter('max_profile_age').value
        self._max_profile_skew = self.get_parameter('max_profile_skew').value
        for sonar in self.sonars.values():
            sonar.configure(params, self._generation, self._read_timeout)

    def _fresh_sample(self, snapshot, now):
        sample = snapshot.sample
        if snapshot.state != 'streaming' or sample is None \
                or sample.generation != self._generation \
                or snapshot.applied_generation != self._generation \
                or not 0 <= now - sample.received_monotonic <= self._max_profile_age:
            return None
        return sample

    def _combined_problem(self, samples):
        if len(samples) != len(self.CHANNELS):
            return 'waiting for fresh confirmed profiles from every channel'
        reference = samples[self.CHANNELS[0]].data
        if any(any(getattr(sample.data, field) != getattr(reference, field)
                   for field in CONSISTENT_FIELDS) for sample in samples.values()):
            return 'channel geometry differs'
        arrivals = [sample.received_monotonic for sample in samples.values()]
        if max(arrivals) - min(arrivals) > self._max_profile_skew:
            return 'channel receive skew exceeds max_profile_skew'
        return None

    def read_and_publish(self):
        """Publish new per-channel samples and, when valid, one complete set."""
        if self._closed:
            return False
        snapshots = {channel: sonar.snapshot() for channel, sonar in self.sonars.items()}
        now = time.monotonic()
        samples = {}
        for channel, snapshot in snapshots.items():
            sample = self._fresh_sample(snapshot, now)
            if sample is None:
                continue
            samples[channel] = sample
            if sample.sequence != self._last_profile_sequences[channel]:
                message = SonarProfile()
                message.header.stamp = Time(nanoseconds=sample.timestamp_ns).to_msg()
                message.header.frame_id = self.get_parameter(channel + '_frame_id').value
                message.configuration_generation = sample.generation
                for field in (*SHARED_FIELDS, *PER_CHANNEL_FIELDS):
                    setattr(message, field, getattr(sample.data, field))
                message.pwr_data = sample.data.pwr_results
                message.scaled_data = sample.scaled
                self.profile_pubs[channel].publish(message)
                self._last_profile_sequences[channel] = sample.sequence

        problem = self._combined_problem(samples)
        if problem is not None:
            self._combined_status = problem
            return False
        if any(samples[c].sequence == self._last_combined_sequences[c]
               for c in self.CHANNELS):
            return False
        reference = samples[self.CHANNELS[0]].data
        oldest = min(samples.values(), key=lambda sample: sample.received_monotonic)

        raw_msg, msg = self.SONAR_RAW_MSG(), self.SONAR_MSG()
        for target in (raw_msg, msg):
            target.header.stamp = Time(nanoseconds=oldest.timestamp_ns).to_msg()
            target.header.frame_id = self._frame_id
            for field in SHARED_FIELDS:
                setattr(target, field, getattr(reference, field))
        for channel, sample in samples.items():
            for field in PER_CHANNEL_FIELDS:
                value = getattr(sample.data, field)
                setattr(raw_msg, channel + '_' + field, value)
                setattr(msg, channel + '_' + field, value)
            setattr(raw_msg, channel + '_pwr_data', sample.data.pwr_results)
            setattr(msg, channel + '_scaled_data', sample.scaled)
        self.range_raw_pub.publish(raw_msg)
        self.range_pub.publish(msg)
        self._last_combined_sequences = {c: sample.sequence for c, sample in samples.items()}
        self._combined_status = 'complete profiles within age and receive skew limits'
        return True

    def publish_diagnostics(self):
        """Expose device health and requested versus profile-verified settings."""
        if self._closed:
            return
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        snapshots = {channel: sonar.snapshot() for channel, sonar in self.sonars.items()}
        now = time.monotonic()
        fresh = {}
        for channel, sonar in self.sonars.items():
            snapshot = snapshots[channel]
            sample = self._fresh_sample(snapshot, now)
            healthy = sample is not None
            if healthy:
                fresh[channel] = sample
            description = snapshot.error or snapshot.state
            if not healthy and not snapshot.error and snapshot.state == 'streaming':
                description = 'profile is stale or configuration is pending'
            status = DiagnosticStatus(
                name=self.get_fully_qualified_name() + '/' + channel,
                hardware_id='tcp://%s:%d' % (sonar.ip_address, sonar.port),
                level=DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR,
                message=description)
            values = {
                'state': snapshot.state,
                'desired_generation': self._generation,
                'applied_generation': snapshot.applied_generation,
                'connection_attempts': snapshot.connection_attempts,
                'connections': snapshot.connections,
                'profiles_received': snapshot.profiles_received,
                'duplicate_profiles': snapshot.duplicate_profiles,
                'parser_errors': snapshot.parser_errors,
                'confirmation': 'profile geometry, speed of sound, and manual gain',
            }
            if snapshot.sample is not None:
                values['profile_age_sec'] = now - snapshot.sample.received_monotonic
                for field in (*CONSISTENT_FIELDS, 'gain_index'):
                    values['reported.' + field] = getattr(snapshot.sample.data, field)
            for name in SonarParams.field_names():
                values['desired.' + name] = getattr(self._desired_params, name)
                if snapshot.applied_params is not None:
                    values['commanded.' + name] = getattr(snapshot.applied_params, name)
            status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
            message.status.append(status)
        problem = self._combined_problem(fresh)
        message.status.append(DiagnosticStatus(
            name=self.get_fully_qualified_name() + '/combined',
            level=DiagnosticStatus.WARN if problem else DiagnosticStatus.OK,
            message=problem or 'complete profiles within age and receive skew limits'))
        self.diagnostics_pub.publish(message)

    def close_sonars(self):
        """Cancel every worker before joining any of them."""
        self._closed = True
        for sonar in self.sonars.values():
            sonar.stop()
        for sonar in self.sonars.values():
            sonar.join()
        self.sonars = {}

    def destroy_node(self):
        """Join workers before their clock, logger, and guard are destroyed."""
        self.close_sonars()
        return super().destroy_node()


def run_sonar_node(node_class, node_name, args=None):
    """Spin the executor while independent workers handle sonar sockets."""
    rclpy.init(args=args)
    node = None
    try:
        node = node_class(node_name)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except SonarError as exception:
        if node is not None:
            node.get_logger().error(str(exception))
        raise SystemExit(1) from exception
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
