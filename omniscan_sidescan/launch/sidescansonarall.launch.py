# -*- coding: utf-8 -*-
"""Bring up all four Omniscan 450 sonars."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = (
    ('sensor_number', '450', 'sensor number used in the topic names'),
    ('port_ip_address', '192.168.2.94', 'address of the port sonar'),
    ('port_port', '51200', 'TCP port of the port sonar'),
    ('starboard_ip_address', '192.168.2.95', 'address of the starboard sonar'),
    ('starboard_port', '51200', 'TCP port of the starboard sonar'),
    ('bow_ip_address', '192.168.2.93', 'address of the bow sonar'),
    ('bow_port', '51200', 'TCP port of the bow sonar'),
    ('stern_ip_address', '192.168.2.92', 'address of the stern sonar'),
    ('stern_port', '51200', 'TCP port of the stern sonar'),
    ('frame_id', 'sonar', 'frame_id stamped on published messages'),
    ('port_frame_id', 'sonar_port', 'physical frame of the port sonar'),
    ('starboard_frame_id', 'sonar_starboard', 'physical frame of the starboard sonar'),
    ('bow_frame_id', 'sonar_bow', 'physical frame of the bow sonar'),
    ('stern_frame_id', 'sonar_stern', 'physical frame of the stern sonar'),
    ('connect_timeout', '5.0', 'seconds to wait for each TCP connection'),
    ('configuration_timeout', '2.0', 'seconds to confirm profile settings'),
    ('read_timeout', '0.5', 'seconds to wait for each sonar profile'),
    ('reconnect_initial_delay', '0.25', 'initial reconnect backoff in seconds'),
    ('reconnect_max_delay', '10.0', 'maximum reconnect backoff in seconds'),
    ('max_profile_age', '0.5', 'maximum host receive age in seconds'),
    ('max_profile_skew', '0.05', 'maximum combined host receive skew in seconds'),
    ('max_consecutive_failures', '10',
     'profile failures before reconnecting one channel; 0 disables'),
    ('speed_of_sound_mm', '1482000', 'speed of sound in water, in mm/s'),
    ('start_mm', '0', 'start of ping range, in mm'),
    ('length_mm', '6000', 'length of the returned profile, in mm'),
    ('msec_per_ping', '0', 'set to limit the ping rate; 0 for the best rate'),
    ('pulse_len_percent', '0.002', 'pulse length, as a fraction of ping time'),
    ('filter_duration_percent', '0.0015', 'filter duration fraction'),
    ('gain_index', '-1', '-1 for auto gain, otherwise 0-7'),
    ('num_results', '600', 'signal data points in the returned profile'),
)

# Parameters typed as integers on the node; launch arguments are strings, so
# they are converted before being handed over.
INTEGER_PARAMETERS = (
    'sensor_number', 'port_port', 'starboard_port', 'bow_port', 'stern_port',
    'max_consecutive_failures', 'speed_of_sound_mm', 'start_mm', 'length_mm',
    'msec_per_ping', 'gain_index', 'num_results',
)

FLOAT_PARAMETERS = (
    'connect_timeout', 'configuration_timeout', 'read_timeout', 'pulse_len_percent',
    'filter_duration_percent', 'reconnect_initial_delay', 'reconnect_max_delay',
    'max_profile_age', 'max_profile_skew',
)


def _parameters():
    """Map every launch argument onto a typed node parameter."""
    parameters = {}
    for name, _, _ in ARGUMENTS:
        configuration = LaunchConfiguration(name)
        if name in INTEGER_PARAMETERS:
            parameters[name] = ParameterValue(configuration, value_type=int)
        elif name in FLOAT_PARAMETERS:
            parameters[name] = ParameterValue(configuration, value_type=float)
        else:
            parameters[name] = configuration
    return parameters


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in ARGUMENTS],
        Node(
            package='omniscan_sidescan',
            executable='sidescansonarall_node',
            name=['omniscan', LaunchConfiguration('sensor_number'), 'all'],
            output='screen',
            emulate_tty=True,
            # Device outages recover inside their own connection workers.
            # Respawn is a fallback for process failures.
            respawn=True,
            respawn_delay=30.0,
            parameters=[_parameters()],
        ),
    ])
