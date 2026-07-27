# -*- coding: utf-8 -*-
"""Bring up all four Omniscan 450 sonars."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

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
    'speed_of_sound_mm', 'start_mm', 'length_mm', 'msec_per_ping',
    'gain_index', 'num_results',
)

FLOAT_PARAMETERS = ('pulse_len_percent', 'filter_duration_percent')


def _parameters():
    """Map every launch argument onto a typed node parameter."""
    parameters = {}
    for name, _, _ in ARGUMENTS:
        configuration = LaunchConfiguration(name)
        if name in INTEGER_PARAMETERS:
            parameters[name] = PythonExpression(['int(', configuration, ')'])
        elif name in FLOAT_PARAMETERS:
            parameters[name] = PythonExpression(['float(', configuration, ')'])
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
            # The sonars are reached over the vehicle network, so a node that
            # starts before they are reachable exits and is retried.
            respawn=True,
            respawn_delay=30.0,
            parameters=[_parameters()],
        ),
    ])
