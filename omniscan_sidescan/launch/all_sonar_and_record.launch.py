# -*- coding: utf-8 -*-
"""Bring up all four sonars and record them to a bag.

The ROS 1 version of this file also included ping_sonar's echosounder launch.
That package is not part of this repository, so it is left out here; include
it alongside this file if you run it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('omniscan_sidescan'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_name', default_value='omniscan_sidescan',
            description='output bag directory; must not already exist'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'sidescansonarall.launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'logger.launch.py')),
            launch_arguments={
                'bag_name': LaunchConfiguration('bag_name'),
            }.items()),
    ])
