# -*- coding: utf-8 -*-
"""Record sonar data to a ROS 2 bag.

The ROS 1 logger.launch enumerated every mavros topic by hand.  Here the
default is to record everything, which is what that list amounted to on the
vehicle; set ``all_topics`` to false and pass ``topics`` to narrow it down.
"""

import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def _selective_recorder(context):
    """Build a recorder command with each configured topic as one argument."""
    topics = shlex.split(LaunchConfiguration('topics').perform(context))
    if not topics:
        raise RuntimeError(
            'topics must contain at least one topic when all_topics is false')

    return [ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '--output',
            LaunchConfiguration('bag_name'), *topics,
        ],
        output='screen',
    )]


def generate_launch_description():
    bag_name = LaunchConfiguration('bag_name')
    all_topics = LaunchConfiguration('all_topics')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_name', default_value='omniscan_sidescan',
            description='output bag directory; must not already exist'),
        DeclareLaunchArgument(
            'all_topics', default_value='true',
            description='record every topic on the graph'),
        DeclareLaunchArgument(
            'topics',
            default_value='/omniscan450/range /omniscan450/range_raw',
            description='space separated topics to record when all_topics is false'),

        ExecuteProcess(
            condition=IfCondition(all_topics),
            cmd=['ros2', 'bag', 'record', '--all', '--output', bag_name],
            output='screen'),

        OpaqueFunction(
            condition=UnlessCondition(all_topics),
            function=_selective_recorder),
    ])
