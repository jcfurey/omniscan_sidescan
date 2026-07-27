# -*- coding: utf-8 -*-
"""Record sonar data to a ROS 2 bag.

The ROS 1 logger.launch enumerated every mavros topic by hand.  Here the
default is to record everything, which is what that list amounted to on the
vehicle; set ``all_topics`` to false and pass ``topics`` to narrow it down.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag_name = LaunchConfiguration('bag_name')
    topics = LaunchConfiguration('topics')
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

        ExecuteProcess(
            condition=UnlessCondition(all_topics),
            cmd=['ros2', 'bag', 'record', '--output', bag_name, topics],
            output='screen',
            shell=True),
    ])
