#!/usr/bin/env python3
"""
RViz2 launch file for debugging the MPC controller.

This launch file sets up RViz2 with debug path topics and correct tf frames already configured

Usage:
    ros2 launch time_constrained_mpc rviz2_debug.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get directories
    pkg_dir = get_package_share_directory('time_constrained_mpc')

    # Paths to configuration files
    rviz_config_file = os.path.join(pkg_dir, 'config', 'example_view.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # RViz2
    rviz_args = ['-d', rviz_config_file] if os.path.exists(
        rviz_config_file) else []
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen')

    # Create the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes
    ld.add_action(rviz_node)

    return ld
