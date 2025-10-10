#!/usr/bin/env python3
"""
Complete Nav2 integration launch file with MPC controller.

This launch file sets up:
- Map server with a simple test map
- Nav2 planner (NavFn by default)
- MPC controller (our custom controller)
- Loopback simulator for testing without a real robot
- Lifecycle manager to manage all nodes
- RViz2 for visualization

Usage:
    ros2 launch time_constrained_mpc nav2_mpc_example.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get directories
    pkg_dir = get_package_share_directory('time_constrained_mpc')

    # Paths to configuration files
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    mpc_params_file = os.path.join(pkg_dir, 'config', 'mpc_params.yaml')
    map_file = os.path.join(pkg_dir, 'maps', 'map.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'nav2_view.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2')

    # Configure Nav2 parameters with map file path
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        },
        convert_types=True
    )

    # Map server
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[configured_params])

    # Planner server
    planner_server_node = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[configured_params])

    # Our MPC Controller
    mpc_controller_node = LifecycleNode(
        package='time_constrained_mpc',
        executable='time_constrained_mpc',
        name='mpc_controller',
        namespace='',
        output='screen',
        parameters=[mpc_params_file, {'use_sim_time': use_sim_time}],
        emulate_tty=True)

    # Lifecycle manager - manages all lifecycle nodes
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'map_server',
                'planner_server',
                'mpc_controller'
            ]}
        ])

    # Loopback simulator - simulates robot motion for testing
    loopback_simulator_node = Node(
        package='nav2_loopback_sim',
        executable='loopback_simulator',
        name='loopback_simulator',
        output='screen',
        parameters=[configured_params])

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        condition=IfCondition(use_rviz),
        output='screen')

    # Create the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add nodes
    ld.add_action(map_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(mpc_controller_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(loopback_simulator_node)
    ld.add_action(rviz_node)

    return ld
