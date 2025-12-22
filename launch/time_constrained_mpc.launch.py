import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('time_constrained_mpc')

    # Path to the parameters file
    params_file = os.path.join(pkg_dir, 'config', 'mpc_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Create the MPC controller lifecycle node
    mpc_node = LifecycleNode(
        package='time_constrained_mpc',
        executable='time_constrained_mpc',
        name='mpc_controller',
        namespace='',
        parameters=[params_file],
        output='screen',
        emulate_tty=True
    )

    # Lifecycle manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart' : True,
                'node_names': ['mpc_controller']
            }
        ])

    return LaunchDescription([
        use_sim_time_arg,
        mpc_node,
        lifecycle_manager_node
    ])
