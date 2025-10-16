import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('time_constrained_mpc')

    # Path to the parameters file
    params_file = os.path.join(pkg_dir, 'config', 'mpc_params.yaml')

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

    # Event to configure the node
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=(
                lambda node: node.name == 'mpc_controller'),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Event to activate the node after configuration
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mpc_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: (
                            node.name == 'mpc_controller'
                        ),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        mpc_node,
        configure_event,
        activate_event,
    ])
