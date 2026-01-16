import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='pose_replay_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pose_replay',
                    plugin='PoseReplayNode',
                    name='pose_replay_node',
                    remappings=[],
                    parameters=[],
                    extra_arguments=[]),
            ],
            output='both',
    )

    return launch.LaunchDescription([container])