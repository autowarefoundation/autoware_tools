import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

 
    save_file = LaunchConfiguration("save_file_path")

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
                    parameters=[{"save_file_path": save_file}],
                    extra_arguments=[]),
            ],
            output='both',
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
                "save_file_path",
                default_value="~/.ros/output.yaml",
        ),
        container
        ])