from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_file = PathJoinSubstitution([
        FindPackageShare("merge_cloud"),
        "config",
        "merge_config.yaml"
    ])

    return LaunchDescription([
        Node(
            package='merge_cloud',
            executable='merge_cloud_node',
            name='merge_cloud_node',
            output='screen',
            parameters=[config_file],
        ),
    ])