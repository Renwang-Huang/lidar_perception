from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_file = PathJoinSubstitution([
        FindPackageShare("dataconvert"),
        "config",
        "dataconvert_config.yaml"
    ])

    return LaunchDescription([
        Node(
            package='dataconvert',
            executable='dataconvert_node',   
            name='livox_to_pointcloud2',     
            output='screen',
            parameters=[config_file],
        ),
    ])