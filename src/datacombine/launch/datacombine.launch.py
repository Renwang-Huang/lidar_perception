from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_file = PathJoinSubstitution([
        FindPackageShare("datacombine"),
        "config",
        "datacombine_config.yaml"
    ])

    return LaunchDescription([
        Node(
            package='datacombine',
            executable='data_combine_node',  
            name='data_combine_node',        
            output='screen',
            parameters=[config_file],
        ),
    ])