import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('livox_lidar_merge'),
        'config',
        'merge_params.yaml'
    )

    lidar_fusion_node = Node(
        package='livox_lidar_merge',
        executable='lidar_merge_node',
        name='lidar_merge_node',
        output='screen',
        parameters=[config_file]  
    )

    return LaunchDescription([
        lidar_fusion_node
    ])
