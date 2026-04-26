import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('rog_map')
    
    config_file = os.path.join(pkg_share_dir, 'config', 'rog_map.yaml')

    rm_node = Node(
        package='rog_map',
        executable='rog_map_node', 
        name='rm_node',
        output='log',                     
        parameters=[config_file]         
    )

    return LaunchDescription([
        rm_node,
    ])
    