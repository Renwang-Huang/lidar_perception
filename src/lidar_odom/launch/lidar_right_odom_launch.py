import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():

    use_rviz = LaunchConfiguration("use_rviz")

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("lidar_odom"), "rviz", "lidar_odom.rviz"]
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("lidar_odom"), "config", "lidar_right_odom.yaml"]
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to start RViz"
            ),

            launch_ros.actions.Node(
                package="lidar_odom",
                namespace="lidar_odom1",
                executable="odom_node",
                name="lidar_odom1",
                output="screen",
                parameters=[{"config_path": config_path.perform(launch.LaunchContext())}]
            ),

            launch_ros.actions.Node(
                package="rviz2",
                namespace="lidar_odom1",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
                condition=IfCondition(use_rviz)
            ),
        ]
    )
