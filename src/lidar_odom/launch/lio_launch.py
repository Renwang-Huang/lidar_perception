import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():

    use_rviz = LaunchConfiguration("use_rviz")

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )

    return launch.LaunchDescription(
        [
            # 声明参数
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Whether to start RViz"
            ),

            # fastlio2
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[{"config_path": config_path.perform(launch.LaunchContext())}]
            ),

            # RViz（带条件）
            launch_ros.actions.Node(
                package="rviz2",
                namespace="fastlio2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
                condition=IfCondition(use_rviz)
            ),
        ]
    )