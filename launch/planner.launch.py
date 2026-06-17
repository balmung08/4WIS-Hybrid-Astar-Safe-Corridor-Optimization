from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_demo_map = LaunchConfiguration("use_demo_map")
    use_rviz = LaunchConfiguration("use_rviz")
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("fourwis_hybrid_astar_cpp"), "config", "planner.yaml"]
    )
    config_file = LaunchConfiguration("config_file")
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("fourwis_hybrid_astar_cpp"), "rviz", "planner.rviz"]
    )
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_demo_map", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("config_file", default_value=default_config_file),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            LogInfo(msg=["Loading planner config: ", config_file]),
            Node(
                package="fourwis_hybrid_astar_cpp",
                executable="fourwis_demo_map_cpp",
                name="fourwis_demo_map_cpp",
                output="screen",
                condition=IfCondition(use_demo_map),
                parameters=[config_file],
            ),
            Node(
                package="fourwis_hybrid_astar_cpp",
                executable="fourwis_planner_cpp",
                name="fourwis_hybrid_astar_cpp",
                output="screen",
                parameters=[config_file],
            ),
            LogInfo(
                msg=["Starting RViz2 with config: ", rviz_config],
                condition=IfCondition(use_rviz),
            ),
            ExecuteProcess(
                cmd=["rviz2", "-d", rviz_config],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
