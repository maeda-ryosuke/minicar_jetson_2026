"""SLAM用scanフィルタと、保存済み地図(pgm)でのAMCL自己位置推定を起動する。odomはホスト側が担当。

slam_localization.launch.py (slam_toolbox) の置き換え候補。map->odom を出すのはどちらか一方だけにする。
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_config = Path(get_package_share_directory("minicar_bringup")) / "config"
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    clock = {"use_sim_time": use_sim_time}
    initial_pose = {
        f"initial_pose.{k}": ParameterValue(LaunchConfiguration(f"initial_{k}"), value_type=float)
        for k in ("x", "y", "yaw")
    }
    map_yaml = {"yaml_filename": ParameterValue(LaunchConfiguration("map_yaml"), value_type=str)}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_yaml", default_value="/maps/track_v1.yaml"),
        DeclareLaunchArgument(
            "scan_params_file",
            default_value=str(bringup_config / "scan_filter_localization.yaml"),
        ),
        DeclareLaunchArgument(
            "amcl_params_file",
            default_value=str(bringup_config / "amcl_localization.yaml"),
        ),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        # 生/scan -> /scan_filtered。stampは入力をそのまま使う。
        Node(
            package="minicar_scan",
            executable="scan_filter_node",
            output="screen",
            parameters=[LaunchConfiguration("scan_params_file"), clock],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[LaunchConfiguration("amcl_params_file"), clock, map_yaml],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[LaunchConfiguration("amcl_params_file"), clock, initial_pose],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[clock, {"autostart": True, "node_names": ["map_server", "amcl"]}],
        ),
    ])
