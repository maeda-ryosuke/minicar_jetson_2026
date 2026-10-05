"""SLAM用scanフィルタとmapping用SLAMを起動する。bag再生時はuse_sim_time:=true。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_config = Path(get_package_share_directory("minicar_bringup")) / "config"
    scan_config = Path(get_package_share_directory("minicar_scan")) / "config"
    clock = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    interactive = {
        "enable_interactive_mode": ParameterValue(
            LaunchConfiguration("interactive_mode"), value_type=bool)
    }
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("interactive_mode", default_value="false"),
        DeclareLaunchArgument(
            "scan_params_file",
            default_value=str(scan_config / "scan_filter_params.yaml"),
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=str(bringup_config / "slam_toolbox_mapping.yaml"),
        ),
        # 生/scan -> /scan_filtered。stampは入力をそのまま使う。
        Node(package="minicar_scan", executable="scan_filter_node", output="screen",
             parameters=[LaunchConfiguration("scan_params_file"), clock]),
        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
             name="slam_toolbox", output="screen",
             parameters=[LaunchConfiguration("slam_params_file"), clock, interactive]),
    ])
