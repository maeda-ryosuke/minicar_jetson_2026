"""SLAM用scanフィルタと、保存済みpose graphでの自己位置推定を起動する。odomはホスト側が担当。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_config = Path(get_package_share_directory("minicar_bringup")) / "config"
    clock = {
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool
        )
    }
    posegraph = {
        "map_file_name": ParameterValue(
            LaunchConfiguration("posegraph_file"), value_type=str
        )
    }
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("posegraph_file", default_value="/maps/track_v1"),
        DeclareLaunchArgument(
            "scan_params_file",
            default_value=str(bringup_config / "scan_filter_localization.yaml"),
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=str(bringup_config / "slam_toolbox_localization.yaml"),
        ),
        # 生/scan -> /scan_filtered。stampは入力をそのまま使う。
        Node(
            package="minicar_scan",
            executable="scan_filter_node",
            output="screen",
            parameters=[LaunchConfiguration("scan_params_file"), clock],
        ),
        Node(
            package="slam_toolbox",
            executable="localization_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[LaunchConfiguration("slam_params_file"), clock, posegraph],
        ),
    ])
