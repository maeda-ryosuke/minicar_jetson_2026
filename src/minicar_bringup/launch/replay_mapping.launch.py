"""rosbag再生用。生/scanからscan_filterとmapping SLAMをsim timeで起動する。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("minicar_bringup"))
    scan_share = Path(get_package_share_directory("minicar_scan"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "scan_params_file",
            default_value=str(scan_share / "config/scan_filter_params.yaml"),
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=str(bringup_share / "config/slam_toolbox_mapping.yaml"),
        ),
        DeclareLaunchArgument("interactive_mode", default_value="false"),
        # bagの/scan_filteredは使わず、条件ごとのパラメータで再生成する。
        Node(
            package="minicar_scan",
            executable="scan_filter_node",
            output="screen",
            parameters=[LaunchConfiguration("scan_params_file"), {"use_sim_time": True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup_share / "launch/slam_mapping.launch.py")),
            launch_arguments={
                "use_sim_time": "true",
                "slam_params_file": LaunchConfiguration("slam_params_file"),
                "interactive_mode": LaunchConfiguration("interactive_mode"),
            }.items(),
        ),
    ])
