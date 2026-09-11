"""scan フィルタと SLAM のみ起動。odom・センサ TF はホストが配信する。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    clock = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="minicar_scan", executable="scan_filter_node", output="screen",
             parameters=[str(Path(get_package_share_directory("minicar_scan")) /
                             "config/scan_filter_params.yaml"), clock]),
        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
             name="slam_toolbox", output="screen",
             parameters=[str(Path(get_package_share_directory("minicar_bringup")) /
                             "config/slam_toolbox_mapping.yaml"), clock]),
    ])
