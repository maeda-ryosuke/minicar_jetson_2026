"""同一コンテナのTG30、静的TF、SLAM用scanフィルタ、任意でD455を起動する。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('minicar_scan'))
    launch_file = package_share / 'launch/tg30.launch.py'
    d455_launch_file = (
        Path(get_package_share_directory('minicar_realsense')) / 'launch/d455.launch.py'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_params_file',
            default_value=str(package_share / 'config/scan_filter_params.yaml'),
        ),
        # D455未接続でもLiDAR系を起動できるよう既定は無効。
        DeclareLaunchArgument('enable_camera', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file))),
        Node(
            package='minicar_scan',
            executable='scan_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('scan_params_file'), {'use_sim_time': False}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(d455_launch_file)),
            condition=IfCondition(LaunchConfiguration('enable_camera')),
        ),
    ])
