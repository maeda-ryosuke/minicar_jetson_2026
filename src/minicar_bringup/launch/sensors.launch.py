"""同一コンテナのTG30、静的TF、SLAM用scanフィルタを起動する。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('minicar_scan'))
    launch_file = package_share / 'launch/tg30.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_params_file',
            default_value=str(package_share / 'config/scan_filter_params.yaml'),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file))),
        Node(
            package='minicar_scan',
            executable='scan_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('scan_params_file'), {'use_sim_time': False}],
        ),
    ])
