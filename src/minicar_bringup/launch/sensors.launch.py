"""同一コンテナのTG30(生/scan)、静的TF、任意でD455を起動する。

SLAM用のscanフィルタはslam_*.launch.pyが起動する。bagには生/scanだけを残し、
再生時にフィルタ条件を変えられるようにするため。
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file = Path(get_package_share_directory('minicar_scan')) / 'launch/tg30.launch.py'
    d455_launch_file = (
        Path(get_package_share_directory('minicar_realsense')) / 'launch/d455.launch.py'
    )
    return LaunchDescription([
        # D455未接続でもLiDAR系を起動できるよう既定は無効。
        DeclareLaunchArgument('enable_camera', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(d455_launch_file)),
            condition=IfCondition(LaunchConfiguration('enable_camera')),
        ),
    ])
