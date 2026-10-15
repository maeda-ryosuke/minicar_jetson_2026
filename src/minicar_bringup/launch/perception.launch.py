"""認識系(scanフィルタ + SLAM Toolbox)を一括起動する。

    ros2 launch minicar_bringup perception.launch.py                    # localization
    ros2 launch minicar_bringup perception.launch.py slam_mode:=mapping # 地図作成

TG30 などのセンサは compose の command (sensors.launch.py) が自動起動するので、
既定では起動しない。compose 外で使うときだけ with_sensors:=true にする
(二重に起動すると TG30 のシリアルポートを取り合う)。

ホスト側の micro-ROS Agent と EKF (odom -> base_link) はこの launch の外。
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SLAM_LAUNCH = {
    "localization": "slam_localization.launch.py",
    "mapping": "slam_mapping.launch.py",
}


def _slam(context):
    mode = LaunchConfiguration("slam_mode").perform(context)
    if mode not in SLAM_LAUNCH:
        # 綴り間違いで何も起動しないまま進まないよう、起動時に落とす。
        raise ValueError(f"slam_mode は {list(SLAM_LAUNCH)} のどれか: {mode!r}")
    launch_dir = Path(get_package_share_directory("minicar_bringup")) / "launch"
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / SLAM_LAUNCH[mode])),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "posegraph_file": LaunchConfiguration("posegraph_file"),
        }.items(),
    )]


def generate_launch_description():
    launch_dir = Path(get_package_share_directory("minicar_bringup")) / "launch"
    return LaunchDescription([
        DeclareLaunchArgument("slam_mode", default_value="localization"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        # localization のときだけ使う。拡張子は付けない。
        DeclareLaunchArgument("posegraph_file", default_value="/maps/track_v1"),
        DeclareLaunchArgument("with_sensors", default_value="false"),
        DeclareLaunchArgument("enable_camera", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "sensors.launch.py")),
            launch_arguments={"enable_camera": LaunchConfiguration("enable_camera")}.items(),
            condition=IfCondition(LaunchConfiguration("with_sensors")),
        ),
        OpaqueFunction(function=_slam),
    ])
