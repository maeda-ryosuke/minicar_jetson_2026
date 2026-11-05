"""ヘアピンだけ速度帯を下げる Nav2 MPPI レースライン追従を一括起動する。

minicar_nav2 の raceline_mppi.launch.py と同じ構成に speed_zone を足したもの。
MPPI の設定は minicar_nav2 の config/nav2_mppi_params.yaml をそのまま使い
(二重管理しない)、FollowPath.vx_max / vx_min だけを speed_zone_params.yaml の
fast_* で上書きして起動する。走行中は speed_zone がヘアピンの矩形の中で
slow_* へ、外で fast_* へ切り替える。

  raceline --FollowPath--> controller_server --/cmd_vel_raw-->
      safety --/cmd_vel--> motor_driver
  speed_zone --set_parameters(vx_max, vx_min)--> controller_server

前提(raceline_mppi.launch.py と同じ):
  * robot_localization: /odometry/filtered と odom -> base_link
  * slam_toolbox: map -> odom
  * LiDAR / IMU: /scan と /imu/data

    ros2 launch minicar_nav2_zone zone_mppi.launch.py \\
      raceline_file:=/maps/raceline.csv

nav2_params_file / speed_zone_params_file はファイル名だけなら、それぞれ
minicar_nav2/config/ と minicar_nav2_zone/config/ の下を探す。

use_safety:=false / use_raceline:=false で safety_node / raceline_manager を
起動しない。停止検知と raceline を別に起動する launch(minicar_bringup の
race_zone.launch.py)から include するときに使う。

minicar_nav2 の raceline_mppi.launch.py / nav2_mppi.launch.py と同時に起動しないこと
(controller_server が二重になる)。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from minicar_nav2.vehicle_limits import min_turning_radius


def _resolve(context, name, package):
    """launch 引数 name を実在するパスにする。ファイル名だけなら package の config/ を探す。"""
    value = LaunchConfiguration(name).perform(context).strip()
    if "/" not in value:
        value = str(Path(get_package_share_directory(package)) / "config" / value)
    if not Path(value).is_file():
        raise FileNotFoundError(f"{name} が見つからない: {value}")
    return value


def _fast_band(speed_zone_params_file):
    with open(speed_zone_params_file) as f:
        p = yaml.safe_load(f)["speed_zone"]["ros__parameters"]
    return float(p["fast_vx_max"]), float(p["fast_vx_min"])


def _mppi_and_zone(context):
    """vehicle_params の旋回制約と speed_zone の fast 速度帯で controller_server を起動する。"""
    nav2_params_file = _resolve(context, "nav2_params_file", "minicar_nav2")
    zone_params_file = _resolve(context, "speed_zone_params_file", "minicar_nav2_zone")
    vehicle_params_file = LaunchConfiguration("vehicle_params_file").perform(context)
    r_min = min_turning_radius(vehicle_params_file)
    vx_max, vx_min = _fast_band(zone_params_file)
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    autostart = ParameterValue(LaunchConfiguration("autostart"), value_type=bool)
    visualize = LaunchConfiguration("visualize").perform(context).lower() == "true"
    trajectory_step = LaunchConfiguration("trajectory_step").perform(context)
    viz = {"visualize": visualize}
    if trajectory_step:
        viz["TrajectoryVisualizer"] = {"trajectory_step": int(trajectory_step)}
    return [
        LogInfo(msg=f"MPPI params = {nav2_params_file}"),
        LogInfo(msg=f"speed_zone params = {zone_params_file}"),
        LogInfo(msg=f"MPPI min_turning_r = {r_min:.3f} m "
                    f"(wheelbase / tan(delta_max), {vehicle_params_file})"),
        LogInfo(msg=f"MPPI 起動時の速度帯 vx {vx_min}..{vx_max} ({zone_params_file})"),
        LogInfo(msg=f"MPPI visualize = {visualize}, "
                    f"trajectory_step = {trajectory_step or 'yaml'}"),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[
                nav2_params_file,
                {"use_sim_time": sim_time},
                # 後ろほど優先される。yaml の値より vehicle_params を正とする。
                {"FollowPath": {"AckermannConstraints": {"min_turning_r": r_min}}},
                # 起動直後は直線用の速度帯。以降は speed_zone が切り替える。
                {"FollowPath": {"vx_max": vx_max, "vx_min": vx_min}},
                {"FollowPath": viz},
            ],
            # 停止検知フィルタ(safety_node か stall_stop_node)だけが最終 /cmd_vel を
            # publishする。
            remappings=[("cmd_vel", "/cmd_vel_raw")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_nav2_mppi",
            output="screen",
            parameters=[
                nav2_params_file,
                {"use_sim_time": sim_time,
                 "autostart": autostart,
                 "node_names": ["controller_server"]},
            ],
        ),
        Node(
            package="minicar_nav2_zone",
            executable="speed_zone_node",
            name="speed_zone",
            output="screen",
            parameters=[
                zone_params_file,
                {"use_sim_time": sim_time},
            ],
        ),
    ]


def generate_launch_description():
    nav2_share = Path(get_package_share_directory("minicar_nav2"))
    zone_share = Path(get_package_share_directory("minicar_nav2_zone"))
    bringup_share = Path(get_package_share_directory("minicar_bringup"))
    safety_share = Path(get_package_share_directory("minicar_safety"))
    raceline_share = Path(get_package_share_directory("minicar_raceline"))

    default_nav2_params = str(nav2_share / "config/nav2_mppi_params.yaml")
    default_zone_params = str(zone_share / "config/speed_zone_params.yaml")
    default_vehicle_params = str(bringup_share / "config/vehicle_params.yaml")
    raceline_launch = str(raceline_share / "launch/raceline.launch.py")

    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        # ファイル名だけ(minicar_nav2/config/ の下)かフルパス。
        DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
        # ファイル名だけ(minicar_nav2_zone/config/ の下)かフルパス。
        DeclareLaunchArgument(
            "speed_zone_params_file", default_value=default_zone_params),
        DeclareLaunchArgument(
            "vehicle_params_file", default_value=default_vehicle_params),
        DeclareLaunchArgument("raceline_file", default_value="/maps/raceline.csv"),
        # 誤通電を防ぐため既定はPWMを出さない。
        DeclareLaunchArgument("backend", default_value="dryrun"),
        # 実走では負荷を避けて OFF。bag 再生で候補軌道を見るときだけ true にする。
        DeclareLaunchArgument("visualize", default_value="false"),
        # 空なら yaml の TrajectoryVisualizer.trajectory_step を使う。
        DeclareLaunchArgument("trajectory_step", default_value=""),
        # 停止検知と raceline を別の launch で起動するときは false にする。
        DeclareLaunchArgument("use_safety", default_value="true"),
        DeclareLaunchArgument("use_raceline", default_value="true"),

        OpaqueFunction(function=_mppi_and_zone),
        Node(
            package="minicar_safety",
            executable="safety_node",
            name="safety_node",
            output="screen",
            parameters=[
                str(safety_share / "config/safety_params.yaml"),
                {"vehicle_params_file": LaunchConfiguration("vehicle_params_file"),
                 "imu_topic": "/imu/data",
                 "use_sim_time": sim_time},
            ],
            condition=IfCondition(LaunchConfiguration("use_safety")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(raceline_launch),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "vehicle_params_file": LaunchConfiguration("vehicle_params_file"),
                "raceline_file": LaunchConfiguration("raceline_file"),
                "backend": LaunchConfiguration("backend"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_raceline")),
        ),
    ])
