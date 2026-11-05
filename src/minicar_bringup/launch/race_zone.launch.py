"""race.launch.py の MPPI を、ヘアピンだけ速度帯を下げる zone 版に差し替えて一括起動する(実機)。

race.launch.py との違いは nav2_mppi.launch.py の代わりに minicar_nav2_zone の
zone_mppi.launch.py を起動することだけ。停止検知(stall_stop)・raceline・モータ・
自己位置推定は race.launch.py と同じ。

    /scan --> [scan_filter] --> [map_server + amcl] : map -> odom
    raceline.csv --> [raceline_manager] --FollowPath--> [controller_server(MPPI)]
        --/cmd_vel_raw--> [stall_stop_node] --/cmd_vel--> [motor_driver] --PWM--> 車両
    [speed_zone] --set_parameters(vx_max, vx_min)--> [controller_server]

MPPI の vx_max / vx_min は speed_zone_params_file の fast_* / slow_* で決まり、
nav2_params_file の値は使われない。それ以外の MPPI の設定は nav2_params_file。

前提(この launch の外で起動済み):
  * ホスト: micro_ros_agent、EKF(/odometry/filtered と odom -> base_link)
  * コンテナ: sensors.launch.py(/scan と静的 TF。compose が自動起動)

    ros2 launch minicar_bringup race_zone.launch.py \\
      raceline_file:=/maps/raceline_center_gate.csv \\
      nav2_params_file:=20261105_pattern2.yaml \\
      speed_zone_params_file:=speed_zone_params.yaml

nav2_params_file はファイル名だけなら minicar_nav2/config/ の下を、
speed_zone_params_file は minicar_nav2_zone/config/ の下を探す。

**backend の既定は fabo_pca9685(PWM を出す)。** raceline_manager はすぐ FollowPath を
送るので、車輪を浮かせるかプロポで止められる状態で起動する。
race.launch.py / raceline_mppi.launch.py / zone_mppi.launch.py / 個別起動の各ノードと
同時に起動しないこと。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _share(package):
    return Path(get_package_share_directory(package))


def _include(package, launch_file, fixed=None, **arguments):
    """arguments は {include 先の引数名: この launch の引数名}。fixed は固定値。"""
    path = _share(package) / "launch" / launch_file
    launch_arguments = {k: LaunchConfiguration(v) for k, v in arguments.items()}
    launch_arguments.update(fixed or {})
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    bringup_config = _share("minicar_bringup") / "config"
    nav2_config = _share("minicar_nav2") / "config"
    zone_config = _share("minicar_nav2_zone") / "config"
    raceline_config = _share("minicar_raceline") / "config"
    safety_config = _share("minicar_safety") / "config"
    motor_config = _share("minicar_motor") / "config"

    args = [
        ("use_sim_time", "false"),
        ("vehicle_params_file", str(bringup_config / "vehicle_params.yaml")),
        # 自己位置推定(AMCL)
        ("map_yaml", "/maps/track_v1.yaml"),
        ("scan_params_file", str(bringup_config / "scan_filter_localization.yaml")),
        ("amcl_params_file", str(bringup_config / "amcl_localization.yaml")),
        ("initial_x", "0.0"),
        ("initial_y", "0.0"),
        ("initial_yaw", "0.0"),
        # MPPI。ファイル名だけ(minicar_nav2/config/ の下)かフルパス。
        ("nav2_params_file", str(nav2_config / "nav2_mppi_params.yaml")),
        # 速度帯。ファイル名だけ(minicar_nav2_zone/config/ の下)かフルパス。
        ("speed_zone_params_file", str(zone_config / "speed_zone_params.yaml")),
        ("autostart", "true"),
        # 実走では負荷を避けて OFF。候補軌道を見るときだけ true。
        ("visualize", "false"),
        # 空なら nav2_params_file の TrajectoryVisualizer.trajectory_step。
        ("trajectory_step", ""),
        # 停止検知フィルタ
        ("stall_stop_params_file", str(safety_config / "stall_stop_params.yaml")),
        # レースライン
        ("raceline_file", "/maps/raceline.csv"),
        ("raceline_params_file", str(raceline_config / "raceline_params.yaml")),
        # モータ
        ("backend", "fabo_pca9685"),
        ("motor_params_file", str(motor_config / "motor_driver_params.yaml")),
        ("pwm_params_file", str(motor_config / "pwm_params.json")),
        # 空なら motor_driver_params.yaml の値。
        ("speed_control", ""),
        ("throttle_map_u", ""),
    ]

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in args],

        _include("minicar_bringup", "slam_localization_amcl.launch.py",
                 use_sim_time="use_sim_time",
                 map_yaml="map_yaml",
                 scan_params_file="scan_params_file",
                 amcl_params_file="amcl_params_file",
                 initial_x="initial_x",
                 initial_y="initial_y",
                 initial_yaw="initial_yaw"),
        # 停止検知と raceline はこの launch の stall_stop / raceline を使うので、
        # zone_mppi 側の safety_node と raceline_manager は起動しない。
        _include("minicar_nav2_zone", "zone_mppi.launch.py",
                 fixed={"use_safety": "false", "use_raceline": "false"},
                 use_sim_time="use_sim_time",
                 autostart="autostart",
                 nav2_params_file="nav2_params_file",
                 speed_zone_params_file="speed_zone_params_file",
                 vehicle_params_file="vehicle_params_file",
                 visualize="visualize",
                 trajectory_step="trajectory_step"),
        _include("minicar_safety", "stall_stop.launch.py",
                 use_sim_time="use_sim_time",
                 params_file="stall_stop_params_file"),
        _include("minicar_raceline", "raceline.launch.py",
                 use_sim_time="use_sim_time",
                 raceline_file="raceline_file",
                 raceline_params_file="raceline_params_file"),
        _include("minicar_motor", "motor_driver.launch.py",
                 use_sim_time="use_sim_time",
                 vehicle_params_file="vehicle_params_file",
                 backend="backend",
                 motor_params_file="motor_params_file",
                 pwm_params_file="pwm_params_file",
                 speed_control="speed_control",
                 throttle_map_u="throttle_map_u"),
    ])
