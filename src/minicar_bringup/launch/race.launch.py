"""実走に必要な走行系を一括起動する(実機)。コマンドリスト.md の個別起動をまとめたもの。

    /scan --> [scan_filter] --> [map_server + amcl] : map -> odom
    raceline.csv --> [raceline_manager] --FollowPath--> [controller_server(MPPI)]
        --/cmd_vel_raw--> [stall_stop_node] --/cmd_vel--> [motor_driver] --PWM--> 車両

前提(この launch の外で起動済み):
  * ホスト: micro_ros_agent、EKF(/odometry/filtered と odom -> base_link)
  * コンテナ: sensors.launch.py(/scan と静的 TF。compose が自動起動)

各ノードの設定は include 先の launch / yaml がそのまま使われる。パラメータファイルは
すべて launch 引数で差し替えられ、compose が src の config をマウントしているので
yaml を直して再起動すれば docker build なしで反映される。

    ros2 launch minicar_bringup race.launch.py \\
      initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0

    # レースライン・MPPI パラメータを変える(nav2_params_file はファイル名だけなら
    # minicar_nav2/config/ の下を探す)
    ros2 launch minicar_bringup race.launch.py \\
      raceline_file:=/maps/raceline_v2.csv \\
      nav2_params_file:=nav2_mppi_params_backup_20261102.yaml

    # PWM を出さずに確かめる / 速度 PI を切る
    ros2 launch minicar_bringup race.launch.py backend:=dryrun speed_control:=off

**backend の既定は fabo_pca9685(PWM を出す)。** raceline_manager はすぐ FollowPath を
送るので、車輪を浮かせるかプロポで止められる状態で起動する。
raceline_mppi.launch.py / control.launch.py / 個別起動の各ノードと同時に起動しないこと。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _share(package):
    return Path(get_package_share_directory(package))


def _include(package, launch_file, **arguments):
    """arguments は {include 先の引数名: この launch の引数名}。"""
    path = _share(package) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments={k: LaunchConfiguration(v) for k, v in arguments.items()}.items(),
    )


def generate_launch_description():
    bringup_config = _share("minicar_bringup") / "config"
    nav2_config = _share("minicar_nav2") / "config"
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
        ("autostart", "true"),
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
        _include("minicar_nav2", "nav2_mppi.launch.py",
                 use_sim_time="use_sim_time",
                 autostart="autostart",
                 nav2_params_file="nav2_params_file",
                 vehicle_params_file="vehicle_params_file"),
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
