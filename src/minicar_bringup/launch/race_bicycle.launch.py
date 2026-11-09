"""race.launch.py の MPPI を、自転車モデル+スピード固定の my_mppi_controller に差し替えて
一括起動する(実機)。race.launch.py は変更しない。

    /scan --> [scan_filter] --> [map_server + amcl] : map -> odom
    TF map -> base_link + zones.yaml --> [speed_zone_publisher] --/target_speed-->
    raceline.csv --> [raceline_manager] --FollowPath--> [controller_server(my_mppi_controller)]
        --/cmd_vel_raw--> [stall_stop_node] --/cmd_vel--> [motor_driver] --PWM--> 車両

race.launch.py との違いは、nav2_mppi.launch.py の代わりに my_mppi.launch.py と
speed_zone_publisher.launch.py を起動することだけ。safety_node は通さない
(safety_node は v だけを加速制限するので、ドライバで復元する舵角がずれる)。

    ros2 launch minicar_bringup race_bicycle.launch.py \\
      initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0

    # MPPI のパラメータ・エリア定義を変える(フルパス)
    ros2 launch minicar_bringup race_bicycle.launch.py \\
      mppi_params_file:=/path/to/my_mppi_params.yaml zones_yaml:=/path/to/zones.yaml

race.launch.py / race_zone.launch.py / minicar_nav2_zone の launch と同時に起動しないこと。
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
    mppi_config = _share("my_mppi_controller") / "config"
    zone_config = _share("speed_zone_publisher") / "config"
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
        # MPPI(フルパス)とエリア定義
        ("mppi_params_file", str(mppi_config / "my_mppi_params.yaml")),
        ("zones_yaml", str(zone_config / "zones.yaml")),
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
        _include("my_mppi_controller", "my_mppi.launch.py",
                 use_sim_time="use_sim_time",
                 autostart="autostart",
                 mppi_params_file="mppi_params_file",
                 vehicle_params_file="vehicle_params_file"),
        _include("speed_zone_publisher", "speed_zone_publisher.launch.py",
                 use_sim_time="use_sim_time",
                 zones_yaml="zones_yaml"),
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
