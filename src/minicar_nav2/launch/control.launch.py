"""制御系(Nav2 MPPI + safety + raceline_manager + motor_driver)を一括起動する。

個別の launch をそのまま include するので、各ノードの設定はそれぞれの
launch / yaml を直せばここにも反映される。

  raceline --FollowPath--> controller_server --/cmd_vel_raw-->
      safety --/cmd_vel--> motor_driver --PWM--> 車両

    # 観測のみ。PWM は出ない
    ros2 launch minicar_nav2 control.launch.py

    # 実機 PWM(車輪を浮かせるか、プロポで止められる状態で)
    ros2 launch minicar_nav2 control.launch.py backend:=fabo_pca9685

起動すると raceline_manager がすぐ FollowPath を送るので、認識系
(perception.launch.py)で自己位置が収束したのを確かめてから起動する。
raceline_mppi.launch.py・個別起動の motor_driver と同時に起動しないこと。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _include(package, launch_file, **arguments):
    path = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments={k: LaunchConfiguration(v) for k, v in arguments.items()}.items(),
    )


def generate_launch_description():
    nav2_share = Path(get_package_share_directory("minicar_nav2"))
    bringup_share = Path(get_package_share_directory("minicar_bringup"))
    safety_share = Path(get_package_share_directory("minicar_safety"))

    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    common = {"use_sim_time": "use_sim_time", "vehicle_params_file": "vehicle_params_file"}

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "vehicle_params_file",
            default_value=str(bringup_share / "config/vehicle_params.yaml")),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=str(nav2_share / "config/nav2_mppi_params.yaml")),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("raceline_file", default_value="/maps/raceline.csv"),
        # 一括起動は走り出すまでが早いので、誤通電を防ぐため既定は PWM を出さない。
        DeclareLaunchArgument("backend", default_value="dryrun"),
        # 空なら motor_driver_params.yaml の値（実測マップ）。上書きするときは
        # yaml の throttle_map_v_mps と同じ 8 点で渡す。
        DeclareLaunchArgument("throttle_map_u", default_value=""),

        _include("minicar_nav2", "nav2_mppi.launch.py",
                 nav2_params_file="nav2_params_file", autostart="autostart", **common),
        Node(
            package="minicar_safety",
            executable="safety_node",
            name="safety_node",
            output="screen",
            parameters=[
                str(safety_share / "config/safety_params.yaml"),
                {"vehicle_params_file": LaunchConfiguration("vehicle_params_file"),
                 "use_sim_time": sim_time},
            ],
        ),
        _include("minicar_raceline", "raceline.launch.py",
                 use_sim_time="use_sim_time", raceline_file="raceline_file"),
        _include("minicar_motor", "motor_driver.launch.py",
                 backend="backend", throttle_map_u="throttle_map_u", **common),
    ])
