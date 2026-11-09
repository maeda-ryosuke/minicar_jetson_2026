"""my_mppi_controller(自転車モデル+スピード固定)の controller_server だけを起動する。

nav2_mppi.launch.py(minicar_nav2)と同じ書き方で、プラグインとパラメータだけが違う。
wheelbase は vehicle_params.yaml から注入する(motor_driver の ω→δ 変換と同じ値にするため)。
/target_speed(speed_zone_publisher)が来ないうちは v=0 のまま走らない。

    ros2 launch my_mppi_controller my_mppi.launch.py
    ros2 launch my_mppi_controller my_mppi.launch.py mppi_params_file:=/path/to/params.yaml
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _controller_server(context):
    params_file = LaunchConfiguration("mppi_params_file").perform(context)
    vehicle_params_file = LaunchConfiguration("vehicle_params_file").perform(context)
    with open(vehicle_params_file) as f:
        wheelbase = float(yaml.safe_load(f)["vehicle"]["wheelbase"])
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    autostart = ParameterValue(LaunchConfiguration("autostart"), value_type=bool)
    return [
        LogInfo(msg=f"MPPI params = {params_file}"),
        LogInfo(msg=f"MPPI wheelbase = {wheelbase:.3f} m ({vehicle_params_file})"),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": sim_time},
                # 後ろほど優先される。yaml の値より vehicle_params を正とする。
                {"FollowPath": {"wheelbase": wheelbase}},
            ],
            remappings=[("cmd_vel", "/cmd_vel_raw")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_my_mppi",
            output="screen",
            parameters=[{
                "use_sim_time": sim_time,
                "autostart": autostart,
                "bond_timeout": 4.0,
                "node_names": ["controller_server"],
            }],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("my_mppi_controller"))
    bringup_share = Path(get_package_share_directory("minicar_bringup"))

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "mppi_params_file", default_value=str(share / "config/my_mppi_params.yaml")),
        DeclareLaunchArgument(
            "vehicle_params_file",
            default_value=str(bringup_share / "config/vehicle_params.yaml")),
        OpaqueFunction(function=_controller_server),
    ])
