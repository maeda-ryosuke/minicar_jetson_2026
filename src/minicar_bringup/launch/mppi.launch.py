"""実機の FTG + MPPI + safety を起動する。TF はホスト側が担当する。"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    vehicle = str(Path(get_package_share_directory("minicar_bringup")) / "config/vehicle_params.yaml")
    actions = [
        DeclareLaunchArgument("vehicle_params_file", default_value=vehicle),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    for kind in ("ftg", "mppi", "safety"):
        package = "minicar_" + kind
        actions.append(Node(
            package=package, executable=kind + "_node", output="screen",
            parameters=[
                str(Path(get_package_share_directory(package)) / "config" / (kind + "_params.yaml")),
                {"vehicle_params_file": LaunchConfiguration("vehicle_params_file"),
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
            ],
        ))
    return LaunchDescription(actions)
