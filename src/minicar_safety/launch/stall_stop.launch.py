"""停止検知フィルタ (stall_stop_node) を単体で起動する (実機)。

    controller_server --/cmd_vel_raw--> [stall_stop_node] --/cmd_vel--> [motor_driver]

    ros2 launch minicar_safety stall_stop.launch.py

safety_node (control.launch.py / raceline_mppi.launch.py が起動する) と
同時に起動しないこと。どちらも /cmd_vel に publish する。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    safety_share = Path(get_package_share_directory("minicar_safety"))
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=str(safety_share / "config/stall_stop_params.yaml")),
        Node(
            package="minicar_safety",
            executable="stall_stop_node",
            name="stall_stop_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file"), {"use_sim_time": sim_time}],
        ),
    ])
