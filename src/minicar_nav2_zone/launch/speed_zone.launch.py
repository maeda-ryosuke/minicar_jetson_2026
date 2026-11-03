"""speed_zone ノードだけを起動する。

controller_server(minicar_nav2 の nav2_mppi.launch.py)と raceline_manager を
別ターミナルで個別に起動しているときに足す。TF と controller_server が揃った
最初の判定で必ず速度帯を送る。それまでは nav2_mppi_params.yaml の
vx_max / vx_min のままなので、こちらを先に起動しておくこと。

    ros2 launch minicar_nav2_zone speed_zone.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    zone_share = Path(get_package_share_directory("minicar_nav2_zone"))
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "speed_zone_params_file",
            default_value=str(zone_share / "config/speed_zone_params.yaml")),
        Node(
            package="minicar_nav2_zone",
            executable="speed_zone_node",
            name="speed_zone",
            output="screen",
            parameters=[
                LaunchConfiguration("speed_zone_params_file"),
                {"use_sim_time": sim_time},
            ],
        ),
    ])
