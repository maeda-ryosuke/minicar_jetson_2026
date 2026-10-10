"""実機の raceline_manager を起動する（実機）。

    raceline.csv -> [raceline_manager] --FollowPath--> controller_server(MPPI)

**この launch は raceline_manager 以外を起動しない。** motor_driver は
minicar_motor の motor_driver.launch.py で単独に起動する。

    ros2 launch minicar_raceline raceline.launch.py \\
      raceline_file:=/maps/raceline.csv

use_sim_time の既定は false（実機なので実時間）。minicar_jetson_2026 の
mppi.launch.py と同じ作法で、文字列の launch 引数を ParameterValue で
bool に変換して渡す。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    raceline_share = Path(get_package_share_directory("minicar_raceline"))

    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        # 参照経路。既定は compose がバインドする /maps。
        DeclareLaunchArgument("raceline_file", default_value="/maps/raceline.csv"),

        Node(
            package="minicar_raceline",
            executable="raceline_manager_node",
            name="raceline_manager",
            output="screen",
            parameters=[
                str(raceline_share / "config" / "raceline_params.yaml"),
                {"raceline_file": LaunchConfiguration("raceline_file"),
                 "use_sim_time": sim_time},
            ],
        ),
    ])
