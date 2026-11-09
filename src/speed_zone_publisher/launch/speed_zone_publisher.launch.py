"""speed_zone_publisher ノードだけを起動する。

    ros2 launch speed_zone_publisher speed_zone_publisher.launch.py
    ros2 launch speed_zone_publisher speed_zone_publisher.launch.py zones_yaml:=/path/to/zones.yaml
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("speed_zone_publisher"))
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("zones_yaml", default_value=str(share / "config/zones.yaml")),
        Node(
            package="speed_zone_publisher",
            executable="speed_zone_publisher",
            name="speed_zone_publisher",
            output="screen",
            parameters=[{
                "zones_yaml": LaunchConfiguration("zones_yaml"),
                "use_sim_time": sim_time,
            }],
        ),
    ])
