"""speed_zone_publisher ノードだけを起動する。

    ros2 launch speed_zone_publisher speed_zone_publisher.launch.py
    ros2 launch speed_zone_publisher speed_zone_publisher.launch.py zones_yaml:=zones_center_gate_low2_shortcut.yaml

zones_yaml はファイル名だけなら speed_zone_publisher/config/ の下、"/" を含めばそのパスとして
読む。存在しなければ起動時に落とす(nav2_mppi.launch.py と同じ)。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_zones_yaml(context):
    """zones_yaml を実在するパスにする。ファイル名だけなら config/ の下を探す。"""
    value = LaunchConfiguration("zones_yaml").perform(context).strip()
    if "/" not in value:
        value = str(Path(get_package_share_directory("speed_zone_publisher")) / "config" / value)
    if not Path(value).is_file():
        raise FileNotFoundError(f"zones_yaml が見つからない: {value}")
    return value


def _node(context):
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return [Node(
        package="speed_zone_publisher",
        executable="speed_zone_publisher",
        name="speed_zone_publisher",
        output="screen",
        parameters=[{
            "zones_yaml": _resolve_zones_yaml(context),
            "use_sim_time": sim_time,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("zones_yaml", default_value="zones.yaml"),
        OpaqueFunction(function=_node),
    ])
