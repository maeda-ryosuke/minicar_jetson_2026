"""Nav2 MPPI の controller_server だけを起動する(検証用)。

safety_node、raceline_manager、motor_driver は起動しない。それらを別ターミナルで
個別に起動して検証するときに使う。通常走行は raceline_mppi.launch.py。

前提として、外部で次が起動済みであること:
  * robot_localization: /odometry/filtered と odom -> base_link
  * slam_toolbox: map -> odom
  * LiDAR: /scan

    [raceline_manager] --FollowPath--> controller_server --/cmd_vel_raw--> [...]
    ([] はこの launch の外で起動する)

**safety_node を通さないので、/cmd_vel_raw はそのまま外へ出る。**
スタック検知と後退脱出は効かない。

    ros2 launch minicar_nav2 nav2_mppi.launch.py

raceline_mppi.launch.py と同時に起動しないこと(controller_server が
二重になる)。

MPPI のパラメータファイルは nav2_params_file で指定する。ファイル名だけなら
minicar_nav2/config/ の下、"/" を含めばそのパスとして読む。存在しなければ起動時に落とす。
compose は src/minicar_nav2/config をコンテナへマウントしているので、yaml を
追加・編集しても docker build は要らない(launch の再起動で反映)。

    ros2 launch minicar_nav2 nav2_mppi.launch.py \
      nav2_params_file:=nav2_mppi_params_backup_20261102.yaml
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from minicar_nav2.vehicle_limits import min_turning_radius


def _resolve_nav2_params(context):
    """nav2_params_file を実在するパスにする。ファイル名だけなら config/ の下を探す。"""
    value = LaunchConfiguration("nav2_params_file").perform(context).strip()
    if "/" not in value:
        value = str(Path(get_package_share_directory("minicar_nav2")) / "config" / value)
    if not Path(value).is_file():
        raise FileNotFoundError(f"nav2_params_file が見つからない: {value}")
    return value


def _controller_server(context):
    """vehicle_params から旋回制約を導出して controller_server を起動する。"""
    nav2_params_file = _resolve_nav2_params(context)
    autostart = ParameterValue(LaunchConfiguration("autostart"), value_type=bool)
    vehicle_params_file = LaunchConfiguration("vehicle_params_file").perform(context)
    r_min = min_turning_radius(vehicle_params_file)
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return [
        LogInfo(msg=f"MPPI params = {nav2_params_file}"),
        LogInfo(msg=f"MPPI min_turning_r = {r_min:.3f} m "
                    f"(wheelbase / tan(delta_max), {vehicle_params_file})"),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[
                nav2_params_file,
                {"use_sim_time": sim_time},
                # 後ろほど優先される。yaml の値より vehicle_params を正とする。
                {"FollowPath": {"AckermannConstraints": {"min_turning_r": r_min}}},
            ],
            # 出力名は raceline_mppi.launch.py と揃える。
            remappings=[("cmd_vel", "/cmd_vel_raw")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_nav2_mppi",
            output="screen",
            parameters=[
                nav2_params_file,
                {"use_sim_time": sim_time,
                 "autostart": autostart,
                 "node_names": ["controller_server"]},
            ],
        ),
    ]


def generate_launch_description():
    nav2_share = Path(get_package_share_directory("minicar_nav2"))
    bringup_share = Path(get_package_share_directory("minicar_bringup"))

    default_nav2_params = str(nav2_share / "config/nav2_mppi_params.yaml")
    default_vehicle_params = str(bringup_share / "config/vehicle_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        # ファイル名だけ(config/ の下)かフルパス。
        DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
        DeclareLaunchArgument(
            "vehicle_params_file", default_value=default_vehicle_params),

        OpaqueFunction(function=_controller_server),
    ])
