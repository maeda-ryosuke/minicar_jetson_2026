"""Nav2 MPPIによるレースライン追従の走行系を一括起動する。

前提として、外部で次が起動済みであること:
  * robot_localization: /odometry/filtered と odom -> base_link
  * slam_toolbox: map -> odom
  * LiDAR / IMU: /scan と /imu/data

指令は必ず次の一本道にする。旧 minicar_mppi と同時起動しないこと。

  raceline --FollowPath--> controller_server --/cmd_vel_raw-->
      safety --/cmd_vel--> motor_driver
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav2_share = Path(get_package_share_directory("minicar_nav2"))
    bringup_share = Path(get_package_share_directory("minicar_bringup"))
    safety_share = Path(get_package_share_directory("minicar_safety"))
    raceline_share = Path(get_package_share_directory("minicar_raceline"))

    default_nav2_params = str(nav2_share / "config/nav2_mppi_params.yaml")
    default_vehicle_params = str(bringup_share / "config/vehicle_params.yaml")
    raceline_launch = str(raceline_share / "launch/raceline.launch.py")

    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    autostart = ParameterValue(LaunchConfiguration("autostart"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
        DeclareLaunchArgument(
            "vehicle_params_file", default_value=default_vehicle_params),
        DeclareLaunchArgument("raceline_file", default_value="/maps/raceline.csv"),
        # 誤通電を防ぐため既定はPWMを出さない。
        DeclareLaunchArgument("backend", default_value="dryrun"),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[
                LaunchConfiguration("nav2_params_file"),
                {"use_sim_time": sim_time},
            ],
            # safety_nodeだけが最終 /cmd_vel をpublishする。
            remappings=[("cmd_vel", "/cmd_vel_raw")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_nav2_mppi",
            output="screen",
            parameters=[
                LaunchConfiguration("nav2_params_file"),
                {"use_sim_time": sim_time,
                 "autostart": autostart,
                 "node_names": ["controller_server"]},
            ],
        ),
        Node(
            package="minicar_safety",
            executable="safety_node",
            name="safety_node",
            output="screen",
            parameters=[
                str(safety_share / "config/safety_params.yaml"),
                {"vehicle_params_file": LaunchConfiguration("vehicle_params_file"),
                 "imu_topic": "/imu/data",
                 "use_sim_time": sim_time},
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(raceline_launch),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "vehicle_params_file": LaunchConfiguration("vehicle_params_file"),
                "raceline_file": LaunchConfiguration("raceline_file"),
                "backend": LaunchConfiguration("backend"),
            }.items(),
        ),
    ])
