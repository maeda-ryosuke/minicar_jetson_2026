"""Launch the D455 publisher and its measured base_link to camera_link TF.

Topics keep the realsense-ros names (/camera/infra1/image_rect_raw, /camera/imu, ...);
the Isaac ROS cuVSLAM container remaps them to its own inputs.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from minicar_realsense.mount_config import load_mount_config


def _launch_actions(context):
    mount = load_mount_config(LaunchConfiguration('mount_params_file').perform(context))

    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        parameters=[LaunchConfiguration('realsense_params_file'), {'use_sim_time': False}],
    )

    mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_d455_mount',
        output='screen',
        arguments=[
            '--x', str(mount['x']), '--y', str(mount['y']), '--z', str(mount['z']),
            '--roll', str(mount['roll']), '--pitch', str(mount['pitch']),
            '--yaw', str(mount['yaw']), '--frame-id', mount['parent_frame'],
            '--child-frame-id', mount['child_frame'],
        ],
        parameters=[{'use_sim_time': False}],
    )

    return [camera, mount_tf]


def generate_launch_description():
    config = Path(get_package_share_directory('minicar_realsense')) / 'config'
    return LaunchDescription([
        DeclareLaunchArgument(
            'realsense_params_file', default_value=str(config / 'd455.yaml')),
        DeclareLaunchArgument(
            'mount_params_file', default_value=str(config / 'camera_mount.yaml')),
        OpaqueFunction(function=_launch_actions),
    ])
