"""Launch a D455 publisher, its measured mount TF, and Isaac ROS cuVSLAM."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from minicar_realsense.mount_config import load_mount_config


def _shutdown_when_process_exits(process, reason):
    return RegisterEventHandler(OnProcessExit(
        target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason=reason))],
    ))


def _launch_actions(context):
    mount = load_mount_config(LaunchConfiguration('mount_params_file').perform(context))

    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        parameters=[LaunchConfiguration('realsense_params_file'), {'use_sim_time': False}],
        remappings=[
            ('infra1/image_rect_raw', '/visual_slam/image_0'),
            ('infra1/camera_info', '/visual_slam/camera_info_0'),
            ('infra2/image_rect_raw', '/visual_slam/image_1'),
            ('infra2/camera_info', '/visual_slam/camera_info_1'),
            ('imu', '/visual_slam/imu'),
        ],
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

    visual_slam = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[LaunchConfiguration('cuvslam_params_file'), {'use_sim_time': False}],
    )
    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='visual_slam_launch_container',
        namespace='',
        composable_node_descriptions=[visual_slam],
        output='screen',
    )

    return [
        _shutdown_when_process_exits(camera, 'RealSense D455 driver exited'),
        _shutdown_when_process_exits(container, 'Isaac ROS Visual SLAM container exited'),
        camera,
        mount_tf,
        container,
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('minicar_realsense'))
    config = share / 'config'
    return LaunchDescription([
        DeclareLaunchArgument(
            'realsense_params_file', default_value=str(config / 'd455.yaml')),
        DeclareLaunchArgument(
            'cuvslam_params_file', default_value=str(config / 'cuvslam.yaml')),
        DeclareLaunchArgument(
            'mount_params_file', default_value=str(config / 'camera_mount.yaml')),
        OpaqueFunction(function=_launch_actions),
    ])
