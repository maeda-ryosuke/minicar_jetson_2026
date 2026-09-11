"""TG30 と取付位置の静的TFを起動する。ドライバ終了時は全体を終了。"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    with open(LaunchConfiguration('tf_params_file').perform(context)) as stream:
        tf = yaml.safe_load(stream)
    driver = Node(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node', namespace='/', output='screen',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': False}],
    )
    transform = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_pub_laser', output='screen',
        arguments=[item for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
                   for item in ('--' + key, str(tf[key]))] +
                  ['--frame-id', tf['parent_frame'], '--child-frame-id', tf['child_frame']],
        parameters=[{'use_sim_time': False}],
    )
    # SDKの初期化失敗でもドライバが0で終了するため、終了コードに依存しない。
    return [RegisterEventHandler(OnProcessExit(
        target_action=driver,
        on_exit=[EmitEvent(event=Shutdown(reason='TG30 driver exited'))],
    )), driver, transform]


def generate_launch_description():
    config = Path(get_package_share_directory('minicar_scan')) / 'config'
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(config / 'TG30.yaml')),
        DeclareLaunchArgument('tf_params_file', default_value=str(config / 'lidar_tf.yaml')),
        OpaqueFunction(function=_nodes),
    ])
