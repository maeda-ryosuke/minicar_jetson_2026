from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]


def _parameters(filename):
    document = yaml.safe_load((PACKAGE / 'config' / filename).read_text(encoding='utf-8'))
    return document['/**']['ros__parameters']


def test_d455_stream_configuration_matches_cuvslam_inputs():
    params = _parameters('d455.yaml')
    assert params['enable_infra1'] is True
    assert params['enable_infra2'] is True
    assert params['enable_color'] is False
    assert params['enable_depth'] is False
    assert params['depth_module.emitter_enabled'] == 0
    assert params['depth_module.profile'] == '640x360x90'
    assert params['gyro_fps'] == 200
    assert params['accel_fps'] == 200
    assert params['unite_imu_method'] == 2


def test_cuvslam_owns_requested_frames_and_uses_imu():
    params = _parameters('cuvslam.yaml')
    assert params['num_cameras'] == 2
    assert params['rectified_images'] is True
    assert params['enable_imu_fusion'] is True
    assert params['enable_localization_n_mapping'] is True
    assert params['map_frame'] == 'map'
    assert params['odom_frame'] == 'odom'
    assert params['base_frame'] == 'base_link'
    assert params['camera_optical_frames'] == [
        'camera_infra1_optical_frame', 'camera_infra2_optical_frame']
    assert params['imu_frame'] == 'camera_gyro_optical_frame'
    assert params['publish_odom_to_base_tf'] is True
    assert params['publish_map_to_odom_tf'] is True
    assert params['image_qos'] == 'SENSOR_DATA'
    assert params['imu_qos'] == 'SENSOR_DATA'


def test_launch_exposes_exact_five_input_topics_and_shutdown_handlers():
    launch = (PACKAGE / 'launch' / 'd455_cuvslam.launch.py').read_text(encoding='utf-8')
    for topic in (
        '/visual_slam/image_0',
        '/visual_slam/camera_info_0',
        '/visual_slam/image_1',
        '/visual_slam/camera_info_1',
        '/visual_slam/imu',
    ):
        assert launch.count(topic) == 1
    assert launch.count('_shutdown_when_process_exits(') == 3  # definition + two uses
    assert "plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode'" in launch
