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
    assert params['publish_tf'] is True


def test_driver_keeps_realsense_topic_names():
    # cuVSLAM (minicar_isaac_vslam container) subscribes to /camera/infra1/image_rect_raw etc.,
    # so the driver must run as /camera/camera without remapping its outputs.
    launch = (PACKAGE / 'launch' / 'd455.launch.py').read_text(encoding='utf-8')
    assert "namespace='camera'" in launch
    assert "name='camera'" in launch
    assert 'remappings' not in launch
    assert 'visual_slam' not in launch
