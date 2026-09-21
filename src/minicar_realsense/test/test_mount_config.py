from pathlib import Path

import pytest
import yaml

from minicar_realsense.mount_config import load_mount_config


def _write_mount(tmp_path, **overrides):
    mount = {
        'configured': True,
        'parent_frame': 'base_link',
        'child_frame': 'camera_link',
        'x': 0.2,
        'y': 0.0,
        'z': 0.1,
        'roll': 0.0,
        'pitch': 0.0,
        'yaw': 0.0,
    }
    mount.update(overrides)
    path = tmp_path / 'mount.yaml'
    path.write_text(yaml.safe_dump({'camera_mount': mount}), encoding='utf-8')
    return path


def test_accepts_measured_mount(tmp_path):
    mount = load_mount_config(_write_mount(tmp_path))
    assert mount['parent_frame'] == 'base_link'
    assert mount['child_frame'] == 'camera_link'
    assert mount['x'] == pytest.approx(0.2)


@pytest.mark.parametrize(
    'overrides, message',
    [
        ({'configured': False}, 'configured=true'),
        ({'x': 0.0, 'z': 0.0}, 'identity'),
        ({'x': '0.2'}, 'must be numeric'),
        ({'child_frame': 'base_link'}, 'must differ'),
    ],
)
def test_rejects_unusable_mount(tmp_path, overrides, message):
    with pytest.raises(RuntimeError, match=message):
        load_mount_config(_write_mount(tmp_path, **overrides))


def test_repository_default_is_intentionally_unconfigured():
    path = Path(__file__).parents[1] / 'config' / 'camera_mount.yaml'
    with pytest.raises(RuntimeError, match='configured=true'):
        load_mount_config(path)
