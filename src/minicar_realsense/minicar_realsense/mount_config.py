"""Validation for the measured base_link to camera_link transform."""

import math
from pathlib import Path

import yaml


_POSE_KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def load_mount_config(path):
    """Load and validate a deliberately configured 6-DoF camera mount."""
    config_path = Path(path)
    with config_path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)

    if not isinstance(document, dict) or not isinstance(document.get('camera_mount'), dict):
        raise RuntimeError(f'{config_path}: camera_mount mapping is required')
    mount = document['camera_mount']
    if mount.get('configured') is not True:
        raise RuntimeError(
            f'{config_path}: set camera_mount.configured=true after entering the measured transform'
        )

    parent = mount.get('parent_frame')
    child = mount.get('child_frame')
    if not isinstance(parent, str) or not parent or not isinstance(child, str) or not child:
        raise RuntimeError(f'{config_path}: parent_frame and child_frame are required')
    if parent == child:
        raise RuntimeError(f'{config_path}: parent_frame and child_frame must differ')

    pose = {}
    for key in _POSE_KEYS:
        value = mount.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError(f'{config_path}: camera_mount.{key} must be numeric')
        value = float(value)
        if not math.isfinite(value):
            raise RuntimeError(f'{config_path}: camera_mount.{key} must be finite')
        pose[key] = value

    if all(abs(pose[key]) < 1e-12 for key in _POSE_KEYS):
        raise RuntimeError(f'{config_path}: an identity camera mount is not accepted')

    return {'parent_frame': parent, 'child_frame': child, **pose}
