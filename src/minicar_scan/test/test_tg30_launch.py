"""実デバイスを開かず、統合launchの設定解決と終了連携を検証する。"""
import os
from pathlib import Path
import signal
import subprocess

from ament_index_python.packages import get_package_share_directory
import yaml


def test_driver_failure_stops_static_tf(tmp_path):
    config = Path(get_package_share_directory('minicar_scan')) / 'config'
    params = yaml.safe_load((config / 'TG30.yaml').read_text())
    # 必ず存在しないポートを指定し、接続中のセンサを触らない。
    params['ydlidar_ros2_driver_node']['ros__parameters']['port'] = str(tmp_path / 'absent')
    invalid = tmp_path / 'invalid.yaml'
    invalid.write_text(yaml.safe_dump(params))
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='231')
    process = subprocess.Popen(
        ['ros2', 'launch', 'minicar_scan', 'tg30.launch.py', f'params_file:={invalid}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True, env=env,
    )
    try:
        output, _ = process.communicate(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate()
        raise AssertionError('ドライバ終了後にlaunchが残存した:\n' + output)
    assert process.returncode == 0, output
    assert 'ydlidar_ros2_driver_node-1]: process started' in output, output
    assert 'static_transform_publisher-2]: process started' in output, output
    assert 'stopping' in output, output
    assert 'static_transform_publisher-2]: process has finished cleanly' in output, output
    assert 'Traceback' not in output, output
