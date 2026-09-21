"""SLAMのlaunch引数とmapping/localization間のTF契約を静的に検証する。"""
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
import yaml


def _params(filename):
    share = Path(get_package_share_directory("minicar_bringup"))
    data = yaml.safe_load((share / "config" / filename).read_text())
    return data["slam_toolbox"]["ros__parameters"]


def test_mapping_and_localization_keep_the_same_frame_contract():
    mapping = _params("slam_toolbox_mapping.yaml")
    localization = _params("slam_toolbox_localization.yaml")

    for params in (mapping, localization):
        assert params["map_frame"] == "map"
        assert params["odom_frame"] == "odom"
        assert params["base_frame"] == "base_link"
        assert params["scan_topic"] == "/scan_filtered"
        assert params["scan_queue_size"] == 1
        assert params["enable_interactive_mode"] is False

    assert mapping["mode"] == "mapping"
    assert localization["mode"] == "localization"
    assert localization["map_start_at_dock"] is False
    assert localization["map_file_name"]


def test_slam_launch_files_expose_required_arguments(tmp_path):
    expected = {
        "slam_mapping.launch.py": (
            "use_sim_time", "interactive_mode", "slam_params_file"
        ),
        "slam_localization.launch.py": (
            "use_sim_time", "posegraph_file", "slam_params_file"
        ),
        "sensors.launch.py": ("scan_params_file",),
    }
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / "ros-log"))
    for launch_file, arguments in expected.items():
        result = subprocess.run(
            ["ros2", "launch", "minicar_bringup", launch_file, "--show-args"],
            check=True,
            capture_output=True,
            text=True,
            timeout=20,
            env=env,
        )
        for argument in arguments:
            assert argument in result.stdout, result.stdout
