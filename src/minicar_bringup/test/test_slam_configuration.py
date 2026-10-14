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
            "use_sim_time", "interactive_mode", "slam_params_file",
            "scan_params_file",
        ),
        "slam_localization.launch.py": (
            "use_sim_time", "posegraph_file", "slam_params_file",
            "scan_params_file",
        ),
        "slam_localization_amcl.launch.py": (
            "use_sim_time", "map_yaml", "amcl_params_file", "scan_params_file",
            "initial_x", "initial_y", "initial_yaw",
        ),
        "sensors.launch.py": ("enable_camera",),
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


def test_localization_scan_filter_matches_laser_range():
    share = Path(get_package_share_directory("minicar_bringup")) / "config"
    scan = yaml.safe_load((share / "scan_filter_localization.yaml").read_text())
    scan = scan["scan_filter_node"]["ros__parameters"]
    localization = _params("slam_toolbox_localization.yaml")
    assert scan["output_topic"] == localization["scan_topic"]
    assert scan["range_max"] == localization["max_laser_range"]
    assert len(localization["map_start_pose"]) == 3


def test_amcl_keeps_the_localization_contract():
    share = Path(get_package_share_directory("minicar_bringup")) / "config"
    amcl = yaml.safe_load((share / "amcl_localization.yaml").read_text())["amcl"]["ros__parameters"]
    localization = _params("slam_toolbox_localization.yaml")
    assert amcl["global_frame_id"] == localization["map_frame"]
    assert amcl["odom_frame_id"] == localization["odom_frame"]
    assert amcl["base_frame_id"] == localization["base_frame"]
    assert amcl["scan_topic"] == localization["scan_topic"]
    assert amcl["laser_max_range"] == localization["max_laser_range"]
    assert amcl["tf_broadcast"] is True
