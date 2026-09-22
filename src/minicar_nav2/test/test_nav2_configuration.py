"""Nav2 MPPI設定と既存パッケージ間の契約を静的に検証する。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


def _share(package):
    return Path(get_package_share_directory(package))


def _yaml(package, relative):
    with (_share(package) / relative).open() as f:
        return yaml.safe_load(f)


def test_mppi_uses_filtered_odometry_and_ackermann_limits():
    nav2 = _yaml("minicar_nav2", "config/nav2_mppi_params.yaml")
    vehicle = _yaml("minicar_bringup", "config/vehicle_params.yaml")
    server = nav2["controller_server"]["ros__parameters"]
    mppi = server["FollowPath"]

    assert server["odom_topic"] == "/odometry/filtered"
    assert server["controller_frequency"] == 1.0 / mppi["model_dt"]
    assert mppi["motion_model"] == "Ackermann"
    assert mppi["AckermannConstraints"]["min_turning_r"] == \
        vehicle["limits"]["r_min"]
    assert 0.0 <= mppi["vx_min"] <= mppi["vx_max"] <= \
        vehicle["limits"]["v_max"]
    assert mppi["ax_max"] <= vehicle["limits"]["a_max"]
    assert mppi["ax_min"] >= -vehicle["limits"]["a_max"]


def test_initial_configuration_does_not_use_costmap_obstacles():
    nav2 = _yaml("minicar_nav2", "config/nav2_mppi_params.yaml")
    mppi = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    costmap = nav2["local_costmap"]["local_costmap"]["ros__parameters"]

    assert costmap["global_frame"] == "odom"
    assert costmap["robot_base_frame"] == "base_link"
    assert costmap["rolling_window"] is True
    assert costmap["plugins"] == []
    assert costmap["filters"] == []
    assert "CostCritic" not in mppi["critics"]
    assert "ObstaclesCritic" not in mppi["critics"]


def test_raceline_horizon_and_plugin_ids_are_consistent():
    nav2 = _yaml("minicar_nav2", "config/nav2_mppi_params.yaml")
    race = _yaml("minicar_raceline", "config/raceline_params.yaml")
    server = nav2["controller_server"]["ros__parameters"]
    mppi = server["FollowPath"]
    manager = race["raceline_manager"]["ros__parameters"]

    horizon_distance = mppi["vx_max"] * mppi["time_steps"] * mppi["model_dt"]
    assert manager["lookahead_m"] > mppi["prune_distance"] >= horizon_distance
    assert manager["controller_id"] in server["controller_plugins"]
    assert manager["goal_checker_id"] in server["goal_checker_plugins"]


def test_launch_keeps_a_single_command_chain_and_uses_imu_data():
    launch = (_share("minicar_nav2") / "launch/raceline_mppi.launch.py").read_text()

    assert '("cmd_vel", "/cmd_vel_raw")' in launch
    assert '"imu_topic": "/imu/data"' in launch
    assert 'package="minicar_safety"' in launch
    assert "raceline.launch.py" in launch
