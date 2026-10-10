"""Nav2 MPPI設定と既存パッケージ間の契約を静的に検証する。"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from minicar_nav2.vehicle_limits import min_turning_radius


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
    # 制御周期 < model_dt にして制御列シフトを OFF にする(シムで精度が良かった)。
    # 等しいとシフト ON になり、制御列の 1 ステップ先が指令として出る。
    assert 1.0 / server["controller_frequency"] < mppi["model_dt"]
    assert mppi["motion_model"] == "Ackermann"
    # 舵角の出所は vehicle_params。yaml の既定値も同じ式の値にそろえておく。
    r_min = min_turning_radius(
        _share("minicar_bringup") / "config/vehicle_params.yaml")
    assert math.isclose(
        mppi["AckermannConstraints"]["min_turning_r"], r_min, abs_tol=1e-3)
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
    # 障害物の入力源を持つ層(obstacle / voxel / static)を入れない。
    assert costmap["plugins"] == ["inflation_layer"]
    assert costmap["inflation_layer"]["plugin"] == \
        "nav2_costmap_2d::InflationLayer"
    assert "filters" not in costmap
    assert "CostCritic" not in mppi["critics"]
    assert "ObstaclesCritic" not in mppi["critics"]


def _walk(node, path=""):
    if isinstance(node, dict):
        for k, v in node.items():
            yield from _walk(v, f"{path}.{k}" if path else k)
    else:
        yield path, node


def test_no_parameter_that_aborts_humble_controller_server():
    """Humble の controller_server が起動時に abort する書き方を検出する(実測)。

    * 空リストは rclcpp が型を決められず「No parameter value set」になる。
    * costmap の width / height は int 型で、8.0 と書くと invalid type になる。
    """
    nav2 = _yaml("minicar_nav2", "config/nav2_mppi_params.yaml")
    empty = [p for p, v in _walk(nav2) if v == []]
    assert empty == [], f"空リストは Humble で起動不能: {empty}"
    costmap = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    for key in ("width", "height"):
        assert type(costmap[key]) is int, f"{key} は int で書くこと"


def test_launch_takes_turning_radius_from_vehicle_params():
    launch = (_share("minicar_nav2") / "launch/raceline_mppi.launch.py").read_text()

    assert "min_turning_radius(vehicle_params_file)" in launch
    assert '"AckermannConstraints": {"min_turning_r": r_min}' in launch


def test_min_turning_radius_formula(tmp_path):
    f = tmp_path / "vehicle.yaml"
    f.write_text("vehicle: {wheelbase: 0.257}\nlimits: {delta_max: 0.2516}\n")
    assert math.isclose(min_turning_radius(f), 0.257 / math.tan(0.2516))


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
