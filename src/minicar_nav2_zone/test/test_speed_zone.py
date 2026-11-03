"""速度帯の判定ロジックと、設定ファイル間の契約を検証する。"""

import math
from pathlib import Path

import pytest
import yaml

from minicar_nav2_zone.speed_zone import ZoneSwitch, in_any, is_slow, parse_zones


PKG = Path(__file__).resolve().parents[1]


def _zone_params():
    with (PKG / "config/speed_zone_params.yaml").open() as f:
        return yaml.safe_load(f)["speed_zone"]["ros__parameters"]


def test_parse_zones_rejects_bad_input():
    with pytest.raises(ValueError):
        parse_zones([0.0, 1.0, 0.0])
    with pytest.raises(ValueError):
        parse_zones([1.0, 0.0, 0.0, 1.0])     # x_min > x_max
    assert len(parse_zones([0, 1, 0, 1, 2, 3, 2, 3])) == 2


def test_preview_slows_before_entering():
    zones = parse_zones([2.0, 3.0, -1.0, 1.0])
    # 矩形の 1m 手前を東向きに走っている。
    assert not in_any(zones, 1.0, 0.0)
    assert is_slow(zones, 1.0, 0.0, 0.0, 1.25)
    # 西向き(遠ざかる向き)なら先読みしても入らない。
    assert not is_slow(zones, 1.0, 0.0, math.pi, 1.25)
    # 抜けた直後(現在位置は外、先読み点も外)は高速。
    assert not is_slow(zones, 3.1, 0.0, 0.0, 1.25)


def test_switch_holds_state_for_min_hold():
    sw = ZoneSwitch(0.5)
    assert sw.update(False, 0.0)           # 初回は必ず変化扱い
    assert sw.update(True, 0.1) is False   # hold 中
    assert sw.slow is False
    assert sw.update(True, 0.6)
    assert sw.slow is True
    assert sw.update(True, 0.7) is False


def test_speed_bands_and_zones_in_yaml():
    p = _zone_params()
    assert 0.0 <= p["slow_vx_min"] <= p["slow_vx_max"]
    assert 0.0 <= p["fast_vx_min"] <= p["fast_vx_max"]
    assert p["slow_vx_max"] <= p["fast_vx_max"]
    assert parse_zones(p["slow_zones"])


def test_mppi_horizon_fits_prune_distance_at_fast_speed():
    """fast_vx_max で予測ホライゾンが prune_distance に収まること(minicar_nav2 の yaml)。"""
    from ament_index_python.packages import get_package_share_directory

    nav2 = Path(get_package_share_directory("minicar_nav2"))
    with (nav2 / "config/nav2_mppi_params.yaml").open() as f:
        mppi = yaml.safe_load(f)["controller_server"]["ros__parameters"]["FollowPath"]
    horizon = _zone_params()["fast_vx_max"] * mppi["time_steps"] * mppi["model_dt"]
    assert horizon <= mppi["prune_distance"] + 1e-9
