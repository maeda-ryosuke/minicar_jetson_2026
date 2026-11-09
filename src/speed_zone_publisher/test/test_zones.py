"""エリア判定のテスト。"""

from pathlib import Path

from speed_zone_publisher.zones import Zone, load_zones, target_speed


SQUARE = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]


def test_inside_zone_uses_zone_speed():
    zones = [Zone("a", 1.5, SQUARE)]
    assert target_speed(3.0, zones, 1.0, 1.0) == 1.5


def test_outside_zone_uses_default_speed():
    zones = [Zone("a", 1.5, SQUARE)]
    assert target_speed(3.0, zones, 3.0, 1.0) == 3.0


def test_overlap_uses_slower_speed():
    zones = [Zone("a", 2.0, SQUARE), Zone("b", 1.2, [[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]])]
    assert target_speed(3.0, zones, 1.5, 1.5) == 1.2
    assert target_speed(3.0, zones, 0.5, 0.5) == 2.0


def test_config_loads():
    default_speed, zones = load_zones(Path(__file__).resolve().parents[1] / "config/zones.yaml")
    assert default_speed > 0.0
    assert all(len(z.polygon) >= 3 for z in zones)
