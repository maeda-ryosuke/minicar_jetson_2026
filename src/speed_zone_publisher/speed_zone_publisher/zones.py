"""エリア定義の読み込みと、位置からの目標速度の判定。ROS に依存しないので単体テストできる。

エリアは map 座標の多角形と速度の組。どのエリアにも入らなければ default_speed、
重なっていれば遅いほうを使う。
"""

from dataclasses import dataclass

import yaml


@dataclass
class Zone:
    name: str
    speed: float
    polygon: list  # [[x, y], ...]


def load_zones(path):
    """zones.yaml を読み、(default_speed, [Zone, ...]) を返す。"""
    with open(path) as f:
        data = yaml.safe_load(f)
    zones = [Zone(z["name"], float(z["speed"]), [[float(x), float(y)] for x, y in z["polygon"]])
             for z in data.get("zones", [])]
    return float(data["default_speed"]), zones


def inside(polygon, x, y):
    """点 (x, y) が多角形の内側か(レイキャスティング)。"""
    result = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            result = not result
    return result


def target_speed(default_speed, zones, x, y):
    """位置 (x, y) の目標速度。"""
    speeds = [z.speed for z in zones if inside(z.polygon, x, y)]
    return min(speeds) if speeds else default_speed
