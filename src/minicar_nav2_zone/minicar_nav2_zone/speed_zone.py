"""自己位置から速度帯(低速/高速)を決める。ROS 非依存。

低速域は map 座標の矩形の集まり。現在位置か、進行方向に preview 先の点の
どちらかが矩形内なら低速にする(手前で減速を始め、抜け切るまで低速)。
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Rect:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x, y):
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


def parse_zones(flat):
    """[x_min, x_max, y_min, y_max, ...] のフラット配列を Rect の列にする。"""
    flat = [float(v) for v in flat]
    if len(flat) % 4 != 0:
        raise ValueError(
            f"slow_zones は 4 個ずつ [x_min, x_max, y_min, y_max]: 長さ {len(flat)}")
    zones = []
    for i in range(0, len(flat), 4):
        r = Rect(*flat[i:i + 4])
        if not (r.x_min < r.x_max and r.y_min < r.y_max):
            raise ValueError(f"slow_zones の {i // 4} 個目は min < max であること: {r}")
        zones.append(r)
    return zones


def in_any(zones, x, y):
    return any(r.contains(x, y) for r in zones)


def is_slow(zones, x, y, yaw, preview):
    """現在位置か、進行方向に preview[m] 先の点が低速域に入っているか。"""
    px = x + preview * math.cos(yaw)
    py = y + preview * math.sin(yaw)
    return in_any(zones, x, y) or in_any(zones, px, py)


class ZoneSwitch:
    """判定結果を min_hold 秒は保持する(境界でのばたつき防止)。"""

    def __init__(self, min_hold):
        self.min_hold = float(min_hold)
        self.slow = None        # 未判定
        self._t_changed = None

    def update(self, slow, now):
        """新しい判定を入れ、状態が変わったら True を返す。初回は必ず True。"""
        if self.slow is None:
            self.slow, self._t_changed = bool(slow), now
            return True
        if bool(slow) == self.slow or now - self._t_changed < self.min_hold:
            return False
        self.slow, self._t_changed = bool(slow), now
        return True
