"""停止検知の判定部分。ROS に依存しないので、そのまま単体テストできる。

「前進を指令しているのに /odom (車輪エンコーダ) の実速度が 0」が
stall_time 続いたら止める。それだけ。

空転は見ない。/odom は車輪の回転なので、壁に押し付けられて車輪が
空回りしていると実速度が出ているように見え、ここでは検知できない。
芝生や壁で車輪ごと止まる場面 (速度 PI が出力上限に張り付いたまま
実速度 0) だけを確実に拾う。

一度止めたらラッチする。reset() を呼ぶまで止めたまま。
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class StallConfig:
    # これより大きい前進指令のときだけ判定する [m/s]。
    # 後退・停止指令は対象外 (止まっていて当然)。
    cmd_min: float = 0.1
    # 実速度がこれ未満なら「止まっている」[m/s]。
    # /odom の分解能は 0.026 m/s なので、0.03 は「0 か 1 段」。
    speed_zero: float = 0.03
    # 止まっている状態がこれだけ続いたら停止 [s]。
    stall_time: float = 1.0
    # 起動 / リセット後、一度この速度を超えるまでは判定しない [m/s]。
    arm_speed: float = 0.2
    # /odom がこれ以上来ていなければ判定しない [s]。
    # 途絶時の停止は motor_driver の odom_lost_stop の担当。
    odom_timeout: float = 0.3

    def validate(self) -> None:
        if not (self.cmd_min >= 0.0 and self.speed_zero > 0.0
                and self.stall_time > 0.0 and self.odom_timeout > 0.0):
            raise ValueError(
                f"cmd_min >= 0, speed_zero > 0, stall_time > 0, odom_timeout > 0 であること: "
                f"{self}")
        if self.arm_speed <= self.speed_zero:
            raise ValueError(
                f"arm_speed ({self.arm_speed}) は speed_zero ({self.speed_zero}) より"
                f"大きくすること。でないと止まったままでも判定が始まる")


class StallDetector:
    """時刻 t [s] は呼び出し側が渡す (ROS の clock でも、bag の時刻でもよい)。"""

    def __init__(self, cfg: StallConfig):
        cfg.validate()
        self.cfg = cfg
        self.v_odom: Optional[float] = None
        self._odom_t: Optional[float] = None
        self.armed = False          # 起動 / リセット後に一度走り出したか
        self.tripped = False        # ラッチ中
        self._since: Optional[float] = None

    def update_odom(self, v: float, t: float) -> None:
        self.v_odom = v
        self._odom_t = t
        # 発進前は判定しない。プロポのマニュアルモードで待っている間や
        # 置いた直後は、指令が出ていても止まっているのが正常なので。
        if abs(v) >= self.cfg.arm_speed:
            self.armed = True

    def update(self, v_cmd: float, t: float) -> bool:
        """指令を受けるたびに呼ぶ。止めるべきなら True (ラッチ)。"""
        if self.tripped:
            return True
        c = self.cfg
        fresh = self._odom_t is not None and t - self._odom_t <= c.odom_timeout
        if (self.armed and fresh and v_cmd > c.cmd_min
                and abs(self.v_odom) < c.speed_zero):
            if self._since is None:
                self._since = t
            elif t - self._since >= c.stall_time:
                self.tripped = True
            return self.tripped
        self._since = None
        return False

    def stalled_for(self, t: float) -> float:
        """止まっている状態が続いている秒数 (ログ用)。"""
        return 0.0 if self._since is None else t - self._since

    def reset(self) -> None:
        """ラッチを外す。再び走り出すまでは判定しない。"""
        self.tripped = False
        self.armed = False
        self._since = None
