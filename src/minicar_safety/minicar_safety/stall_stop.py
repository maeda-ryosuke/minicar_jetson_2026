"""停止検知と後退による復帰の判定部分。ROS に依存しないので、そのまま単体テストできる。

「前進を指令しているのに /odom (車輪エンコーダ) の実速度が 0」が
stall_time 続いたら、recovery_duration だけ後退して素通しへ戻る。

空転は見ない。/odom は車輪の回転なので、壁に押し付けられて車輪が
空回りしていると実速度が出ているように見え、ここでは検知できない。
芝生や壁で車輪ごと止まる場面 (速度 PI が出力上限に張り付いたまま
実速度 0) だけを確実に拾う。

後退しても抜けられず max_recoveries 回続けて詰まったら、止めてラッチする
(reset() を呼ぶまで止めたまま)。前進で arm_speed を超えれば回数は 0 に戻る。
"""

from dataclasses import dataclass
from typing import Optional, Tuple

PASS = "pass"         # 素通し
RECOVER = "recover"   # 後退中
STOP = "stop"         # 復帰をあきらめて停止 (ラッチ)


@dataclass
class StallConfig:
    # これより大きい前進指令のときだけ判定する [m/s]。
    # 後退・停止指令は対象外 (止まっていて当然)。
    cmd_min: float = 0.1
    # 実速度がこれ未満なら「止まっている」[m/s]。
    # /odom の分解能は 0.026 m/s なので、0.03 は「0 か 1 段」。
    speed_zero: float = 0.03
    # 止まっている状態がこれだけ続いたら後退 [s]。
    stall_time: float = 1.0
    # 起動 / リセット後、一度この速度を超えるまでは判定しない [m/s]。
    arm_speed: float = 0.2
    # /odom がこれ以上来ていなければ判定しない [s]。
    # 途絶時の停止は motor_driver の odom_lost_stop の担当。
    odom_timeout: float = 0.3
    # 後退の速さ [m/s] (正で指定、内部で負にする) と時間 [s]。
    recovery_speed: float = 0.4
    recovery_duration: float = 1.5
    # 前進できないまま続けてこの回数後退したら、あきらめて停止する。
    max_recoveries: int = 3

    def validate(self) -> None:
        if not (self.cmd_min >= 0.0 and self.speed_zero > 0.0
                and self.stall_time > 0.0 and self.odom_timeout > 0.0
                and self.recovery_speed > 0.0 and self.recovery_duration > 0.0
                and self.max_recoveries >= 0):
            raise ValueError(
                f"cmd_min >= 0, speed_zero / stall_time / odom_timeout / recovery_speed / "
                f"recovery_duration > 0, max_recoveries >= 0 であること: {self}")
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
        self.mode = PASS
        self.recoveries = 0         # 前進できないまま続けて後退した回数
        self.recover_kappa = 0.0    # 後退中の曲率 [1/m]
        self._since: Optional[float] = None
        self._recover_until = 0.0

    def update_odom(self, v: float, t: float) -> None:
        self.v_odom = v
        self._odom_t = t
        # 発進前は判定しない。プロポのマニュアルモードで待っている間や
        # 置いた直後は、指令が出ていても止まっているのが正常なので。
        if abs(v) >= self.cfg.arm_speed:
            self.armed = True
        # 前進で走れたら、後退で抜けられたとみなす。
        if self.mode == PASS and v >= self.cfg.arm_speed:
            self.recoveries = 0

    def update(self, v_cmd: float, omega_cmd: float, t: float) -> Optional[Tuple[float, float]]:
        """指令を受けるたびに呼ぶ。None なら素通し。そうでなければ代わりに出す (v, omega)。"""
        c = self.cfg
        if self.mode == STOP:
            return (0.0, 0.0)
        if self.mode == RECOVER:
            if t < self._recover_until:
                v = -c.recovery_speed
                return (v, v * self.recover_kappa)
            self.mode = PASS
            self._since = None

        fresh = self._odom_t is not None and t - self._odom_t <= c.odom_timeout
        if not (self.armed and fresh and v_cmd > c.cmd_min
                and abs(self.v_odom) < c.speed_zero):
            self._since = None
            return None
        if self._since is None:
            self._since = t
        if t - self._since < c.stall_time:
            return None

        # 詰まった。
        self._since = None
        if self.recoveries >= c.max_recoveries:
            self.mode = STOP
            return (0.0, 0.0)
        # 切り返しと同じ要領で、後退時は舵を逆に切る。omega = v * kappa なので、
        # 前進で右へ曲がろうとして詰まったなら、後退では左に切る。
        self.recoveries += 1
        self.recover_kappa = -omega_cmd / v_cmd
        self.mode = RECOVER
        self._recover_until = t + c.recovery_duration
        v = -c.recovery_speed
        return (v, v * self.recover_kappa)

    def reset(self) -> None:
        """停止のラッチを外す。再び走り出すまでは判定しない。"""
        self.mode = PASS
        self.armed = False
        self.recoveries = 0
        self._since = None
