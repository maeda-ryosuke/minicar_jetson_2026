#!/usr/bin/env python3
"""Twist -> アクチュエータ指令の変換。ROS も I2C も要らない純粋な計算層。

    /cmd_vel (v[m/s], omega[rad/s])
        -> delta[rad]        自転車モデルの逆変換
        -> u_s, u_t ∈[-1,1]  校正マップ (デッドバンド・左右非対称をここで吸収)
        -> PWM カウント      PCA9685 の 12bit 値 (pwm_params.json の実測値)

この 3 段を分けてあるのは、更新される理由が段ごとに違うから。

  - 逆変換はリポジトリ共通の物理式で、変わらない。
  - 校正マップは「車両特性」。実車を計測して埋める
    (jetracer_ros2_handoff.md 【5-9】ステップ 1)。タイヤやサーボホーンを
    組み替えたら測り直す。
  - PWM カウントの3点は「サーボ / ESC の端点」。FaBo の 01_find_pwm.ipynb で
    測って pwm_params.json に保存する値そのもの。

段を混ぜて「delta -> カウント」の一枚マップにすると、タイヤを替えただけで
ESC の数字まで書き換えることになり、どこまでが実測でどこからが
部品仕様か分からなくなる。

=== 単位がカウントなのは FaBo に合わせたため ===

FaBo の実走 Notebook は PCA9685 の 12bit カウントを直接書いており、
校正値 pwm_params.json も

    {"pwm_steering": {"left":310, "center":410, "right":510},
     "pwm_speed":    {"back":390, "stop":410,   "front":430}}

というカウント値で持つ。us に翻訳して保持すると、実機で測った数字を
そのまま入れられなくなる。詳細は minicar_motor/pwm_backend.py の冒頭。

    colcon test --packages-select minicar_motor   # 単体テスト
"""

from dataclasses import dataclass, field
import math

import numpy as np


# 中立を表す指令。異常入力時はすべてここへ落とす。
# 「止める」ではなく「中立」なのは、サーボと ESC が最後のパルスを
# ラッチし続けるため。何も出さないことは停止を意味しない。
NEUTRAL_U = 0.0


def piecewise_linear(x: float, table: np.ndarray) -> float:
    """区分線形補間。テーブル範囲外は端点でクランプする。

    外挿しないのは、校正マップが「実測した範囲」だけを表すため。
    端の外は測っていないので、傾きを延長すると測っていない領域の値を
    でっち上げることになる。クランプなら少なくとも実測の端で頭打ちになる。
    """
    return float(np.interp(x, table[:, 0], table[:, 1]))


def as_table(pairs, name: str) -> np.ndarray:
    """[[x, y], ...] を検証して ndarray にする。

    x が昇順でないと np.interp が警告も出さずに壊れた値を返す。
    校正マップの並べ間違いは走ってみるまで気付けない種類の事故なので、
    読み込み時に落とす。
    """
    arr = np.asarray(pairs, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 2 or arr.shape[0] < 2:
        raise ValueError(f"{name} は [[x, y], ...] の 2 点以上で指定すること: {pairs}")
    if not np.all(np.diff(arr[:, 0]) > 0):
        raise ValueError(f"{name} の第1列は狭義単調増加で並べること: {arr[:, 0].tolist()}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} に有限でない値がある: {pairs}")
    return arr


@dataclass
class MotorConfig:
    """車両諸元 + 校正マップ + アクチュエータ仕様。

    車両諸元 (wheelbase / delta_max / v_min / v_max) は
    minicar_bringup/config/vehicle_params.yaml が唯一の出所。
    ここへ値を書き写さない。
    """

    # --- vehicle_params.yaml 由来 ---
    wheelbase: float = 0.257      # [m]
    delta_max: float = 0.42       # [rad]
    v_min: float = -1.0           # [m/s] 後退側の下限
    v_max: float = 2.0            # [m/s]

    # --- 変換 ---
    # これ未満の |v| では Twist に操舵情報が入っていない。
    # omega = v*tan(delta)/L なので v->0 では delta によらず omega->0 になり、
    # 逆変換すると 0/0 になる。この領域では直前の舵角を保持する。
    v_eps: float = 0.05           # [m/s]

    # --- 校正マップ (未校正の暫定値。実測で差し替える) ---
    # steer_map:    delta[rad] -> u_s ∈[-1,1]。サーボの極性反転もここで表す
    #               (delta が正で u が負になるテーブルを書けばよい)。
    # throttle_map: v[m/s]     -> u_t ∈[-1,1]。v=0 付近の段差が ESC の
    #               デッドバンド。前進側と後退側で段差の大きさを変えられる。
    steer_map: np.ndarray = field(
        default_factory=lambda: as_table([[-0.42, -1.0], [0.0, 0.0], [0.42, 1.0]],
                                         "steer_map"))
    throttle_map: np.ndarray = field(
        default_factory=lambda: as_table([[-1.0, -0.5], [0.0, 0.0], [2.0, 1.0]],
                                         "throttle_map"))

    # --- サーボ / ESC の端点 (pwm_params.json の実測値) ---
    # PCA9685 の 12bit カウント。FaBo の 01_find_pwm.ipynb で測る値そのもの。
    # left/right は「サーボの可動端」ではなく「そこまで切ってよい値」。
    # center は実測した中立で、left と right の中点とは限らない。
    steer_left: int = 310         # u_s = -1 のカウント
    steer_center: int = 410       # u_s =  0 のカウント (実測中立)
    steer_right: int = 510        # u_s = +1 のカウント
    throttle_back: int = 390      # u_t = -1 のカウント
    throttle_stop: int = 410      # u_t =  0 のカウント (ESC の中立)
    throttle_front: int = 430     # u_t = +1 のカウント

    # 最後の砦。PCA9685 の分解能そのもの。ここを越えることは無いはずだが、
    # pwm_params.json を手で書き換えたときに効く。
    count_min: int = 0
    count_max: int = 4095

    @classmethod
    def from_yaml(cls, vehicle: dict, motor: dict,
                  pwm: dict | None = None) -> "MotorConfig":
        """vehicle_params.yaml / motor_driver_params.yaml / pwm_params.json から組む。

        pwm は FaBo の pwm_params.json をそのまま読んだ dict。
        キー構成は FaBo の 01_find_pwm.ipynb が書き出すものと同一にしてある
        (実機で測った JSON を変換せずにそのまま置けるようにするため)。
        """
        c = cls()
        v, lim = vehicle["vehicle"], vehicle["limits"]
        c.wheelbase = float(v["wheelbase"])
        c.delta_max = float(lim["delta_max"])
        c.v_min = float(lim["v_min"])
        c.v_max = float(lim["v_max"])

        if "steer_map" in motor:
            c.steer_map = as_table(motor["steer_map"], "steer_map")
        if "throttle_map" in motor:
            c.throttle_map = as_table(motor["throttle_map"], "throttle_map")
        if "v_eps" in motor:
            c.v_eps = float(motor["v_eps"])
        for k in ("count_min", "count_max"):
            if k in motor:
                setattr(c, k, int(motor[k]))

        if pwm is not None:
            try:
                st, sp = pwm["pwm_steering"], pwm["pwm_speed"]
                c.steer_left = int(st["left"])
                c.steer_center = int(st["center"])
                c.steer_right = int(st["right"])
                c.throttle_back = int(sp["back"])
                c.throttle_stop = int(sp["stop"])
                c.throttle_front = int(sp["front"])
            except (KeyError, TypeError, ValueError) as e:
                # 形が違う JSON を黙って既定値で流すと、未校正の暫定値で
                # 実機に通電することになる。ここで落とす。
                raise ValueError(
                    "pwm_params.json の形が FaBo の 01_find_pwm.ipynb の出力と違う。"
                    '{"pwm_steering":{"left","center","right"},'
                    ' "pwm_speed":{"front","stop","back"}} であること: '
                    f"{e}") from e

        c.validate()
        return c

    def validate(self) -> None:
        """走らせる前に落とせる矛盾をここで落とす。

        校正マップの取り違えは、走行中は「なんとなく曲がらない」としてしか
        現れず、原因の特定に時間がかかる。起動時に落とすほうが安い。
        """
        if self.wheelbase <= 0.0:
            raise ValueError(f"wheelbase は正であること: {self.wheelbase}")
        if self.delta_max <= 0.0:
            raise ValueError(f"delta_max は正であること: {self.delta_max}")
        if not (self.v_min < 0.0 <= self.v_max):
            raise ValueError(f"v_min < 0 <= v_max であること: {self.v_min}, {self.v_max}")
        if self.v_eps <= 0.0:
            raise ValueError(f"v_eps は正であること: {self.v_eps}")

        # PWM カウントの3点。中立が端点の間に無いと、u=0 を指令したときの
        # 出力が「左いっぱい」や「全開」になりうる。走ってみるまで気付けない
        # 種類の事故なので起動時に落とす。
        for name, lo, mid, hi in (
                ("pwm_steering", self.steer_left, self.steer_center, self.steer_right),
                ("pwm_speed", self.throttle_back, self.throttle_stop,
                 self.throttle_front)):
            if not (min(lo, hi) <= mid <= max(lo, hi)):
                raise ValueError(
                    f"{name} の中立 {mid} が端点 {lo}..{hi} の間に無い")
            if lo == hi:
                raise ValueError(
                    f"{name} の端点が同じ値 ({lo})。振れ幅が 0 になる")
            for v in (lo, mid, hi):
                if not (self.count_min <= v <= self.count_max):
                    raise ValueError(
                        f"{name} の値 {v} が count_min..count_max "
                        f"({self.count_min}..{self.count_max}) の外")
        if self.count_min >= self.count_max:
            raise ValueError(
                f"count_min < count_max であること: "
                f"{self.count_min} >= {self.count_max}")

        # マップの出力が [-1,1] を外れていると、パルス幅のクランプに
        # 頼ることになる。頼った時点で「マップの端 = 実測の端」ではなく
        # なるので、ここで弾く。
        for name, table in (("steer_map", self.steer_map),
                            ("throttle_map", self.throttle_map)):
            u = table[:, 1]
            if np.any(np.abs(u) > 1.0 + 1e-9):
                raise ValueError(f"{name} の出力は [-1, 1] に収めること: {u.tolist()}")

        # 操舵マップが delta_max の全域を覆っていないと、車両限界まで
        # 舵を切れない。MPPI/MPC 側は delta_max まで使う前提で解いている。
        if self.steer_map[0, 0] > -self.delta_max + 1e-9 or \
           self.steer_map[-1, 0] < self.delta_max - 1e-9:
            raise ValueError(
                f"steer_map が delta_max=±{self.delta_max} を覆っていない: "
                f"{self.steer_map[0, 0]}..{self.steer_map[-1, 0]}")


@dataclass(frozen=True)
class MotorCommand:
    """1 周期ぶんの出力。debug publish とテストのためすべての中間値を持つ。"""

    delta_rad: float       # 逆変換 + クランプ後の舵角
    u_steer: float         # 正規化操舵指令 ∈[-1,1]
    u_throttle: float      # 正規化スロットル指令 ∈[-1,1]
    v_cmd: float           # クランプ後の速度指令 [m/s]
    steer_saturated: bool  # delta_max に張り付いたか
    valid: bool            # 入力が有限で、変換できたか


NEUTRAL_COMMAND = MotorCommand(
    delta_rad=0.0, u_steer=NEUTRAL_U, u_throttle=NEUTRAL_U,
    v_cmd=0.0, steer_saturated=False, valid=False,
)


class MotorDriverCore:
    """Twist -> 正規化指令。状態を持たない。

    「直前の舵角」は呼び出し側から渡す。ノード側が保持している値
    (スルーレート制限後の実際の指令) と、ここが使う値を必ず一致させたいため。
    内部に隠し持つと、ノードが中立を割り込ませた後にずれる。
    """

    def __init__(self, cfg: MotorConfig):
        cfg.validate()
        self.cfg = cfg

    # ------------------------------------------------------------------
    def steer_angle(self, v: float, omega: float, delta_prev: float = 0.0) -> float:
        """Twist から前輪舵角を復元する [rad]。

        リポジトリ全体が delta = atan(omega * L / v) で一致している
        (mppi_node.py:175 が omega = v*tan(delta)/L で出し、
         steer_debug.py:209 が同じ式で読み戻している)。
        ここはその最終段なので、同じ式を使わないと
        「MPPI が要求した舵角」と「車輪が向く角」が食い違う。

        |v| < v_eps では直前の舵角を保持する。0 にしないのは、
        safety_node の後退脱出が v を 0 を跨いで反転させるため
        (minicar_safety の safety_node の後退脱出)。そこで舵を中立へ戻すと、
        切り返しの途中で一瞬まっすぐになり脱出できなくなる。
        """
        if abs(v) < self.cfg.v_eps:
            delta = delta_prev
        else:
            delta = math.atan(omega * self.cfg.wheelbase / v)
        return float(np.clip(delta, -self.cfg.delta_max, self.cfg.delta_max))

    def steer_to_u(self, delta: float) -> float:
        """舵角[rad] -> 正規化操舵指令 ∈[-1,1]。

        ノードはスルーレート制限を掛けた**後の**舵角でここを引く。
        制限前の値で引くと、publish する舵角と実際にサーボへ送る値が
        食い違い、steer_debug の突き合わせが意味を持たなくなる。
        """
        if not math.isfinite(delta):
            return NEUTRAL_U
        delta = float(np.clip(delta, -self.cfg.delta_max, self.cfg.delta_max))
        return float(np.clip(piecewise_linear(delta, self.cfg.steer_map), -1.0, 1.0))

    def throttle_to_u(self, v: float) -> float:
        """速度指令[m/s] -> 正規化スロットル指令 ∈[-1,1]。"""
        if not math.isfinite(v):
            return NEUTRAL_U
        v = float(np.clip(v, self.cfg.v_min, self.cfg.v_max))
        return float(np.clip(piecewise_linear(v, self.cfg.throttle_map), -1.0, 1.0))

    def convert(self, v: float, omega: float, delta_prev: float = 0.0) -> MotorCommand:
        """Twist 1 つを正規化指令へ変換する。

        非有限値は中立で返す。例外を投げないのは、制御ノードが NaN を
        出した瞬間にドライバまで道連れで落ちると、ESC が最後のパルスを
        ラッチしたまま誰も止められなくなるため。止まって生き残るほうが安全。
        """
        if not (math.isfinite(v) and math.isfinite(omega) and math.isfinite(delta_prev)):
            return NEUTRAL_COMMAND

        delta = self.steer_angle(v, omega, delta_prev)
        v_cmd = float(np.clip(v, self.cfg.v_min, self.cfg.v_max))

        return MotorCommand(
            delta_rad=delta,
            u_steer=self.steer_to_u(delta),
            u_throttle=self.throttle_to_u(v_cmd),
            v_cmd=v_cmd,
            steer_saturated=abs(delta) >= self.cfg.delta_max - 1e-9,
            valid=True,
        )

    # ------------------------------------------------------------------
    def to_count(self, u: float, kind: str) -> int:
        """正規化指令 -> PCA9685 の 12bit カウント。kind は "steer" / "throttle"。

        **3 点区分線形**にしてある。u=0 が center/stop、操舵は u=+1 が left、
        u=-1 が right、スロットルは u=+1 が front、u=-1 が back。

        操舵の符号は REP-103 に合わせる (delta>0 = omega>0 = 左旋回)。
        steer_map は delta と u を同符号で並べるので、u=+1 が right だと
        左を指令して右へ切れる (実機で確認済みの不具合)。

        === FaBo の map_rc との意図的な差分 ===

        FaBo の実走 Notebook は

            map_rc(x, 224, 0, pwm_right, pwm_left)

        という **left..right の 2 点線形**で引いており、実測した center を
        出力に使っていない (center は起動時に 1 回書くだけ)。左右が center に
        対して非対称だと、中央を指令したときの出力が実測中立からずれる。

        FaBo の用途 (カメラ画像からの推論値を流し込む) ではそれで足りるが、
        こちらは delta=0 を頻繁に出す ── 停止中、直線、ウォッチドッグの
        中立出力、後退シーケンスの中立ギャップ。ここが実測中立でないと、
        「止めたつもりなのにじわじわ曲がる」状態になる。
        よって center/stop を折れ点として使う。

        クランプは最後の砦。端点の外へ出るとサーボが機械端に突っ込んで
        ギアを舐めるか、ESC が異常値として入力を無視する。
        """
        if not math.isfinite(u):
            u = NEUTRAL_U
        u = float(np.clip(u, -1.0, 1.0))
        if kind == "steer":
            lo, mid, hi = (self.cfg.steer_right, self.cfg.steer_center,
                           self.cfg.steer_left)
        elif kind == "throttle":
            lo, mid, hi = (self.cfg.throttle_back, self.cfg.throttle_stop,
                           self.cfg.throttle_front)
        else:
            raise ValueError(f'kind は "steer" か "throttle": {kind!r}')
        count = mid + u * ((hi - mid) if u >= 0.0 else (mid - lo))
        return int(round(float(np.clip(count, self.cfg.count_min,
                                       self.cfg.count_max))))


# ======================================================================
# 速度フィードバック
# ======================================================================

@dataclass(frozen=True)
class SpeedOutput:
    """1 周期ぶんの速度制御の出力。debug publish 用にすべての項を持つ。"""

    u: float        # 最終スロットル u_t = u_ff + u_p + u_i を [0, u_max] でクランプ
    u_ff: float     # フィードフォワード (throttle_map の値)
    u_p: float      # 比例項
    u_i: float      # 積分項
    error: float    # v_cmd - v_meas [m/s]


class SpeedController:
    """前進側の速度 PI。throttle_map をフィードフォワードにして、足りない分を足す。

        u_t = u_ff(v_cmd) + kp * e + ki * ∫e dt,   e = v_cmd - v_meas

    === なぜ要るか ===

    ESC + ブラシモータは「電圧の指令」で、同じカウントでも負荷で速度が変わる。
    実走 bag (2026-10-16 18:25 / 19:00) では:

      - コース上の実速度は指令の約 7 割 (床で作った throttle_map が合わない)
      - 舵いっぱいのコーナーやつなぎ目で停止し、**同じカウントのまま
        7〜100 秒以上動き出せなかった**。オープンループでは止まっても出力が増えない

    積分項は止まっている間も誤差 (>= 0.5 m/s) を溜め続けるので、静止摩擦を
    越えるまで出力を上げて押し出す。

    === 安全側の設計 ===

      - 出力は [0, u_max]。PI でブレーキ・後退は出さない (後退は従来どおり
        後退シーケンス経由のオープンループ)
      - 積分項は [i_min, i_max] で頭打ち。/odom が壊れて 0 を返し続けても、
        出力は u_ff + kp*v_cmd + i_max までしか上がらない
      - 出力が上限 (または 0) に張り付いている間は、その向きへ積分しない
        (アンチワインドアップ)。解除した瞬間に溜めた分で飛び出さない
      - 停止指令・中立・/odom 途絶のたびに reset() で積分を捨てる (ノード側)
    """

    def __init__(self, kp: float, ki: float, i_min: float, i_max: float,
                 u_max: float):
        if kp < 0.0 or ki < 0.0:
            raise ValueError(f"speed_kp / speed_ki は 0 以上であること: {kp}, {ki}")
        if not (i_min <= 0.0 <= i_max):
            raise ValueError(
                f"speed_i_min <= 0 <= speed_i_max であること: {i_min}, {i_max}")
        if not (0.0 < u_max <= 1.0):
            raise ValueError(f"speed_u_max は (0, 1] であること: {u_max}")
        self.kp, self.ki = float(kp), float(ki)
        self.i_min, self.i_max = float(i_min), float(i_max)
        self.u_max = float(u_max)
        self.u_i = 0.0

    def reset(self) -> None:
        self.u_i = 0.0

    def update(self, v_cmd: float, v_meas: float, u_ff: float,
               dt: float) -> SpeedOutput:
        """1 周期進める。入力が非有限なら積分を捨てて u_ff だけを返す。"""
        if not all(math.isfinite(x) for x in (v_cmd, v_meas, u_ff, dt)) or dt <= 0.0:
            self.reset()
            u = float(np.clip(u_ff, 0.0, self.u_max)) if math.isfinite(u_ff) else 0.0
            return SpeedOutput(u=u, u_ff=u_ff, u_p=0.0, u_i=0.0, error=0.0)

        e = v_cmd - v_meas
        u_p = self.kp * e
        u_i = float(np.clip(self.u_i + self.ki * e * dt, self.i_min, self.i_max))
        u_raw = u_ff + u_p + u_i
        # 条件付き積分: 張り付いた向きへさらに溜めない。
        if (u_raw > self.u_max and e > 0.0) or (u_raw < 0.0 and e < 0.0):
            u_i = self.u_i
            u_raw = u_ff + u_p + u_i
        self.u_i = u_i
        u = float(np.clip(u_raw, 0.0, self.u_max))
        return SpeedOutput(u=u, u_ff=u_ff, u_p=u_p, u_i=u_i, error=e)
