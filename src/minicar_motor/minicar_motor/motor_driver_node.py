#!/usr/bin/env python3
"""モータドライバ (実機) — 指令チェーンの出口。docs/LIDDER_MAPII.PNG の "Motor Driver"。

    [mppi/nav2] --/cmd_vel_raw--> [safety_node] --/cmd_vel--> [motor_driver] --PWM--> 車両

Twist を舵角とスロットルへ変換し、PCA9685 の PWM として出す。変換そのものは
motor_driver_core.py (ROS 非依存)、PWM の書き込みは pwm_backend.py が持つ。
このファイルは購読・時間・状態遷移・publish だけを持つ。

責務は 4 つ。

  1. 変換        Twist -> delta -> 正規化指令 -> PWM カウント
  2. ウォッチドッグ  指令が途切れたら中立を出し続ける
  3. アーミング   起動直後は中立を出し、ESC が中立を検出できるようにする
  4. 後退シーケンス ESC が後退に入るための 中立->中立->後退 (FaBo 準拠)

なぜ safety_node と分けるか: あちらは「車両運動の安全」(加速度制限・
スタック脱出・上流の死活監視) を見ていて、制御方式に依存しない。
こちらは「アクチュエータの物理」(サーボの可動端、ESC のアーミングと
後退シーケンス、PWM の範囲) を見ていて、車体のハードに依存する。
混ぜると、制御を差し替えるたびにハード固有の知識を作り直すことになる。

=== 2, 3 と終了時の中立は FaBo には無い。こちらで足したもの ===

FaBo の実走 Notebook はウォッチドッグもタイムアウトもデストラクタも持たず、
Stop ボタンでスロットルだけ中立へ戻す (ステアは切ったまま)。推論スレッドが
例外で死ぬと PWM は最後の値を保持し続ける。自律走行では上流が落ちることを
前提にしないといけないので、次をすべてこちら側で持つ:

    起動直後の中立出力 / cmd_timeout / NaN で中立 / 終了時 finally で中立
    / 停止時に舵をスルーレート制限つきで中央へ戻す

舵を中央へ戻すのは、切ったまま保持するとサーボがタイヤの復元力に対して
ストールし続けて発熱するため。

=== 物理的な最後の砦はソフトではない ===

FaBo の基板は RC サーボマルチプレクサ (Pololu 2806) でプロポ手動と AI を
切り替える。**プロポ側が AI モードでないと、このノードが何を出しても車は
動かない。** 逆に言えば、暴走したらプロポで切れば止まる。初通電のときは
必ずプロポを手元に置くこと。

起動は launch から:  ros2 launch minicar_raceline raceline.launch.py
単体で上げるときは minicar_motor/config の YAML と JSON、および
minicar_bringup の vehicle_params.yaml のパスを --ros-args で渡す。
"""

import json

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from minicar_motor.motor_driver_core import MotorConfig, MotorDriverCore
from minicar_motor.pwm_backend import make_backend

REVERSE_MODES = ("neutral_neutral_reverse", "direct", "disabled")


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver")

        self.declare_parameter("rate", 50.0)
        self.declare_parameter("cmd_topic", "/cmd_vel")
        # 車両諸元は minicar_bringup の YAML。**必須**。既定を持たせないのは、
        # 決め打ちにすると「別の諸元で走らせているつもりで既定を読んでいた」に
        # 気付けないため (minicar_ftg / minicar_mppi と同じ作法)。
        self.declare_parameter("vehicle_params_file", "")
        self.declare_parameter("cmd_timeout", 0.3)
        self.declare_parameter("arm_neutral_duration", 2.0)
        self.declare_parameter("steer_slew_rate_max", 10.0)
        self.declare_parameter("reverse_mode", "neutral_neutral_reverse")
        # FaBo の time.sleep(0.1) に合わせる。中立を 2 回挟むので合計 0.2s。
        self.declare_parameter("neutral_gap_duration", 0.1)

        self.declare_parameter("backend", "dryrun")
        self.declare_parameter("steering_channel", 0)
        self.declare_parameter("throttle_channel", 1)
        # Orin Nano は 7 (FaBo の notebooks/17_run.ipynb の board_settings)。
        # Nano=1, NX/Xavier=8。実機で i2cdetect -y -r 7 に 0x40 が出るか確認する。
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("i2c_address", 0x40)
        # FaBo の実走 Notebook は 60Hz。50 と取り違えると、カウントは
        # デューティ比なのでパルス幅が一様に 1.2 倍ずれる。
        self.declare_parameter("pwm_freq_hz", 60.0)

        # 校正値 (PCA9685 の 12bit カウント)。FaBo の 01_find_pwm.ipynb が
        # 書き出す JSON をそのまま置く。形を変えないのは、実機で測った
        # ファイルを変換せずにコピーできるようにするため。
        self.declare_parameter("pwm_params_file", "")
        self.declare_parameter("v_eps", 0.05)
        self.declare_parameter("count_min", 0)
        self.declare_parameter("count_max", 4095)

        # ROS パラメータは配列の配列を持てないので x と y を別配列で受け、
        # ここで対にする。長さ違いと非単調は MotorConfig 側で落ちる。
        self.declare_parameter("steer_map_delta_rad", [-0.42, 0.0, 0.42])
        self.declare_parameter("steer_map_u", [-0.8, 0.0, 0.8])
        self.declare_parameter("throttle_map_v_mps", [-1.0, -0.001, 0.0, 0.001, 2.0])
        self.declare_parameter("throttle_map_u", [-0.5, -0.10, 0.0, 0.10, 0.5])

        vp_path = str(self.get_parameter("vehicle_params_file").value)
        if not vp_path:
            raise ValueError(
                "vehicle_params_file に車両諸元 YAML のパスを指定してください "
                "(minicar_bringup/config/vehicle_params.yaml)")
        with open(vp_path) as f:
            vp = yaml.safe_load(f)
        pwm_path = str(self.get_parameter("pwm_params_file").value)
        if not pwm_path:
            raise ValueError(
                "pwm_params_file に PWM 校正 JSON のパスを指定してください "
                "(FaBo の notebooks/01_find_pwm.ipynb が書き出すファイル)")
        with open(pwm_path) as f:
            pwm = json.load(f)
        motor_kw = {
            "steer_map": self._pairs("steer_map_delta_rad", "steer_map_u"),
            "throttle_map": self._pairs("throttle_map_v_mps", "throttle_map_u"),
            "v_eps": float(self.get_parameter("v_eps").value),
            "count_min": int(self.get_parameter("count_min").value),
            "count_max": int(self.get_parameter("count_max").value),
        }
        self.cfg = MotorConfig.from_yaml(vp, motor_kw, pwm)
        self.core = MotorDriverCore(self.cfg)

        self.reverse_mode = str(self.get_parameter("reverse_mode").value)
        if self.reverse_mode not in REVERSE_MODES:
            raise ValueError(
                f"reverse_mode は {REVERSE_MODES} のいずれか: {self.reverse_mode!r}")

        self.dt = 1.0 / float(self.get_parameter("rate").value)
        self.timeout = float(self.get_parameter("cmd_timeout").value)
        self.arm_duration = float(self.get_parameter("arm_neutral_duration").value)
        self.slew_max = float(self.get_parameter("steer_slew_rate_max").value)
        self.gap_duration = float(self.get_parameter("neutral_gap_duration").value)
        self.ch_steer = int(self.get_parameter("steering_channel").value)
        self.ch_throttle = int(self.get_parameter("throttle_channel").value)

        self.backend = make_backend(
            self.get_parameter("backend").value,
            bus=int(self.get_parameter("i2c_bus").value),
            address=int(self.get_parameter("i2c_address").value),
            freq_hz=float(self.get_parameter("pwm_freq_hz").value),
            # PCA9685 を開いた瞬間の出力。スロットルの中立にしておく。
            # 前進値で開くと、ノードの中立出力が始まる前に車が動き出す。
            initial_value=self.cfg.throttle_stop,
            logger=self.get_logger(),
        )

        self._cmd = None            # 直近に受けた (v, omega)
        self._cmd_time = None
        self._delta = 0.0           # 実際にサーボへ送っている舵角[rad]
        self._armed = False
        self._start_time = self.get_clock().now()
        self._rev_state = "idle"    # "idle" / "gap1" / "gap2" / "engaged"
        self._rev_until = None

        self.create_subscription(
            Twist, self.get_parameter("cmd_topic").value, self.on_cmd, 10)
        self.pub_delta = self.create_publisher(Float64, "/motor/steer_angle", 10)
        self.pub_u = self.create_publisher(
            Float64MultiArray, "/motor/cmd_normalized", 10)
        self.pub_count = self.create_publisher(
            Float64MultiArray, "/motor/pwm_count", 10)
        self.create_timer(self.dt, self.on_timer)

        c = self.cfg
        self.get_logger().info(
            f"motor_driver started. {self.get_parameter('cmd_topic').value} -> "
            f"backend={self.backend.name} (ch steer={self.ch_steer} "
            f"throttle={self.ch_throttle})  reverse_mode={self.reverse_mode}  "
            f"delta_max={c.delta_max:.3f}rad "
            f"v={c.v_min:+.1f}..{c.v_max:+.1f}m/s"
        )
        self.get_logger().info(
            f"PWM 校正 ({pwm_path}, {self.get_parameter('pwm_freq_hz').value}Hz): "
            f"steer {c.steer_left}/{c.steer_center}/{c.steer_right} "
            f"throttle {c.throttle_back}/{c.throttle_stop}/{c.throttle_front} "
            f"(left/center/right, back/stop/front)"
        )
        if self.backend.name == "dryrun":
            self.get_logger().warn(
                "backend=dryrun。PWM は出力していない。実機で走らせるには "
                "backend:=fabo_pca9685 を指定すること")
        self.get_logger().warn(
            "校正マップは未校正の暫定値。実測値へ差し替えるまでは車輪を浮かせるか、"
            "即時に電源を切れる状態で低速確認すること "
            "(minicar_motor/config の pwm_params.json と "
            "motor_driver_params.yaml の警告を参照)")

    # ------------------------------------------------------------------
    def _pairs(self, x_name: str, y_name: str) -> list:
        xs = list(self.get_parameter(x_name).value)
        ys = list(self.get_parameter(y_name).value)
        if len(xs) != len(ys):
            raise ValueError(
                f"{x_name} と {y_name} は同じ長さであること: {len(xs)} != {len(ys)}")
        return [[float(a), float(b)] for a, b in zip(xs, ys)]

    def on_cmd(self, msg: Twist) -> None:
        self._cmd = (msg.linear.x, msg.angular.z)
        self._cmd_time = self.get_clock().now()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def _slew(self, target: float) -> float:
        """舵角の変化率を制限する。サーボ機構の保護が目的。

        車両運動としての制限ではないので、既定値はサーボの実力と同程度に
        取ってある (制御を鈍らせない)。効いてほしいのは、NaN からの復帰や
        中立割り込みからの復帰で指令が瞬間的に飛ぶ場面。
        """
        step = self.slew_max * self.dt
        return float(np.clip(target, self._delta - step, self._delta + step))

    def _write(self, delta: float, u_t: float) -> None:
        """舵角とスロットルを PWM カウントとして書き、同じ値を debug へ流す。

        publish する値は「実際に書いた値」であること。変換の途中の値を
        publish すると、steer_debug との突き合わせで嘘をつく。
        """
        self._delta = delta
        u_s = self.core.steer_to_u(delta)
        c_s = self.core.to_count(u_s, "steer")
        c_t = self.core.to_count(u_t, "throttle")

        self.backend.set_count(self.ch_steer, c_s)
        self.backend.set_count(self.ch_throttle, c_t)

        self.pub_delta.publish(Float64(data=float(delta)))
        self.pub_u.publish(Float64MultiArray(data=[float(u_s), float(u_t)]))
        self.pub_count.publish(Float64MultiArray(data=[float(c_s), float(c_t)]))

    def neutral(self, center_steer: bool = True) -> None:
        """スロットルを中立にする。停止経路はすべてここを通る。

        中立を「出す」のであって「出すのを止める」のではない。サーボと ESC は
        最後のパルスをラッチし続けるので、publish を止めても車は走り続ける
        (safety_node.stop() と同じ理由。あちらは Twist、ここは PWM)。

        舵は既定でスルーレート制限つきで中央へ戻す。停止中に舵を切ったまま
        保持すると、サーボがタイヤの復元力に対してストールし続けて発熱する。
        後退シーケンスの中立ギャップだけは切り返しの舵を保持したいので
        center_steer=False で呼ぶ。
        """
        delta = self._slew(0.0) if center_steer else self._delta
        self._write(delta, 0.0)
        # 中立を挟んだので、次の後退要求はシーケンスをやり直す。
        self._rev_state = "idle"

    # ------------------------------------------------------------------
    def _throttle_u(self, v_cmd: float) -> float:
        """速度指令 -> 正規化スロットル。後退シーケンスをここで踏む。

        既定の "neutral_neutral_reverse" は **FaBo の実装をそのまま写したもの**。
        01_find_pwm.ipynb の set_back / check_back が後退へ入るとき

            set_channel_value(THROTTLE_CH, pwm_stop); time.sleep(0.1)
            set_channel_value(THROTTLE_CH, pwm_stop); time.sleep(0.1)
            set_channel_value(THROTTLE_CH, pwm_back)

        という順を踏んでいる。中立を 2 回挟んでから後退値を出す。
        TT-02 系の ESC は前進から直接後退へ行けず、一度中立を認識させる
        必要があるため。素通しすると**ブレーキがかかるだけで下がらず**、
        safety_node のスタック脱出 (recovery_speed を負にして出す) が
        無言で効かなくなる。ログにも異常が出ないので発見が遅れる。

        FaBo は「ブレーキとして逆方向信号を送る」段を持たない。以前ここに
        あった brake 段はこちらの推測だったので削除した。
        """
        if v_cmd >= -self.cfg.v_eps:
            # 前進または停止。次の後退要求に備えて状態を戻す。
            self._rev_state = "idle"
            return self.core.throttle_to_u(v_cmd)

        if self.reverse_mode == "disabled":
            self.get_logger().warn(
                f"後退指令 v={v_cmd:+.2f} を却下 (reverse_mode=disabled)",
                throttle_duration_sec=1.0)
            return self.core.throttle_to_u(0.0)

        if self.reverse_mode == "direct":
            return self.core.throttle_to_u(v_cmd)

        # --- neutral_neutral_reverse (FaBo 準拠) ---
        now = self._now_s()
        if self._rev_state == "idle":
            self._rev_state = "gap1"
            self._rev_until = now + self.gap_duration
            self.get_logger().info(
                f"後退シーケンス開始 (v={v_cmd:+.2f}): "
                f"中立 {self.gap_duration:.2f}s x2 -> 後退")

        if self._rev_state == "gap1":
            if now < self._rev_until:
                return 0.0
            self._rev_state = "gap2"
            self._rev_until = now + self.gap_duration

        if self._rev_state == "gap2":
            if now < self._rev_until:
                return 0.0
            self._rev_state = "engaged"
            self.get_logger().info("後退シーケンス完了。後退指令を通す")

        return self.core.throttle_to_u(v_cmd)

    # ------------------------------------------------------------------
    def on_timer(self) -> None:
        now = self.get_clock().now()

        # 1. アーミング。ESC が中立を検出できるまでスロットルを通さない。
        if not self._armed:
            if (now - self._start_time).nanoseconds * 1e-9 >= self.arm_duration:
                self._armed = True
                self.get_logger().info(
                    f"アーミング完了 ({self.arm_duration:.1f}s の中立出力)。指令を通す")
            else:
                self.neutral()
                return

        # 2. ウォッチドッグ。上流が落ちても、解が出なくても、ここで止まる。
        if self._cmd is None or self._cmd_time is None:
            self.neutral()
            return
        age = (now - self._cmd_time).nanoseconds * 1e-9
        if age > self.timeout:
            self.neutral()
            self.get_logger().warn(
                f"cmd stale ({age:.2f}s > {self.timeout}s)。中立を出力",
                throttle_duration_sec=1.0)
            return

        # 3. 変換。
        v, omega = self._cmd
        out = self.core.convert(v, omega, self._delta)
        if not out.valid:
            self.neutral()
            self.get_logger().error(
                f"/cmd_vel に有限でない値 (v={v} omega={omega})。中立を出力",
                throttle_duration_sec=1.0)
            return

        # 4. 出力。舵はスルーレート制限、スロットルは後退シーケンス経由。
        #    中立ギャップ中だけは舵を保持する (切り返しの舵を崩さない)。
        u_t = self._throttle_u(out.v_cmd)
        in_gap = self._rev_state in ("gap1", "gap2")
        delta = self._delta if in_gap else self._slew(out.delta_rad)
        self._write(delta, u_t)

        self.get_logger().info(
            f"v={out.v_cmd:+.2f} delta={np.degrees(delta):+6.2f}deg "
            f"(飽和 {abs(delta) / self.cfg.delta_max * 100:3.0f}%) "
            f"u=[{self.core.steer_to_u(delta):+.2f}, {u_t:+.2f}] "
            f"rev={self._rev_state}",
            throttle_duration_sec=1.0)


def main() -> None:
    rclpy.init()
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 落ちるときこそ中立を書く。ESC は最後のパルスを保持するので、
        # 書かずに終わると走ったまま誰も止められなくなる。
        try:
            node.neutral()
        except Exception:
            pass
        try:
            node.backend.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
