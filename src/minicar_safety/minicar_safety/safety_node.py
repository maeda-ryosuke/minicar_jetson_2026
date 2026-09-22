#!/usr/bin/env python3
"""安全フィルタ — 指令チェーンの最終段。

    [mppi_node] --/cmd_vel_raw--> [safety_node] --/cmd_vel--> 実機走行系

制御ノードの出力を素通しし、異常時だけ自分の指令に差し替える。責務は3つ:

  1. スタック検知と後退による脱出
  2. ウォッチドッグ (指令が途切れたら停止)
  3. 加速度制限 (アクチュエータ直前でかける)

なぜ制御ノードから分離したか:

  - 制御方式に依存しない。pursuit を MPPI に差し替えても、この安全機構は
    そのまま生き残る。制御ノードの中に置くと差し替えのたびに作り直しになる。
  - 制御の変更で異常検知を壊せない。実際、FTG のバブル飽和を直した際に
    衝突検知が副作用で消え、空転したまま odom が距離を数え続けて
    「55m 走行(真値は 6.45m)」と誤報告する状態を作ってしまった。
  - 制御ノードが落ちても止まる。停止指令を出す主体が制御ノード自身だと、
    それが死んだ瞬間に誰も止められなくなる。実機走行系が指令をラッチする
    場合でも、独立したウォッチドッグが停止指令を出し続ける。

publisher を1つに保つのが要点。安全ノードを別に立てて両方が /cmd_vel へ
publish すると、同じ周期同士で殴り合ってどちらが勝つか不定になる。
必ず「チェーンの途中に挟むフィルタ」にすること。
(Nav2 の nav2_collision_monitor と同じ形)

    ros2 run minicar_safety safety_node --ros-args -p vehicle_params_file:=/absolute/path/vehicle_params.yaml
"""

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_node")

        self.declare_parameter("rate", 20.0)
        self.declare_parameter("cmd_timeout", 0.3)
        self.declare_parameter("stuck_cmd_threshold", 0.15)
        self.declare_parameter("stuck_imu_threshold", 0.05)
        self.declare_parameter("stuck_time", 1.0)
        self.declare_parameter("stuck_v_threshold", 0.15)
        self.declare_parameter("stuck_scan_threshold", 0.02)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("recovery_speed", 0.4)
        self.declare_parameter("recovery_duration", 1.5)
        self.declare_parameter("vehicle_params_file", "")
        self.declare_parameter("in_topic", "/cmd_vel_raw")
        self.declare_parameter("out_topic", "/cmd_vel")
        self.declare_parameter("imu_topic", "/imu/data")

        vehicle_params_file = self.get_parameter("vehicle_params_file").value
        if not vehicle_params_file:
            raise ValueError("vehicle_params_file に車両諸元 YAML のパスを指定してください")
        with open(vehicle_params_file) as f:
            vp = yaml.safe_load(f)
        self.a_max = float(vp["limits"]["a_max"])
        self.kappa_max = 1.0 / float(vp["limits"]["r_min"])

        self.dt = 1.0 / float(self.get_parameter("rate").value)
        self.timeout = float(self.get_parameter("cmd_timeout").value)

        self._cmd = None          # 直近に受けた (v, omega)
        self._cmd_time = None
        self._imu_wz = 0.0
        self._v_prev = 0.0
        self._stuck_since = None
        self._stuck_why = ""
        self._prev_scan = None    # 前回スキャン。動いているかの実測に使う
        self._scan_delta = None   # 連続スキャンの平均変化量[m]
        self._mode = "pass"       # "pass" / "recover"
        self._recover_until = None
        self._recover_kappa = 0.0

        self.create_subscription(
            Twist, self.get_parameter("in_topic").value, self.on_cmd, 10)
        self.create_subscription(
            Imu, self.get_parameter("imu_topic").value,
            lambda m: setattr(self, "_imu_wz", m.angular_velocity.z),
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value,
            self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            Twist, self.get_parameter("out_topic").value, 10)
        self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"safety_node started. {self.get_parameter('in_topic').value} -> "
            f"{self.get_parameter('out_topic').value}  a_max={self.a_max}"
        )

    def on_cmd(self, msg: Twist) -> None:
        self._cmd = (msg.linear.x, msg.angular.z)
        self._cmd_time = self.get_clock().now()

    def _publish(self, v: float, omega: float) -> None:
        """加速度制限をかけて出す。最終段でかけるのが正しい。

        制御ノード側でかけると、こちらが指令を上書きしている間に
        あちらの内部状態が実際とずれ、素通しに戻った瞬間に速度が飛ぶ。
        """
        dv = self.a_max * self.dt
        v = float(np.clip(v, self._v_prev - dv, self._v_prev + dv))
        self._v_prev = v
        out = Twist()
        out.linear.x = v
        out.angular.z = omega
        self.pub.publish(out)

    def stop(self) -> None:
        """ゼロ指令。AckermannSteering は最後の指令を保持し続けるので、
        止めたいときは明示的に 0 を送らないと走り続ける。"""
        self._v_prev = 0.0
        self.pub.publish(Twist())

    # ------------------------------------------------------------------
    def _is_stuck(self, cmd_v: float, cmd_wz: float) -> bool:
        """壁に刺さって動けていないかを判定する。判定経路は2つ。

        (1) 旋回: 指令 omega に対し IMU の実測角速度が出ていない
        (2) 前進: 指令 v が出ているのにスキャンが変化していない

        /odom は使えない。車輪回転と操舵角から積分した推測航法なので、
        壁に押し付けられて空転していても「指令どおり」を返し続ける
        (実測: odom が 0.49rad/s のとき IMU は 0.000)。

        (2) を足したのは、(1) だけでは「ほぼ直進のまま詰まった場合」を
        見逃すため。MPPI 版で実際に起きた: 壁に密着して omega が
        0.02〜0.14 と閾値 0.15 未満に収まり、90秒間まったく検知されず
        後退脱出も走らなかった。

        前進の実測に使えるセンサが無いのが厄介なところ。IMU の加速度は
        等速走行と静止を区別できず、odom は嘘をつく。そこでスキャン自体を
        使う。動いていれば連続スキャンの距離が変わる。実機でも同じ手が使える。
        """
        thr_cmd = float(self.get_parameter("stuck_cmd_threshold").value)
        thr_act = float(self.get_parameter("stuck_imu_threshold").value)
        thr_v = float(self.get_parameter("stuck_v_threshold").value)
        thr_scan = float(self.get_parameter("stuck_scan_threshold").value)
        hold = float(self.get_parameter("stuck_time").value)

        turning_but_still = abs(cmd_wz) > thr_cmd and abs(self._imu_wz) < thr_act
        driving_but_still = (abs(cmd_v) > thr_v and self._scan_delta is not None
                             and self._scan_delta < thr_scan)

        now = self.get_clock().now()
        if turning_but_still or driving_but_still:
            if self._stuck_since is None:
                self._stuck_since = now
                self._stuck_why = "旋回不能" if turning_but_still else "前進不能"
                return False
            return (now - self._stuck_since).nanoseconds * 1e-9 > hold
        self._stuck_since = None
        return False

    def on_scan(self, msg: LaserScan) -> None:
        """連続スキャンの変化量を測る。「本当に動いているか」の実測値。

        静止していれば差分はセンサノイズ相当(sigma=0.01m なら平均 0.011m
        程度)にしかならない。0.3m/s で動いていれば 10Hz スキャン間で 3cm
        進むので、距離の変化はその数倍のオーダーになる。
        """
        r = np.asarray(msg.ranges, dtype=np.float64)
        r = np.where(np.isfinite(r), r, msg.range_max)
        if self._prev_scan is not None and self._prev_scan.size == r.size:
            self._scan_delta = float(np.abs(r - self._prev_scan).mean())
        self._prev_scan = r

    def _enter_recovery(self, v: float, omega: float) -> None:
        """後退による脱出を開始する。

        壁に密着した車が前進しながら曲がるのは幾何的に不可能なので、
        制御ノードが「右へ 72度」と言い続けても永久に抜けられない。
        切り返し(3点ターン)と同じ要領で、後退時は舵を逆に切る。

        自転車モデルでは omega = v * kappa なので、v<0 のまま同じ向きに
        回頭を続けるには kappa の符号を反転させる必要がある。前進で右へ
        曲がろうとして詰まったなら、後退では左に切る。
        素通ししている指令から kappa = omega/v を復元できるので、
        制御ノードから内部状態を貰う必要はない。
        """
        kappa = omega / v if abs(v) > 1e-3 else 0.0
        kappa = float(np.clip(kappa, -self.kappa_max, self.kappa_max))
        dur = float(self.get_parameter("recovery_duration").value)

        self._mode = "recover"
        self._recover_until = self.get_clock().now() + Duration(seconds=dur)
        self._recover_kappa = -kappa
        self._stuck_since = None
        self._v_prev = 0.0
        sd = "n/a" if self._scan_delta is None else f"{self._scan_delta:.4f}"
        self.get_logger().error(
            f"STUCK 検知 [{self._stuck_why}] (指令 v={v:+.2f} omega={omega:+.3f} / "
            f"IMU {self._imu_wz:+.3f} スキャン変化 {sd})。"
            f"{dur:.1f}秒 後退して脱出する (kappa {kappa:+.2f} -> {self._recover_kappa:+.2f})"
        )

    def _do_recovery(self) -> None:
        """後退指令を出す。復帰時刻を過ぎたら素通しへ戻る。

        注意: 現在の LiDAR は 270deg なので真後ろ(±135deg 以遠)が見えず、
        後退の安全確認ができない。そのため時間と速度で保守的に打ち切って
        いる(既定 0.4m/s x 1.5s = 最大 0.6m)。実機 TG30 は 360deg あるので
        実機では後方も見て判定できる。ここはシムの方が条件が悪い。
        """
        if self.get_clock().now() >= self._recover_until:
            self._mode = "pass"
            self._v_prev = 0.0
            self.get_logger().info("後退による脱出を終了、素通しへ復帰")
            self.stop()
            return

        v = -abs(float(self.get_parameter("recovery_speed").value))
        self._publish(v, v * self._recover_kappa)
        self.get_logger().info(
            f"後退中 v={v:+.2f} omega={v * self._recover_kappa:+.3f} "
            f"(IMU 実測 {self._imu_wz:+.3f})",
            throttle_duration_sec=0.5,
        )

    # ------------------------------------------------------------------
    def on_timer(self) -> None:
        if self._mode == "recover":
            # 脱出中は制御ノードの指示を無視する。壁に密着した状態で
            # 出される「右へ 72度」は幾何的に実行できないため。
            self._do_recovery()
            return

        # ウォッチドッグ。制御ノードが落ちても、解が出なくても、ここで止まる。
        if self._cmd is None or self._cmd_time is None:
            self.stop()
            return
        age = (self.get_clock().now() - self._cmd_time).nanoseconds * 1e-9
        if age > self.timeout:
            self.stop()
            self.get_logger().warn(
                f"cmd stale ({age:.2f}s > {self.timeout}s). STOP",
                throttle_duration_sec=1.0,
            )
            return

        v, omega = self._cmd
        self._publish(v, omega)
        if self._is_stuck(v, omega):
            self._enter_recovery(v, omega)


def main() -> None:
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
