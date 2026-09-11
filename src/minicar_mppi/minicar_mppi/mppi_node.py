#!/usr/bin/env python3
"""MPPI の ROS グルー。pursuit_node を置き換える。

    [ftg_node] --/ftg/target_point--> [mppi_node] --/cmd_vel_raw--> [safety_node]
        ^                                 ^   ^
      /scan                            /scan  /odom

最適化本体は mppi_core.py（ROS 非依存）。このファイルは購読・変換・publish
だけを持つ。safety_node とのインタフェースは Twist 一本なので、
ウォッチドッグ・スタック検知・後退脱出・加速度制限はそのまま効く。

重要: 障害物には /ftg/scan_filtered ではなく生の /scan を使う。
あちらは前方180度に絞り安全バブルを 0 で塗った FTG 専用の加工済みデータで、
ロールアウトは横にも伸びるので捨てられた点も障害物として要る。

    ros2 run minicar_mppi mppi_node --ros-args -p vehicle_params_file:=/absolute/path/vehicle_params.yaml
"""

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PointStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from minicar_mppi.mppi_core import MPPI, MPPIConfig


class MppiNode(Node):
    def __init__(self):
        super().__init__("mppi_node")

        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("target_timeout", 0.5)
        self.declare_parameter("vehicle_params_file", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("target_topic", "/ftg/target_point")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("viz_samples", 30)
        self.declare_parameter("publish_esdf", True)
        # コスト内訳の記録先。空文字で無効。ログは 1秒スロットルが掛かっていて
        # 全周期ぶんが残らないので、後から掃引するにはファイルが要る。
        self.declare_parameter("cost_log_file", "/tmp/mppi_cost.csv")

        # MPPIConfig のフィールドは全て ROS パラメータで上書きできるようにする
        # (走らせながら ros2 param set で詰められるように)。
        base = MPPIConfig()
        self._tunable = [
            "K", "T", "dt", "lam", "sigma_a", "sigma_delta",
            "grid_x_min", "grid_x_max", "grid_y_half", "grid_res", "grid_dilate",
            "w_collision", "w_obs", "d_safe", "w_goal", "w_speed", "v_target",
            "w_smooth", "w_heading", "heading_danger_distance",
            "heading_time_power", "steer_tau",
        ]
        for k in self._tunable:
            self.declare_parameter(k, getattr(base, k))

        vehicle_params_file = self.get_parameter("vehicle_params_file").value
        if not vehicle_params_file:
            raise ValueError("vehicle_params_file に車両諸元 YAML のパスを指定してください")
        with open(vehicle_params_file) as f:
            vp = yaml.safe_load(f)
        mppi_kw = {k: self.get_parameter(k).value for k in self._tunable}
        self.cfg = MPPIConfig.from_yaml(vp, mppi_kw)
        self.mppi = MPPI(self.cfg)

        # コスト内訳の CSV。ヘッダは項名が確定する初回書き込み時に出す。
        path = str(self.get_parameter("cost_log_file").value)
        self._cost_log = None
        if path:
            # buffering=1 (行バッファ) にして、Ctrl-C で落ちても直前まで残す。
            self._cost_log = open(path, "w", buffering=1)
            self._cost_log_header = False
            self.get_logger().info(f"cost 内訳を {path} に記録する")

        self.rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / self.rate
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.target_timeout = float(self.get_parameter("target_timeout").value)

        self._scan = None
        self._scan_time = None
        self._angles = None       # スキャンの諸元は初回に確定させる
        self._target = None
        self._target_time = None
        self._v_meas = 0.0
        self._delta_est = 0.0     # 現在の舵角の推定値(実測できないので自前で持つ)
        self._frame = "rear_axle"

        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value,
            self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            PointStamped, self.get_parameter("target_topic").value,
            self.on_target, 10)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10)

        self.pub = self.create_publisher(
            Twist, self.get_parameter("cmd_topic").value, 10)
        self.pub_marker = self.create_publisher(MarkerArray, "/mppi/markers", 10)
        self.pub_grid = self.create_publisher(OccupancyGrid, "/mppi/esdf", 1)
        self.create_timer(self.dt, self.on_timer)

        c = self.cfg
        self.get_logger().info(
            f"mppi_node started. K={c.K} T={c.T} dt={c.dt} "
            f"(horizon {c.T * c.dt:.2f}s) lam={c.lam} "
            f"half_width={c.half_width} tau={c.steer_tau}"
        )

    # ------------------------------------------------------------------
    def on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if self._angles is None or self._angles.size != n:
            self._angles = msg.angle_min + np.arange(n) * msg.angle_increment
            self._r_min, self._r_max = msg.range_min, msg.range_max
        self._scan = np.asarray(msg.ranges, dtype=np.float64)
        self._scan_time = self.get_clock().now()

    def on_target(self, msg: PointStamped) -> None:
        # ftg_node は後輪車軸(tt02/rear_axle)座標で出しているので変換不要。
        self._target = (msg.point.x, msg.point.y)
        self._target_time = self.get_clock().now()

    def on_odom(self, msg: Odometry) -> None:
        # 速度だけ使う。位置と姿勢は使わない(反応制御なので不要だし、
        # この odom は車輪回転からの推測航法で信用できない)。
        # 速度も空転時は嘘をつくが、ロールアウトの初速としては最善の入手可能値。
        self._v_meas = float(msg.twist.twist.linear.x)

    def _age(self, t) -> float:
        if t is None:
            return float("inf")
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def on_timer(self) -> None:
        # スキャンが無い/古いときは publish を止める。safety_node の
        # ウォッチドッグが車を停止させる。ここで自前で止めないのは、
        # 停止の責任を安全ノード1箇所に集約しておくため。
        if self._scan is None or self._age(self._scan_time) > self.scan_timeout:
            self.get_logger().warn("scan が無い/古い。指令を停止",
                                   throttle_duration_sec=1.0)
            return

        px, py = self.mppi.esdf.scan_to_points(
            self._scan, self._angles, self.cfg.lidar_dx, self._r_min, self._r_max)
        self.mppi.esdf.build(px, py)

        goal = None
        if self._target is not None and self._age(self._target_time) <= self.target_timeout:
            goal = self._target

        v0 = float(np.clip(self._v_meas, self.cfg.v_min, self.cfg.v_max))
        out = self.mppi.step(v0, self._delta_est, goal)

        # 出すのは指令そのもの(論文 Alg.1 の SendToActuators(u_0))。
        # 遅れ適用後の値を出すと、gz の AckermannSteering が持つ同じ時定数の
        # 一次遅れと二重に掛かって、ロールアウトの前提と実車がずれる。
        v, delta_cmd = out["v"], out["delta_cmd"]
        if not (np.isfinite(v) and np.isfinite(delta_cmd)):
            self.get_logger().error("MPPI の解が NaN。指令を停止",
                                    throttle_duration_sec=1.0)
            self.mppi.reset()
            return
        # 次周期の初期舵角には、実車が追従した先の推定値(遅れ適用後)を渡す。
        self._delta_est = out["delta"]

        cmd = Twist()
        cmd.linear.x = v
        # gz の AckermannSteering は delta = atan(omega*L/v) で舵角を作るので、
        # omega = v*tan(delta)/L を渡せば辻褄が合う。publish する舵角と
        # ここの delta は必ず同じものを使うこと。
        cmd.angular.z = v * np.tan(delta_cmd) / self.cfg.wheelbase
        self.pub.publish(cmd)

        self._publish_viz(out, goal)
        # 飽和率を出す。「MPPI が最大舵角を要求しているのか」を指令側から
        # 判定できるようにするため。100% に達しないならコスト設計の問題で、
        # 達しているのに車が曲がらないならシム側かタイヤ側の問題。
        sat = abs(delta_cmd) / self.cfg.delta_max * 100.0
        self.get_logger().info(
            f"v={v:.2f} delta={np.degrees(delta_cmd):+6.2f}deg "
            f"(δmax {np.degrees(self.cfg.delta_max):.2f}, 飽和 {sat:3.0f}%) "
            f"omega={cmd.angular.z:+.3f} cost_min={out['cost_min']:.0f} "
            f"衝突サンプル {out['collision_frac']*100:.0f}% "
            f"goal={'有' if goal else '無'}",
            throttle_duration_sec=1.0,
        )
        # コスト内訳。sd(サンプル間の標準偏差)が小さい項は、値が大きくても
        # 最適化を動かしていない。平均だけ見ると効いていると誤読する。
        parts = out["cost_parts"]
        self.get_logger().info(
            "cost 内訳 [平均/sd/占有] " + "  ".join(
                f"{k} {m:.0f}/{sd:.0f}/{p:.0f}%" for k, (m, sd, p) in parts.items()
            ) + f"   ESS {out['ess']:.0f}/{self.cfg.K}",
            throttle_duration_sec=1.0,
        )
        self._write_cost_log(out, v, delta_cmd, goal)

    # ------------------------------------------------------------------
    def _write_cost_log(self, out: dict, v: float, delta_cmd: float, goal) -> None:
        """コスト内訳を CSV に 1行足す。ログと違いスロットルせず全周期出す。"""
        if self._cost_log is None:
            return
        parts = out["cost_parts"]
        if not self._cost_log_header:
            cols = ["t", "v", "delta_cmd", "cost_min", "cost_mean", "ess",
                    "collision_frac", "goal"]
            for k in parts:
                cols += [f"{k}_mean", f"{k}_sd", f"{k}_pct"]
            self._cost_log.write(",".join(cols) + "\n")
            self._cost_log_header = True

        t = self.get_clock().now().nanoseconds * 1e-9
        row = [f"{t:.3f}", f"{v:.4f}", f"{delta_cmd:.5f}",
               f"{out['cost_min']:.1f}", f"{out['cost_mean']:.1f}",
               f"{out['ess']:.1f}", f"{out['collision_frac']:.4f}",
               "1" if goal else "0"]
        for m, sd, p in parts.values():
            row += [f"{m:.1f}", f"{sd:.1f}", f"{p:.1f}"]
        self._cost_log.write(",".join(row) + "\n")

    def destroy_node(self) -> bool:
        if self._cost_log is not None:
            self._cost_log.close()
            self._cost_log = None
        return super().destroy_node()

    # ------------------------------------------------------------------
    def _publish_viz(self, out: dict, goal) -> None:
        ma = MarkerArray()

        # サンプル軌道。1本ずつ Marker にすると数が多すぎるので、
        # 1つの LINE_LIST にまとめて線分として詰める。
        n_viz = int(self.get_parameter("viz_samples").value)
        if n_viz > 0:
            sx, sy = out["samples"]
            idx = np.linspace(0, sx.shape[0] - 1, min(n_viz, sx.shape[0])).astype(int)
            m = Marker()
            m.header.frame_id = self._frame
            m.ns, m.id = "mppi", 0
            m.type, m.action = Marker.LINE_LIST, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.004
            m.color.r = m.color.g = m.color.b = 0.55
            m.color.a = 0.35
            pts = []
            for k in idx:
                xs, ys = sx[k], sy[k]
                for t in range(len(xs) - 1):
                    pts.append(Point(x=float(xs[t]), y=float(ys[t])))
                    pts.append(Point(x=float(xs[t + 1]), y=float(ys[t + 1])))
            m.points = pts
            ma.markers.append(m)

        # 採用軌道
        tx, ty = out["traj"]
        m = Marker()
        m.header.frame_id = self._frame
        m.ns, m.id = "mppi", 1
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03
        m.color.r, m.color.g, m.color.a = 1.0, 0.55, 1.0
        m.points = [Point(x=float(x), y=float(y)) for x, y in zip(tx, ty)]
        ma.markers.append(m)

        self.pub_marker.publish(ma)

        if bool(self.get_parameter("publish_esdf").value):
            self._publish_grid()

    def _publish_grid(self) -> None:
        """距離場を OccupancyGrid で出す。壁に近いほど値が大きい。"""
        e = self.mppi.esdf
        g = OccupancyGrid()
        g.header.frame_id = self._frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = e.res
        g.info.width, g.info.height = e.nx, e.ny
        g.info.origin.position.x = e.x0
        g.info.origin.position.y = e.y0
        g.info.origin.orientation.w = 1.0
        # 0..2m を 100..0 に写す。近いほど濃く見える。
        val = np.clip(1.0 - e.dist / 2.0, 0.0, 1.0) * 100.0
        # OccupancyGrid は row-major (index = iy*width + ix)、
        # dist は [ix, iy] なので転置してから flatten する。
        g.data = val.T.astype(np.int8).ravel().tolist()
        self.pub_grid.publish(g)


def main() -> None:
    rclpy.init()
    node = MppiNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
