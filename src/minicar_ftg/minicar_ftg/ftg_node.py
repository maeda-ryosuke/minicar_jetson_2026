#!/usr/bin/env python3
"""FTG (Follow The Gap) — ステップ 2-0 / 2-1。

現時点では gap 検出も操舵もしない。/scan を受けて

  2-0: 初回スキャンの生データ性質を診断ログに出す
       （当たらないビームが inf か range_max か、angle_increment の分母が
         N か N-1 か、NaN が出るか。ここを推測すると後段が全部狂う）
  2-1: 前方コーン抽出 → 無効値置換 → クリップ → 平滑化

まで行い、結果を /ftg/scan_filtered に LaserScan として出す。
rviz2 で /scan と重ねて見れば前処理の妥当性を目視確認できる。

パッケージの実行コマンド（車両諸元を明示する）:
    ros2 run minicar_ftg ftg_node --ros-args -p vehicle_params_file:=/absolute/path/vehicle_params.yaml
"""

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.ndimage import uniform_filter1d
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class FtgNode(Node):
    def __init__(self):
        super().__init__("ftg_node")

        # --- パラメータ（config/ftg_params.yaml から注入。実行中に
        #     ros2 param set でも変えられるようにしてある） ---
        self.declare_parameter("cone_half_angle_deg", 90.0)
        self.declare_parameter("range_clip", 5.0)
        self.declare_parameter("smooth_window", 5)
        self.declare_parameter("bubble_radius", 0.20)
        self.declare_parameter("best_point_tolerance", 0.05)
        self.declare_parameter("hysteresis_gain", 60.0)
        self.declare_parameter("forward_bias_gain", 100.0)
        self.declare_parameter("lookahead_distance", 1.5)
        self.declare_parameter("lookahead_min", 0.3)
        self.declare_parameter("vehicle_params_file", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("filtered_topic", "/ftg/scan_filtered")
        self.declare_parameter("target_topic", "/ftg/target_point")
        self.declare_parameter("marker_topic", "/ftg/markers")

        # 車両諸元は ROS パラメータではなく plain YAML から読む。
        # 物理量であって走行中に触るものではないため、チューニング用の
        # ftg_params.yaml とは意図的に系統を分けている。
        vp_path = self.get_parameter("vehicle_params_file").value
        if not vp_path:
            raise ValueError("vehicle_params_file に車両諸元 YAML のパスを指定してください")
        with open(vp_path) as f:
            vp = yaml.safe_load(f)
        # bicycle model の原点(後輪車軸)から見た LiDAR の前方オフセット。
        self.lidar_dx = float(vp["vehicle"]["lidar_x_from_rear_axle"])
        self.frame_rear_axle = "rear_axle"
        self.get_logger().info(
            f"loaded {vp_path}: lidar_x_from_rear_axle={self.lidar_dx}"
        )

        scan_topic = self.get_parameter("scan_topic").value
        filtered_topic = self.get_parameter("filtered_topic").value

        # gz ブリッジ側の QoS が reliable でも best_effort でも受けられるよう
        # 購読は sensor_data(best_effort) 固定にしておく。逆向きは繋がらない。
        self.sub = self.create_subscription(
            LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(LaserScan, filtered_topic, 10)
        self.pub_target = self.create_publisher(
            PointStamped, self.get_parameter("target_topic").value, 10
        )
        self.pub_marker = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, 10
        )

        self._diagnosed = False   # 2-0 の診断は初回のみ
        self._cone_cache = None   # (angle_min, angle_increment, n) -> (i0, i1)
        self._last_target = None  # (x, y)。gap 無しなら None
        self._prev_angle = None   # 直前に選んだ狙点の方位[rad]。振動抑制用

        self.get_logger().info(f"ftg_node started. waiting for {scan_topic} ...")

    # ------------------------------------------------------------------
    # 2-0: 生データ診断（初回のみ）
    # ------------------------------------------------------------------
    def _diagnose(self, msg: LaserScan) -> None:
        r = np.asarray(msg.ranges, dtype=np.float64)
        n = r.size
        span = msg.angle_max - msg.angle_min

        n_pinf = int(np.isposinf(r).sum())
        n_ninf = int(np.isneginf(r).sum())
        n_nan = int(np.isnan(r).sum())
        finite = r[np.isfinite(r)]

        # 「当たらないビーム」の表現方法を特定する。inf なのか range_max
        # ちょうどなのか range_max 超えなのかで置換ロジックが変わる。
        n_at_max = int(np.isclose(finite, msg.range_max).sum())
        n_over_max = int((finite > msg.range_max).sum())
        n_under_min = int((finite < msg.range_min).sum())

        # angle_increment の分母が N か N-1 かを実測値と突き合わせる。
        inc_by_n = span / n if n else float("nan")
        inc_by_n1 = span / (n - 1) if n > 1 else float("nan")

        lines = [
            "===== 2-0 scan diagnostics (first message) =====",
            f"  frame_id        : {msg.header.frame_id}",
            f"  n_beams         : {n}",
            f"  angle_min/max   : {msg.angle_min:.6f} / {msg.angle_max:.6f} rad"
            f"  ({np.degrees(msg.angle_min):.2f} / {np.degrees(msg.angle_max):.2f} deg)",
            f"  angle_increment : {msg.angle_increment:.8f} rad"
            f"  ({np.degrees(msg.angle_increment):.4f} deg)",
            f"    span/N        = {inc_by_n:.8f}  <- {'MATCH' if np.isclose(msg.angle_increment, inc_by_n) else 'no'}",
            f"    span/(N-1)    = {inc_by_n1:.8f}  <- {'MATCH' if np.isclose(msg.angle_increment, inc_by_n1) else 'no'}",
            f"  range_min/max   : {msg.range_min} / {msg.range_max}",
            f"  scan_time       : {msg.scan_time}   time_increment: {msg.time_increment}",
            f"  intensities len : {len(msg.intensities)}",
            "  --- invalid value representation ---",
            f"  +inf            : {n_pinf}",
            f"  -inf            : {n_ninf}",
            f"  NaN             : {n_nan}",
            f"  == range_max    : {n_at_max}",
            f"  >  range_max    : {n_over_max}",
            f"  <  range_min    : {n_under_min}",
        ]
        if finite.size:
            lines += [
                f"  finite count    : {finite.size}",
                f"  finite min/max  : {finite.min():.4f} / {finite.max():.4f}",
            ]
        else:
            lines.append("  finite count    : 0  (!!! 全ビームが無効値)")
        lines.append("================================================")
        self.get_logger().info("\n".join(lines))

    # ------------------------------------------------------------------
    # 2-1: 前処理
    # ------------------------------------------------------------------
    def _cone_indices(self, msg: LaserScan) -> tuple:
        """前方コーンに対応するビーム区間 [i0, i1] を返す。

        FOV が 270° あるため、制限せずに最長 gap を探すと後方(±90°〜±135°)
        の巨大な空きが必ず勝ち、車が後ろを向く。ここで先に切っておく。
        """
        key = (msg.angle_min, msg.angle_increment, len(msg.ranges))
        if self._cone_cache and self._cone_cache[0] == key:
            return self._cone_cache[1]

        half = np.radians(self.get_parameter("cone_half_angle_deg").value)
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        inside = np.flatnonzero(np.abs(angles) <= half)
        if inside.size == 0:  # コーン角が狭すぎる等の異常設定
            idx = (0, n - 1)
        else:
            idx = (int(inside[0]), int(inside[-1]))

        self._cone_cache = (key, idx)
        return idx

    def _preprocess(self, msg: LaserScan):
        clip = float(self.get_parameter("range_clip").value)
        win = int(self.get_parameter("smooth_window").value)

        i0, i1 = self._cone_indices(msg)
        r = np.asarray(msg.ranges[i0 : i1 + 1], dtype=np.float64)

        # +inf = 「その方向には clip 距離まで障害物が無い」= 自由空間。
        # NaN  = センサ異常。自由と解釈すると壁へ突っ込むので 0(=最も近い
        #        障害物)として保守側に倒す。gz では通常出ないはずで、
        #        出ていれば診断ログの NaN カウントで気づける。
        r = np.where(np.isposinf(r), clip, r)
        r = np.where(np.isnan(r), 0.0, r)
        r = np.where(np.isneginf(r), 0.0, r)

        # クリップは平滑化より先に行う。順序を逆にすると 30m の遠方値が
        # 移動平均で近傍ビームを引き上げ、壁が消えたように見える。
        np.clip(r, 0.0, clip, out=r)

        if win > 1:
            # 端を 'nearest' で埋める。convolve の 'same' は端で
            # 実質的に 0 埋めになり、コーン端の壁を消してしまう。
            r = uniform_filter1d(r, size=win, mode="nearest")

        return r, i0, i1

    # ------------------------------------------------------------------
    # 2-2: gap 検出（Safety Bubble 方式）
    # ------------------------------------------------------------------
    @staticmethod
    def _runs(mask: np.ndarray):
        """True が連続する全区間を [(lo, hi), ...] で返す（両端含む）。

        端に 0 を挟んでから差分を取る定石。ループを回さずに済む。
        """
        if not mask.any():
            return []
        edges = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
        starts = np.flatnonzero(edges == 1)
        ends = np.flatnonzero(edges == -1)  # 終端は排他的
        return [(int(s), int(e - 1)) for s, e in zip(starts, ends)]

    @classmethod
    def _longest_run(cls, mask: np.ndarray):
        runs = cls._runs(mask)
        if not runs:
            return None
        return max(runs, key=lambda t: t[1] - t[0])

    def _find_gap(self, r: np.ndarray, angle_increment: float, cone_angle_min: float):
        """戻り値 (free, gap_lo, gap_hi, i_best) / gap 無しなら None。

        free は バブルを 0 で塗った配列（rviz にそのまま出して可視化する）。
        """
        bubble = float(self.get_parameter("bubble_radius").value)
        tol = float(self.get_parameter("best_point_tolerance").value)

        # ① 最近傍点。「一番危ない障害物」1つだけに注目するのがこの方式。
        i_min = int(np.argmin(r))
        d_min = float(r[i_min])

        # ② 安全バブル。半径 bubble[m] の円に重なるビームを 0 にする。
        #    片側の角度半幅 = asin(bubble / d_min) なので、近いほど広く遮る。
        #
        #    d_min <= bubble（障害物がバブル半径より近い）で asin が定義域外に
        #    なるが、ここで「gap なし」にすると破綻する。車幅 0.15m の車が
        #    幅 0.6m 級の通路を正常に抜けるだけで壁まで 0.2m を切るため、
        #    走行中に何度も全停止してしまう。定義域外は「その点の周り ±90deg を
        #    遮る」に飽和させ、反対側へ逃げる余地を残す。
        #    本当の緊急（逃げ場ゼロ）は ③ で gap が見つからないことで判定する。
        #    d_min == 0 は前処理で NaN / -inf を 0 に写した結果か、壁に接触
        #    している場合に出る。素直に割ると ZeroDivisionError でノードごと
        #    落ちるので、遮蔽を最大(±90deg)に振り切らせる。
        ratio = 1.0 if d_min <= 1e-6 else bubble / d_min
        half_rad = float(np.arcsin(np.clip(ratio, 0.0, 1.0)))
        n_half = int(np.ceil(half_rad / angle_increment))
        lo = max(0, i_min - n_half)
        hi = min(r.size - 1, i_min + n_half)

        free = r.copy()
        free[lo : hi + 1] = 0.0

        # ③ 通行可能な最長の連続区間 = gap。幅が広いほど余裕がある。
        run = self._longest_run(free > 0.0)
        if run is None:
            return None
        gap_lo, gap_hi = run

        # ④ gap 内の狙点。単純な argmax は使えない。range_clip で頭打ちに
        #    なったビームが大量に同値になり、argmax は同値の先頭を返すため
        #    狙点が常に gap の片側へ寄る（直線で片側に寄っていく症状）。
        #    そこで「最大値の tol 以内のビーム群」の中央を採る。
        seg = free[gap_lo : gap_hi + 1]
        near_max = seg >= seg.max() * (1.0 - tol)
        # 候補が複数の開口部に割れている場合、候補全体の中央は谷間=壁を
        # 指してしまうので、連続した塊(クラスタ)ごとに評価して1つ選ぶ。
        clusters = self._runs(near_max)
        if not clusters:
            return None

        # 幅だけで選ぶと、左右にほぼ同幅の開口があるとき毎スキャン勝者が
        # 入れ替わり、目標が +75deg / -75deg と交互に振れる（実測済み）。
        # 舵が左右に振られて正味の旋回が進まず、そのまま壁へ直進する。
        # 直前の選択から離れた候補ほど減点し、選択を粘らせて振動を止める。
        gain = float(self.get_parameter("hysteresis_gain").value)
        fwd = float(self.get_parameter("forward_bias_gain").value)
        best_i, best_score = None, -np.inf
        for c_lo, c_hi in clusters:
            c_mid = gap_lo + (c_lo + c_hi) // 2
            ang = cone_angle_min + c_mid * angle_increment
            score = float(c_hi - c_lo + 1)  # 幅[beam]
            if gain > 0.0 and self._prev_angle is not None:
                score -= gain * abs(ang - self._prev_angle)
            # 前進バイアス。真正面から離れた方向ほど減点する。
            # これが無いと、大きな方位角を出し続けた結果 Pure Pursuit が
            # 曲率上限で飽和し、車が最小旋回半径の円を延々と回り続ける
            # （実測: 45秒で右に4.86周、左には5degしか回っていない）。
            # 二乗にしてあるのは、緩いコーナーは安く・反転級の大角度だけ
            # 高くするため。線形だと普通のコーナーまで曲がりにくくなる。
            if fwd > 0.0:
                score -= fwd * ang * ang
            if score > best_score:
                best_score, best_i = score, c_mid

        return free, gap_lo, gap_hi, best_i

    # ------------------------------------------------------------------
    # 2-3: 狙点 → 車両座標の目標点
    # ------------------------------------------------------------------
    def _target_point(self, angle: float, dist: float) -> tuple:
        """LiDAR 極座標の狙点を、後輪車軸原点の目標点 (x, y, bearing, L) に直す。

        原点をずらすと方位角も変わる点に注意。LiDAR は後輪車軸より
        0.2285m 前にあるので、真横に近い狙点ほど後輪車軸から見た方位は
        小さくなる。Pure Pursuit の幾何は後輪車軸基準なので、ここで
        直しておかないと舵角が系統的に過大になる。
        """
        ld = float(self.get_parameter("lookahead_distance").value)
        ld_min = float(self.get_parameter("lookahead_min").value)

        # LiDAR 座標 -> 後輪車軸座標（前方 dx ぶん平行移動するだけ）
        px = dist * np.cos(angle) + self.lidar_dx
        py = dist * np.sin(angle)

        bearing = float(np.arctan2(py, px))
        reach = float(np.hypot(px, py))

        # 前方注視距離。壁までの距離を超えて置くと目標点が壁の中に入るので
        # 実際に開けている距離で頭打ちにする。
        L = float(np.clip(min(reach, ld), ld_min, ld))
        return L * np.cos(bearing), L * np.sin(bearing), bearing, L

    def _publish_target(self, header, x: float, y: float, bearing: float = 0.0,
                        closest: float = 0.0) -> None:
        pt = PointStamped()
        pt.header.stamp = header.stamp
        pt.header.frame_id = self.frame_rear_axle
        pt.point.x, pt.point.y, pt.point.z = x, y, 0.0
        self.pub_target.publish(pt)

        ma = MarkerArray()
        sphere = Marker()
        sphere.header = pt.header
        sphere.ns, sphere.id = "ftg", 0
        sphere.type, sphere.action = Marker.SPHERE, Marker.ADD
        sphere.pose.position = pt.point
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12
        sphere.color.g, sphere.color.a = 1.0, 1.0
        ma.markers.append(sphere)

        line = Marker()
        line.header = pt.header
        line.ns, line.id = "ftg", 1
        line.type, line.action = Marker.LINE_STRIP, Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.02
        line.color.g, line.color.a = 1.0, 0.8
        origin = PointStamped().point  # (0,0,0) = 後輪車軸
        line.points = [origin, pt.point]
        ma.markers.append(line)

        # 目標角度を数値で出す。マーカーの向きだけだと何度なのか読めない。
        text = Marker()
        text.header = pt.header
        text.ns, text.id = "ftg", 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = pt.point.x
        text.pose.position.y = pt.point.y
        text.pose.position.z = 0.25
        text.pose.orientation.w = 1.0
        text.scale.z = 0.15          # 文字高さ[m]
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f"{np.degrees(bearing):+.1f}deg  closest {closest:.2f}m"
        ma.markers.append(text)

        self.pub_marker.publish(ma)

    # ------------------------------------------------------------------
    def on_scan(self, msg: LaserScan) -> None:
        if not self._diagnosed:
            self._diagnose(msg)
            self._diagnosed = True

        proc, i0, i1 = self._preprocess(msg)
        cone_angle_min = msg.angle_min + i0 * msg.angle_increment

        result = self._find_gap(proc, msg.angle_increment, cone_angle_min)

        if result is None:
            # gap 無し。ここで無理に方向をひねり出すと壁に向かって走り続ける。
            # 明示的に「解なし」とし、2-5 の制御側で停止に落とす。
            self._last_target = None
            self._prev_angle = None   # ヒステリシスの基準もリセット
            shown = proc
            self.get_logger().warn(
                f"no gap: 全方位が遮蔽 (closest={proc.min():.2f}m). EMERGENCY",
                throttle_duration_sec=1.0,
            )
        else:
            shown, gap_lo, gap_hi, i_best = result
            angle = cone_angle_min + i_best * msg.angle_increment
            x, y, bearing, L = self._target_point(angle, float(proc[i_best]))
            self._last_target = (x, y)
            self._prev_angle = angle
            self._publish_target(msg.header, x, y, bearing, float(proc.min()))
            self.get_logger().info(
                f"gap[{gap_lo}:{gap_hi}] w={gap_hi - gap_lo + 1}beam "
                f"beam={np.degrees(angle):+6.2f}deg -> "
                f"bearing={np.degrees(bearing):+6.2f}deg L={L:.2f}m "
                f"pt=({x:+.2f},{y:+.2f})  closest={proc.min():.2f}m",
                throttle_duration_sec=1.0,
            )

        # バブルを 0 で塗った配列をそのまま出す。rviz2 で /ftg/scan_filtered を
        # 見れば「どこを遮蔽してどこを gap と判定したか」が可視化される。
        out = LaserScan()
        out.header = msg.header
        out.angle_min = cone_angle_min
        out.angle_max = msg.angle_min + i1 * msg.angle_increment
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = float(self.get_parameter("range_clip").value)
        out.ranges = shown.astype(np.float32).tolist()
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = FtgNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C / SIGTERM。rclpy は SIGTERM を ExternalShutdownException に
        # 変えて投げてくるので、拾わないと終了のたびに traceback が出る。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
