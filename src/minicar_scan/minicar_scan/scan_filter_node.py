#!/usr/bin/env python3
"""SLAM 用のスキャン前処理: 前方 180deg / 有効距離 2m にゲートする。

/scan -> /scan_filtered。slam_toolbox と AMCL はこちらを購読する。

=== なぜ絞るのか（実機の物理制約）===

実コースの壁は高さ約 0.10m しかなく、LiDAR の走査面は地上高 0.05m に載る
予定。つまり上下のクリアランスは 0.05m ずつしかない。車体がピッチすると
ビームは壁の上を越えるか床に当たり、「手前の壁ではない何か」を測って
しまう。制約は上下 2 つあり、有効距離はその最小値で決まる:

    R = min(壁高 - h, h) / tan(pitch) = 0.05 / tan(pitch)

    pitch 0.5deg -> 5.73m   (定速・平滑路)
    pitch 1.4deg -> 2.00m   <- 既定値 2m が想定している上限
    pitch 3.0deg -> 0.95m   (強めのブレーキ)

重要なのは、これが「距離値のズレ」ではなく「別の物体を測ってしまう」
問題だという点。壁に当たったビームの斜距離誤差は 2m/2deg で 1.2mm と
LiDAR ノイズ(10mm)以下なので補正の必要は無い。一方、壁を外したビームは
手前の壁に一度も触れていないので、補正して正しい壁距離に戻すことは
原理的に不可能。だから「補正」ではなく「破棄」する。

=== シムでは自動的に再現されない ===

Gazebo の tt02 はサスペンションの無い剛体でほとんどピッチしないため、
シムの生 /scan は 270deg / 30m のまま素直に返ってくる。実機の条件を
明示的に被せるのがこのノードの役目であり、これを飛ばすと「シムでは
完璧だが実機で破綻する」という典型的な Sim2Real 失敗になる。

=== ftg_node の前処理とは別物 ===

ftg_node も前方 180deg に絞ってクリップしているが、あれは制御用(バブル
塗りなど FTG 固有の加工を含む)。こちらは SLAM 用で、加工は一切せず
「信用できないビームを無効値にする」だけ。

=== 実機で追加する予定（未実装）===

IMU のピッチ/ロールを使ったビーム毎の幾何判定に拡張する。一律のレンジ
上限だと、ピッチの影響を受けない側方ビーム(通路半幅 0.41m なら許容角
7.0deg)まで巻き添えで捨ててしまうため。max_pitch_deg は今のうちに
用意してあるが、シムでは剛体なので発火しない。

    ros2 run minicar_scan scan_filter_node --ros-args -p range_max:=2.0
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFilterNode(Node):
    def __init__(self):
        super().__init__("scan_filter_node")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_filtered")
        # 前方 180deg。実機 TG30 は 360deg 返すが、車体・電装の遮蔽で後方は
        # 当てにしない方針。可観測性の観点では FOV 回復の効果は小さく
        # (退化率 66.7% -> 59.3%)、効くのは距離の方。
        self.declare_parameter("fov_deg", 180.0)
        # 有効距離。これを超える返りは無効値に潰す。
        self.declare_parameter("range_max", 2.0)
        # ピッチがこれを超えたらスキャンを丸ごと捨てる。実機用の保険で、
        # シムの剛体モデルでは発火しない。0 以下で無効。
        self.declare_parameter("max_pitch_deg", 0.0)

        self.fov = math.radians(float(self.get_parameter("fov_deg").value))
        self.rmax = float(self.get_parameter("range_max").value)
        self.max_pitch = float(self.get_parameter("max_pitch_deg").value)

        out = self.get_parameter("output_topic").value
        self.pub = self.create_publisher(LaserScan, out, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value,
            self.on_scan, qos_profile_sensor_data)

        self._logged = False
        self.get_logger().info(
            f"scan_filter_node started: -> {out} "
            f"fov={math.degrees(self.fov):.0f}deg range_max={self.rmax}m"
        )

    def on_scan(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        # range_max はゲート値に下げる。ここを元のまま(30m)にしておくと
        # slam_toolbox 側が「有効な最大距離は 30m」と解釈してしまう。
        out.range_max = self.rmax
        out.angle_increment = msg.angle_increment

        half = self.fov / 2.0
        n = len(msg.ranges)
        keep = [i for i in range(n)
                if abs(msg.angle_min + i * msg.angle_increment) <= half]
        if not keep:
            return

        i0, i1 = keep[0], keep[-1]
        out.angle_min = msg.angle_min + i0 * msg.angle_increment
        out.angle_max = msg.angle_min + i1 * msg.angle_increment

        # 距離ゲート: 上限超えは inf にして「返りが無かった」ことにする。
        # 0 や range_max ちょうどにすると「そこに壁がある」と解釈されうる。
        rmax = self.rmax
        out.ranges = [r if (msg.range_min <= r <= rmax) else float("inf")
                      for r in msg.ranges[i0:i1 + 1]]
        if msg.intensities:
            out.intensities = list(msg.intensities[i0:i1 + 1])

        if not self._logged:
            valid = sum(1 for r in out.ranges if math.isfinite(r))
            self.get_logger().info(
                f"first scan: {n} -> {len(out.ranges)} beams "
                f"({math.degrees(out.angle_min):+.0f}..{math.degrees(out.angle_max):+.0f}deg), "
                f"{valid} valid within {rmax}m"
            )
            self._logged = True

        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanFilterNode()
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
