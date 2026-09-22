#!/usr/bin/env python3
"""Race Line Manager (実機) — CSV読込 + Path変換 + FollowPath送信 + ラップ管理。

docs/LIDDER_MAPII.PNG の構成図の「Race Line Manager (自作ノード)」がこれ。

    raceline.csv -> [raceline_manager] --FollowPath--> controller_server(MPPI)
                                       --/raceline/lap--> (周回数)

Gazebo 側リポジトリの nodes/raceline_manager_node.py と**別物として独立に
持っている**。実機向けの違いは 3 点:

  * **map_origin_in_world を持たない。** CSV は最初から map 座標。
    シム側は gz の world -> map 静的TF のぶん原点を引くが、実機の地図は
    slam_toolbox が作る map そのもので world は存在しない。
  * **パラメータは config/raceline_params.yaml から読む。** 起動は
    ros2 run minicar_raceline raceline_manager_node。
  * **use_sim_time の既定は false。** launch が ParameterValue で注入する。

=== なぜ 1 周分を一度に投げないか ===

controller_server は経路の終点を goal_checker でゴール判定し、着いたら止まる。
周回路を 1 周分渡すと 1 周で止まり、始点と終点が重なっているので
走り出した直後に「到着」と判定されることもある。そこで現在位置から
前方 lookahead_m だけを切り出し、feed_rate で送り直す(新しいゴールが
来ると controller_server は経路を差し替える)。終点は常に前方にあるので
ゴール判定は発火しない。

前方長の条件: lookahead_m > prune_distance >= vx_max * time_steps * model_dt
(nav2 の MPPI 設定。vx_max 1.0 / 56 ステップ / 0.05s なら 4.0 > 3.0 >= 2.8)
**vx_max を上げたら lookahead_m と prune_distance も一緒に伸ばすこと。**
足りないと経路の終点がホライゾンの内側に入り、PathFollowCritic が
「終点で止まる」軌道を好むようになる。

=== 速度について: このノードは速度を決めない ===

**速度は全周 vx_max 固定**(nav2 の FollowPath.vx_max)。このノードは経路の
形だけを渡し、速度には一切触れない。raceline CSV に v_ref 列は無い
(理由は minicar_raceline/raceline_loop.py のコメント)。

帰結: **コーナーで自動的に減速しない。** vx_max は実走で決める。

=== ラップ管理 ===

**参照経路上の進捗で数える。** スタート線を跨いだかではなく、最近傍添字の
進み(LoopPath.step)を積算して周長に達したら 1 周とする。

こうする理由は 2 つ:
  * シムの真値(/ground_truth/odom)に頼らない。実機には存在しない。
    こちらは _locate() が引いている TF map->base_link をそのまま使う。
  * 後退脱出で戻ると進捗も戻る。LoopPath.step は符号つきなので、
    safety_node が後退した分は自動で引かれる。

=== 自己位置 ===

TF map -> base_link を引く。実機では slam_toolbox が map -> odom、
ホスト側の robot_localization(EKF) が odom -> base_link を出す。
このノードはどちらが出しているかを知らなくてよい。
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformException, TransformListener

from minicar_raceline.raceline_loop import LoopPath


class RaceLineManagerNode(Node):
    def __init__(self):
        super().__init__("raceline_manager")

        # 参照経路 CSV。既定を空にしてあるのは、パスを決め打ちにすると
        # 「別の経路を走らせているつもりで既定のファイルを走っていた」に
        # 気付けないため。launch か --ros-args で必ず渡す。
        self.declare_parameter("raceline_file", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("action_name", "follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")

        self.declare_parameter("lookahead_m", 4.0)
        self.declare_parameter("feed_rate", 2.0)            # [Hz] FollowPath の再送
        # 最近傍の探索窓[m]。全探索は蛇行コースで隣の通路へ吸着する。
        self.declare_parameter("nearest_window", 2.0)
        # 経路からこれ以上離れたら全探索でやり直す[m]。
        self.declare_parameter("relocalize_distance", 0.80)
        # ラップ管理。周長のこの割合まで進んだら次の通過を数え始める。
        # 1.0 ちょうどにすると、始点付近で前後に揺れたときに二重計上する。
        self.declare_parameter("lap_arm_fraction", 0.9)

        rl = str(self.get_parameter("raceline_file").value)
        if not rl:
            raise ValueError(
                "raceline_file に参照経路 CSV のパスを指定してください "
                "(列は x, y, yaw。座標系は map)")
        self.path = LoopPath.from_csv(rl)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.controller_id = str(self.get_parameter("controller_id").value)
        self.goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self.lookahead = float(self.get_parameter("lookahead_m").value)
        self.window = float(self.get_parameter("nearest_window").value)
        self.reloc = float(self.get_parameter("relocalize_distance").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(
            self, FollowPath, str(self.get_parameter("action_name").value))
        # 周回数。落としたくないが履歴も要らないので depth=1。
        self.pub_lap = self.create_publisher(Int32, "/raceline/lap", 1)

        self._idx = None
        self._sent = 0
        # ラップ管理の状態
        self.lap_arm = (float(self.get_parameter("lap_arm_fraction").value)
                        * self.path.length)
        self._progress = 0.0     # 今の周回に入ってからの経路上の進み[m]
        self._laps = 0
        self._lap_t0 = None

        feed_rate = float(self.get_parameter("feed_rate").value)
        self.create_timer(1.0 / feed_rate, self.on_feed)

        self.get_logger().info(
            f"raceline_manager started: {rl} "
            f"周長 {self.path.length:.2f}m ({self.path.n} 点) / "
            f"前方 {self.lookahead}m を {feed_rate}Hz で送る / "
            f"frame {self.map_frame}->{self.base_frame}")
        self.get_logger().warn(
            "速度は controller_server の vx_max 固定。このノードは関与しない。"
            "コーナーで自動減速しないので vx_max は実走で決めること")

    # ------------------------------------------------------------------
    def _locate(self):
        """現在位置の参照経路上の添字。TF がまだ無ければ None。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.0))
        except TransformException as e:
            self.get_logger().warn(
                f"TF {self.map_frame}->{self.base_frame} 待ち: {e}",
                throttle_duration_sec=2.0)
            return None
        # CSV は map 座標なので、TF の値をそのまま使う(原点の付け替えは無い)。
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        idx, d = self.path.nearest(x, y, self._idx, self.window)
        if self._idx is not None and d > self.reloc:
            idx, d = self.path.nearest(x, y, None)
            self.get_logger().warn(
                f"経路から {d:.2f}m 離れたので最近傍を全探索し直した "
                f"(s={self.path.s[idx]:.2f})")
        if self._idx is not None:
            self._advance(self._idx, idx)
        self._idx = idx
        return idx

    # ------------------------------------------------------------------
    def _advance(self, i_prev, i_new):
        """経路上の進みを積算し、周長に達したら 1 周として数える。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._lap_t0 is None:
            self._lap_t0 = now
        # 最近傍を全探索し直した直後は添字が飛ぶ。周長の半分を超える
        # 「進み」は跳びなので捨てる(LoopPath.step は +-n/2 に畳むので、
        # 跳びは必ず大きな値になる)。
        d = self.path.step(i_prev, i_new) * self.path.ds
        if abs(d) > 0.5 * self.path.length:
            return
        self._progress += d
        if self._progress >= self.lap_arm and self.path.step(i_prev, i_new) > 0:
            if self._progress >= self.path.length:
                self._laps += 1
                self.get_logger().error(      # 目立たせたいので error レベル
                    f"[LAP] {self._laps} 周目 完了  所要 {now - self._lap_t0:.1f}s")
                self.pub_lap.publish(Int32(data=self._laps))
                self._progress -= self.path.length
                self._lap_t0 = now

    def on_feed(self) -> None:
        idx = self._locate()
        if idx is None:
            return
        if not self.client.server_is_ready():
            self.get_logger().info(
                "controller_server の follow_path を待っている "
                "(nav2 の controller_server が起動しているか確認する)",
                throttle_duration_sec=2.0)
            return

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for j in self.path.ahead(idx, self.lookahead):
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(self.path.x[j])
            p.pose.position.y = float(self.path.y[j])
            yaw = float(self.path.yaw[j])
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(p)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        # 結果は待たない。前のゴールは次の送信で差し替わる。
        self.client.send_goal_async(goal)
        self._sent += 1
        if self._sent == 1:
            self.get_logger().info(
                f"最初の FollowPath を送信 "
                f"(s={self.path.s[idx]:.2f}m, {len(path.poses)} 点)")


def main() -> None:
    rclpy.init()
    node = RaceLineManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy は SIGTERM を ExternalShutdownException に変えて投げてくるので、
        # 拾わないと終了のたびに traceback が出る。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
