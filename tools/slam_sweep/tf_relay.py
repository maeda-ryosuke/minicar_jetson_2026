#!/usr/bin/env python3
"""bag再生時に、bagの /tf から自己位置推定以外の変換だけを /tf へ中継する(コンテナ内で実行)。

    ros2 bag play <bag> --clock --remap /tf:=/tf_bag &
    python3 /tools/slam_sweep/tf_relay.py [--raceline /maps/raceline.csv] [--amcl-csv out.csv]

走行時に slam_toolbox が出した map->odom は /tf へ流さず、走行時の自己位置として
/slam_pose (PoseStamped, frame map) に出す。再生中の自己位置推定(AMCL など)の map->odom と
rviz2 で並べるため。あわせて次も出す。
  - /particle_cloud (nav2_msgs/ParticleCloud) を /particle_cloud_poses (PoseArray) に変換
    (ホストの rviz2 に nav2_rviz_plugins が無くても表示できるように)
  - 追従経路 CSV を /raceline_path (Path, frame map, latched)
  - --amcl-csv を指定したら /amcl_pose を csv に記録 (t, x, y, yaw, 共分散 xx, yy, yawyaw)
"""
import argparse
import csv
import math

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Relay(Node):
    def __init__(self, args):
        super().__init__('tf_relay', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 100)
        self.pub_slam = self.create_publisher(PoseStamped, '/slam_pose', 10)
        self.pub_pc = self.create_publisher(PoseArray, '/particle_cloud_poses', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_path = self.create_publisher(Path, '/raceline_path', latched)
        self.create_subscription(TFMessage, '/tf_bag', self.on_tf, 100)
        self.create_subscription(ParticleCloud, '/particle_cloud', self.on_pc, qos_profile_sensor_data)
        self.mo = None
        self.writer = None
        if args.amcl_csv:
            self.fp = open(args.amcl_csv, 'w', newline='')
            self.writer = csv.writer(self.fp)
            self.writer.writerow(['t', 'x', 'y', 'yaw', 'cov_xx', 'cov_yy', 'cov_yawyaw'])
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        if args.raceline:
            self.publish_path(args.raceline)

    def publish_path(self, fname):
        path = Path()
        path.header.frame_id = 'map'
        with open(fname) as fp:
            for row in csv.DictReader(fp):
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.pose.position.x, p.pose.position.y = float(row['x']), float(row['y'])
                yaw = float(row['yaw'])
                p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                path.poses.append(p)
        self.pub_path.publish(path)

    def on_tf(self, msg):
        out = TFMessage()
        for tr in msg.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                v = tr.transform.translation
                self.mo = (v.x, v.y, _yaw(tr.transform.rotation))
                continue
            out.transforms.append(tr)
            if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link' and self.mo:
                self.publish_slam(tr)
        if out.transforms:
            self.pub_tf.publish(out)

    def publish_slam(self, tr):
        """走行時の map->odom と bag の odom->base_link を合成した、走行時の自己位置。"""
        mx, my, mth = self.mo
        v = tr.transform.translation
        th = mth + _yaw(tr.transform.rotation)
        p = PoseStamped()
        p.header.stamp = tr.header.stamp
        p.header.frame_id = 'map'
        p.pose.position.x = mx + math.cos(mth) * v.x - math.sin(mth) * v.y
        p.pose.position.y = my + math.sin(mth) * v.x + math.cos(mth) * v.y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        self.pub_slam.publish(p)

    def on_pc(self, msg):
        out = PoseArray()
        out.header = msg.header
        out.poses = [Pose(position=p.pose.position, orientation=p.pose.orientation) for p in msg.particles]
        self.pub_pc.publish(out)

    def on_amcl(self, msg):
        p, c = msg.pose.pose, msg.pose.covariance
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.writer.writerow([f'{t:.3f}', p.position.x, p.position.y, _yaw(p.orientation), c[0], c[7], c[35]])
        self.fp.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--raceline', default='')
    ap.add_argument('--amcl-csv', default='')
    args = ap.parse_args()
    rclpy.init()
    node = Relay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
