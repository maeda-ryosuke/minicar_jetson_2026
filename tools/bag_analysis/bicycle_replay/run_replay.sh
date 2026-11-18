#!/usr/bin/env bash
# 実走 bag を開ループで再生し、my_mppi_controller と speed_zone_publisher の出力を記録する
# (replay コンテナ内で実行)。車は bag のとおりに動き、MPPI の出力には従わない。
#
#   bash /tools/bag_analysis/bicycle_replay/run_replay.sh /bags/20261105_fast4 [raceline.csv]
#
# イメージの /ws/install は古くなる(raceline の lookahead が 4 m のままだった)ので、
# 使うパッケージは毎回 /src(リポジトリの src)から /tmp/replay_ws にビルドし直す。
# 出力は <bag>/analysis/bicycle/ の replay_bag(記録)と各ノードのログ。
set -e

BAG=$1
RACELINE=${2:-/maps/raceline_center_gate_low2.csv}
OUT=${3:-$BAG/analysis/bicycle}
ZONES=${ZONES:-$(dirname "$0")/zones_fast4.yaml}

source /ws/install/setup.bash
(cd /tmp && colcon build --base-paths /src --build-base /tmp/replay_ws/build \
  --install-base /tmp/replay_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select minicar_raceline my_mppi_controller speed_zone_publisher minicar_bringup)
source /tmp/replay_ws/install/setup.bash
mkdir -p "$OUT"
rm -rf "$OUT/replay_bag"

pids=()
start() {  # start <ログ名> <コマンド...>
  local log=$1; shift
  setsid "$@" > "$OUT/$log.log" 2>&1 &
  pids+=($!)
}

start mppi ros2 launch my_mppi_controller my_mppi.launch.py use_sim_time:=true
start zone ros2 launch speed_zone_publisher speed_zone_publisher.launch.py \
  use_sim_time:=true zones_yaml:="$ZONES"
start raceline ros2 launch minicar_raceline raceline.launch.py \
  use_sim_time:=true raceline_file:="$RACELINE"
start record ros2 bag record --use-sim-time -o "$OUT/replay_bag" \
  /target_speed /speed_zone_publisher/markers /trajectories /transformed_global_plan /cmd_vel_raw
sleep 8

# 自己位置(map->odom と odom->base_link)とセンサだけを流す。MPPI の出力系は流さない。
ros2 bag play "$BAG" --clock 100 --topics /tf /tf_static /scan /odometry/filtered /odom
sleep 2

for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null || true; done
wait || true
# コンテナは root で書くので、ホストから扱えるよう bag と同じ所有者にする
chown -R --reference="$BAG" "$BAG/analysis"
echo "記録: $OUT/replay_bag"
