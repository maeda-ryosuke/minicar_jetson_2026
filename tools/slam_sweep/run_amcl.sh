#!/bin/bash
# replayコンテナ内で自己位置推定を1条件実行する:
#   run_amcl.sh <outdir> <bag> <engine: amcl|slam> <map> <x> <y> <yaw> [<record_dir>]
# <map>: amcl は地図yaml、slam は posegraph(拡張子なし)。条件のパラメータは <outdir>/{amcl,slam,scan}.yaml。
# bagの /tf は /tf_bag へ remap し、tf_relay.py が走行時の map->odom を除いて /tf へ中継する
# (走行時の slam_toolbox の map->odom が入ったbagでも、再生中の推定と衝突しないように)。
# <record_dir> を渡すと、rviz2 で見るための結果bagを記録する。
set -u
OUT=$1; BAG=$2; ENGINE=$3; MAP=$4; X=$5; Y=$6; YAW=$7; REC=${8:-}
cd "$OUT"
if [ "$ENGINE" = amcl ]; then
  setsid ros2 launch minicar_bringup slam_localization_amcl.launch.py use_sim_time:=true \
    map_yaml:="$MAP" scan_params_file:="$OUT/scan.yaml" amcl_params_file:="$OUT/amcl.yaml" \
    initial_x:="$X" initial_y:="$Y" initial_yaw:="$YAW" > loc.log 2>&1 &
else
  setsid ros2 launch minicar_bringup slam_localization.launch.py use_sim_time:=true \
    posegraph_file:="$MAP" scan_params_file:="$OUT/scan.yaml" slam_params_file:="$OUT/slam.yaml" \
    > loc.log 2>&1 &
fi
LAUNCH=$!
setsid python3 /tools/slam_sweep/tf_relay.py --raceline /maps/raceline.csv --amcl-csv "$OUT/amcl_pose.csv" \
  > relay.log 2>&1 &
RELAY=$!
sleep 8
python3 /tools/slam_sweep/monitor.py "$OUT/tf.csv" 0.05 > monitor.log 2>&1 &
MON=$!
if [ -n "$REC" ]; then
  rm -rf "$REC"
  setsid ros2 bag record -o "$REC" /clock /tf /tf_static /scan_filtered /map /amcl_pose \
    /particle_cloud_poses /slam_pose /raceline_path /odometry/filtered > record.log 2>&1 &
  RECP=$!
  sleep 2
fi
ros2 bag play "$BAG" --clock --rate 1.0 --remap /tf:=/tf_bag \
  --topics /scan /tf /tf_static /odom /imu /odometry/filtered > play.log 2>&1
sleep 2
[ -n "$REC" ] && kill -INT -- -$RECP 2>/dev/null
kill -INT $MON 2>/dev/null
kill -INT -- -$RELAY 2>/dev/null
kill -INT -- -$LAUNCH 2>/dev/null
sleep 3
kill -9 -- -$LAUNCH -$RELAY 2>/dev/null
wait 2>/dev/null
test -s "$OUT/tf.csv" && echo OK || echo FAIL
