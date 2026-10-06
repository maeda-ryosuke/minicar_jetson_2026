#!/bin/bash
# replayコンテナ内でlocalizationを1条件実行する: run_loc.sh <outdir> <bag> <posegraph>
set -u
OUT=$1; BAG=$2; GRAPH=$3
cd "$OUT"
setsid ros2 launch minicar_bringup slam_localization.launch.py use_sim_time:=true \
  posegraph_file:="$GRAPH" scan_params_file:="$OUT/scan.yaml" slam_params_file:="$OUT/slam.yaml" \
  > slam.log 2>&1 &
LAUNCH=$!
sleep 8
python3 /tools/slam_sweep/monitor.py "$OUT/tf.csv" 0.2 > monitor.log 2>&1 &
MON=$!
ros2 bag play "$BAG" --clock --rate 1.0 \
  --topics /scan /tf /tf_static /odom /imu /odometry/filtered > play.log 2>&1
sleep 2
kill -INT $MON 2>/dev/null
kill -INT -- -$LAUNCH 2>/dev/null
sleep 3
kill -9 -- -$LAUNCH 2>/dev/null
wait 2>/dev/null
test -s "$OUT/tf.csv" && echo OK || echo FAIL
