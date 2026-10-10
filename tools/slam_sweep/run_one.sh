#!/bin/bash
# replayコンテナ内で1条件を実行する: run_one.sh <outdir> <bag>
# 呼び出し側で ROS_DOMAIN_ID を条件ごとに変えて並列実行する。
set -u
OUT=$1; BAG=$2
cd "$OUT"
setsid ros2 launch minicar_bringup slam_mapping.launch.py use_sim_time:=true \
  scan_params_file:="$OUT/scan.yaml" slam_params_file:="$OUT/slam.yaml" > slam.log 2>&1 &
LAUNCH=$!
sleep 5
python3 /tools/slam_sweep/monitor.py "$OUT/tf.csv" > monitor.log 2>&1 &
MON=$!
ros2 bag play "$BAG" --clock --rate 1.0 \
  --topics /scan /tf /tf_static /odom /imu /odometry/filtered > play.log 2>&1
sleep 3
ros2 run nav2_map_server map_saver_cli -f "$OUT/map" --ros-args -p use_sim_time:=true > saver.log 2>&1
# localization用のpose graph(map.posegraph / map.data)も残す。
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$OUT/map'}" > serialize.log 2>&1
kill -INT $MON 2>/dev/null
kill -INT -- -$LAUNCH 2>/dev/null
sleep 3
kill -9 -- -$LAUNCH 2>/dev/null
wait 2>/dev/null
test -f "$OUT/map.pgm" && echo OK || echo FAIL
