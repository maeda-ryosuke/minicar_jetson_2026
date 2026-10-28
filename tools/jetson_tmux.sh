#!/usr/bin/env bash
# Jetson 実機の起動一式を tmux の 1 セッション (car) にまとめる。SSH は 1 回だけでよい。
#
#   ./tools/jetson_tmux.sh          起動 (既にあればアタッチし直す)
#   tmux attach -t car              SSH を張り直したあとに戻る
#   tmux kill-session -t car        全部止める
#
# ウィンドウ (画面下のタブ。クリックでも切替できる):
#   0 agent / 1 ekf / 2 loc / 3 nav2 / 4 stall … 自動で起動する
#   5 bag / 6 raceline / 7 motor          … コマンドを打った状態で止めてある。
#                                           準備ができたら そのウィンドウで Enter
#   8 sh                                  … 空のシェル (yaml の編集など)
#
# 操作: Ctrl-b 0〜8 で切替、Ctrl-b d で抜ける (中のプロセスは動き続ける)。
# ノードを起動し直すときは、そのウィンドウで Ctrl-C → ↑ → Enter。
#
# 注意: SSH が切れても車は走り続ける (tmux の中のプロセスは止まらない)。
# 止めるのはプロポ、または motor ウィンドウの Ctrl-C / tmux kill-session。
#
# sudo なしで docker を使えない場合は、ウィンドウごとに sudo のパスワードを聞かれる。
# 一度 `sudo usermod -aG docker $USER` して再ログインすると聞かれなくなる。
set -eu

SESSION=car
REPO=$(cd "$(dirname "$0")/.." && pwd)

if tmux has-session -t "$SESSION" 2>/dev/null; then
    exec tmux attach -t "$SESSION"
fi

if docker info >/dev/null 2>&1; then
    DC="docker compose"
else
    DC="sudo docker compose"
fi
X="$DC exec jetson bash -c"

# win <名前> <コマンド> <run|wait>
#   run  … Enter まで送って起動する
#   wait … コマンドを打つだけ。起動は人が Enter で行う
win() {
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux new-window -t "$SESSION" -n "$1" -c "$REPO"
    else
        tmux new-session -d -s "$SESSION" -n "$1" -c "$REPO"
        tmux set -t "$SESSION" mouse on
    fi
    if [ "$3" = run ]; then
        tmux send-keys -t "$SESSION:$1" "$2" Enter
    else
        tmux send-keys -t "$SESSION:$1" "$2"
    fi
}

win agent "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6" run
win ekf "ros2 launch kalmanfilter ekf.launch.py" run
win loc "sleep 3; $X \"ros2 launch minicar_bringup slam_localization_amcl.launch.py initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0\"" run
win nav2 "$X \"ros2 launch minicar_nav2 nav2_mppi.launch.py\"" run
# /cmd_vel_raw -> /cmd_vel。motor_driver はこれの出力を受けるので、無いと走らない。
# 詰まったら後退して自動で復帰する。復帰をあきらめて止まったら、
# このウィンドウのログに reset のコマンドが出る。
win stall "$X \"ros2 launch minicar_safety stall_stop.launch.py\"" run
# $(date ...) は Enter を押した時刻で展開させたいので、シングルクォートで \$ のまま送る。
win bag "$X"' "ros2 bag record -o /bags/\$(date +%Y%m%d%H%M)_run /cmd_vel_raw /cmd_vel /safety/stall_state /odom /odometry/filtered /motor/pwm_count /motor/cmd_normalized /motor/steer_angle /motor/speed_ctrl /rosout /amcl_pose /tf /tf_static"' wait
win raceline "$X \"ros2 launch minicar_raceline raceline.launch.py\"" wait
win motor "$X \"ros2 launch minicar_motor motor_driver.launch.py\"" wait
win sh "" wait

tmux select-window -t "$SESSION:agent"
exec tmux attach -t "$SESSION"
