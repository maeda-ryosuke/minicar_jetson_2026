#!/usr/bin/env python3
"""stall_stop (停止検知の判定) の単体テスト。ROS も車両も不要。

    cd /ws && colcon test --packages-select minicar_safety       --event-handlers console_direct+ && colcon test-result --verbose
"""

import pytest

from minicar_safety.stall_stop import StallConfig, StallDetector

DT = 0.05  # 指令の周期 (MPPI 20Hz)


def run(det, v_cmd, v_odom, seconds, t0=0.0):
    """指令と /odom を同じ周期で流し、最後の判定と終了時刻を返す。"""
    t, out = t0, False
    while t < t0 + seconds - 1e-9:
        det.update_odom(v_odom, t)
        out = det.update(v_cmd, t)
        t += DT
    return out, t


def moving_detector():
    det = StallDetector(StallConfig())
    run(det, 0.5, 0.5, 0.5)
    assert det.armed
    return det


def test_trips_after_stall_time():
    det = moving_detector()
    out, t = run(det, 0.5, 0.0, 0.95, t0=1.0)
    assert not out
    out, _ = run(det, 0.5, 0.0, 0.1, t0=t)
    assert out


def test_latches_until_reset():
    det = moving_detector()
    run(det, 0.5, 0.0, 1.5, t0=1.0)
    assert det.tripped
    # 動き出しても指令をゼロにしても解除しない
    out, _ = run(det, 0.0, 0.5, 1.0, t0=3.0)
    assert out
    det.reset()
    assert not det.tripped and not det.armed
    assert not det.update(0.5, 5.0)


def test_not_armed_before_first_move():
    """マニュアルモードで待っている間 (指令あり・実速 0) は止めない。"""
    det = StallDetector(StallConfig())
    out, t = run(det, 0.5, 0.0, 10.0)
    assert not out and not det.armed
    # 走り出したら判定が始まる
    run(det, 0.5, 0.5, 0.5, t0=t)
    out, _ = run(det, 0.5, 0.0, 1.5, t0=t + 0.5)
    assert out


def test_slow_but_moving_is_not_stall():
    """芝生 (0.07 m/s まで落ちる) は止まっていない。"""
    det = moving_detector()
    out, _ = run(det, 0.5, 0.052, 5.0, t0=1.0)
    assert not out


def test_short_stop_resets_timer():
    det = moving_detector()
    t = 1.0
    for _ in range(5):
        out, t = run(det, 0.5, 0.0, 0.8, t0=t)
        assert not out
        _, t = run(det, 0.5, 0.3, 0.1, t0=t)


@pytest.mark.parametrize("v_cmd", [0.0, 0.05, -0.4])
def test_no_forward_command_no_trip(v_cmd):
    det = moving_detector()
    out, _ = run(det, v_cmd, 0.0, 5.0, t0=1.0)
    assert not out


def test_odom_stale_no_trip():
    """/odom 途絶中は判定しない (motor_driver の odom_lost_stop の担当)。"""
    det = moving_detector()
    det.update_odom(0.0, 1.0)
    t = 1.0
    while t < 5.0:
        assert not det.update(0.5, t)
        t += DT


def test_invalid_config():
    with pytest.raises(ValueError):
        StallDetector(StallConfig(arm_speed=0.02, speed_zero=0.03))
    with pytest.raises(ValueError):
        StallDetector(StallConfig(stall_time=0.0))
