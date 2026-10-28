#!/usr/bin/env python3
"""stall_stop (停止検知と後退復帰の判定) の単体テスト。ROS も車両も不要。

    cd /ws && colcon test --packages-select minicar_safety       --event-handlers console_direct+ && colcon test-result --verbose
"""

import pytest

from minicar_safety.stall_stop import PASS, RECOVER, STOP, StallConfig, StallDetector

DT = 0.05  # 指令の周期 (MPPI 20Hz)


def run(det, v_cmd, v_odom, seconds, t0=0.0, omega=0.0):
    """指令と /odom を同じ周期で流し、最後の出力と終了時刻を返す。"""
    t, out = t0, None
    while t < t0 + seconds - 1e-9:
        det.update_odom(v_odom, t)
        out = det.update(v_cmd, omega, t)
        t += DT
    return out, t


def moving_detector():
    det = StallDetector(StallConfig())
    run(det, 0.5, 0.5, 0.5)
    assert det.armed
    return det


def test_recovers_after_stall_time():
    det = moving_detector()
    out, t = run(det, 0.5, 0.0, 0.95, t0=1.0)
    assert out is None
    out, _ = run(det, 0.5, 0.0, 0.1, t0=t)
    assert det.mode == RECOVER
    assert out[0] == pytest.approx(-0.4)


def test_recovery_steers_opposite():
    """前進で左 (omega > 0) へ曲がろうとして詰まったら、後退は右へ切る (kappa < 0)。"""
    det = moving_detector()
    out, _ = run(det, 0.5, 0.0, 1.1, t0=1.0, omega=0.5)
    assert det.recover_kappa == pytest.approx(-1.0)
    v, omega = out
    assert v < 0 and omega / v == pytest.approx(-1.0)


def test_returns_to_pass_after_duration():
    det = moving_detector()
    _, t = run(det, 0.5, 0.0, 1.1, t0=1.0)
    assert det.mode == RECOVER
    # 後退中は指令を無視して後退を出し続ける
    out, t = run(det, 0.5, -0.4, 1.4, t0=t)
    assert det.mode == RECOVER and out[0] < 0
    out, _ = run(det, 0.5, 0.0, 0.2, t0=t)
    assert det.mode == PASS and out is None


def test_moving_forward_resets_count():
    det = moving_detector()
    t = 1.0
    for _ in range(5):
        _, t = run(det, 0.5, 0.0, 1.1, t0=t)       # 詰まる
        _, t = run(det, 0.5, -0.4, 1.5, t0=t)      # 後退
        _, t = run(det, 0.5, 0.5, 0.5, t0=t)       # 抜けて前進できた
        assert det.mode == PASS and det.recoveries == 0


def test_stops_after_max_recoveries():
    """後退しても前進できないまま続けば、あきらめて停止 (ラッチ)。"""
    det = moving_detector()
    t = 1.0
    for _ in range(3):
        _, t = run(det, 0.5, 0.0, 1.1, t0=t)
        assert det.mode == RECOVER
        _, t = run(det, 0.5, 0.0, 1.5, t0=t)       # 後退も効かない
    out, t = run(det, 0.5, 0.0, 1.1, t0=t)
    assert det.mode == STOP and out == (0.0, 0.0)
    # 動き出しても指令をゼロにしても解除しない
    out, _ = run(det, 0.0, 0.5, 1.0, t0=t)
    assert out == (0.0, 0.0)
    det.reset()
    assert det.mode == PASS and not det.armed and det.recoveries == 0
    assert det.update(0.5, 0.0, t + 2.0) is None


def test_not_armed_before_first_move():
    """マニュアルモードで待っている間 (指令あり・実速 0) は何もしない。"""
    det = StallDetector(StallConfig())
    out, t = run(det, 0.5, 0.0, 10.0)
    assert out is None and not det.armed
    # 走り出したら判定が始まる
    run(det, 0.5, 0.5, 0.5, t0=t)
    run(det, 0.5, 0.0, 1.5, t0=t + 0.5)
    assert det.mode == RECOVER


def test_slow_but_moving_is_not_stall():
    """芝生 (0.07 m/s まで落ちる) は止まっていない。"""
    det = moving_detector()
    out, _ = run(det, 0.5, 0.052, 5.0, t0=1.0)
    assert out is None and det.mode == PASS


def test_short_stop_resets_timer():
    det = moving_detector()
    t = 1.0
    for _ in range(5):
        out, t = run(det, 0.5, 0.0, 0.8, t0=t)
        assert out is None
        _, t = run(det, 0.5, 0.3, 0.1, t0=t)


@pytest.mark.parametrize("v_cmd", [0.0, 0.05, -0.4])
def test_no_forward_command_no_trip(v_cmd):
    det = moving_detector()
    out, _ = run(det, v_cmd, 0.0, 5.0, t0=1.0)
    assert out is None and det.mode == PASS


def test_odom_stale_no_trip():
    """/odom 途絶中は判定しない (motor_driver の odom_lost_stop の担当)。"""
    det = moving_detector()
    det.update_odom(0.0, 1.0)
    t = 1.0
    while t < 5.0:
        assert det.update(0.5, 0.0, t) is None
        t += DT


def test_invalid_config():
    with pytest.raises(ValueError):
        StallDetector(StallConfig(arm_speed=0.02, speed_zero=0.03))
    with pytest.raises(ValueError):
        StallDetector(StallConfig(stall_time=0.0))
    with pytest.raises(ValueError):
        StallDetector(StallConfig(recovery_duration=0.0))
