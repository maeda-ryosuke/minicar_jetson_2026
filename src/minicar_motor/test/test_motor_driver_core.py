#!/usr/bin/env python3
"""motor_driver_core / pwm_backend の単体テスト。ROS も I2C も車両も不要。

    cd /ws && colcon test --packages-select minicar_motor       --event-handlers console_direct+ && colcon test-result --verbose

車両諸元は minicar_bringup、校正値はこのパッケージの config から読む。
読めなければ既定値で回す。既定値だけで組むと、YAML を変えてもテストが
追従せず「誰も走らせていない設定を検証している」状態になる。

ここで確定させたいのは 1 点に尽きる ──
**実機に通電する前に、指令が想定外の値にならないことを保証する。**
サーボの機械端への突っ込みと ESC への異常値は、実機でしか起きないのに
実機で試すのが一番高くつく。
"""

import math
import os
import sys

import numpy as np

from minicar_motor.motor_driver_core import (
    MotorCommand, MotorConfig, MotorDriverCore, SpeedController, as_table,
    piecewise_linear,
)
from minicar_motor.pwm_backend import DryRunBackend, make_backend

# パッケージ内の config。colcon test はソースツリーで走るので、
# share ディレクトリではなくここを見る。
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(PKG, "config")


def _vehicle_params_path() -> str:
    """minicar_bringup の車両諸元 YAML。

    install 済みなら share から、そうでなければ colcon ワークスペースの
    ソースツリーから探す。**ここに値を写さない** ── 車両諸元の出所は
    minicar_bringup 1 箇所だけ、という実機リポジトリの約束を崩さないため。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("minicar_bringup"),
                            "config", "vehicle_params.yaml")
    except Exception:
        return os.path.join(os.path.dirname(PKG), "minicar_bringup",
                            "config", "vehicle_params.yaml")

FAILURES: list[str] = []


def check(condition: bool, message: str) -> None:
    if not condition:
        FAILURES.append(message)


def load_cfg() -> MotorConfig:
    """YAML があれば実設定で、無ければ既定値で組む。"""
    vp_path = _vehicle_params_path()
    md_path = os.path.join(CONFIG, "motor_driver_params.yaml")
    if not os.path.exists(vp_path):
        print(f"  {vp_path} が無いので既定値で検証する")
        return MotorConfig()

    import yaml
    with open(vp_path) as f:
        vp = yaml.safe_load(f)
    motor = {}
    if os.path.exists(md_path):
        with open(md_path) as f:
            p = yaml.safe_load(f)["motor_driver"]["ros__parameters"]
        motor = {
            "steer_map": [[x, y] for x, y in
                          zip(p["steer_map_delta_rad"], p["steer_map_u"])],
            "throttle_map": [[x, y] for x, y in
                             zip(p["throttle_map_v_mps"], p["throttle_map_u"])],
            **{k: p[k] for k in ("v_eps", "count_min", "count_max") if k in p},
        }
        print(f"  {md_path} の実設定で検証する")

    # PWM カウントの実測値。FaBo の 01_find_pwm.ipynb が書き出す JSON と同形。
    pwm = None
    pwm_path = os.path.join(CONFIG, "pwm_params.json")
    if os.path.exists(pwm_path):
        import json
        with open(pwm_path) as f:
            pwm = json.load(f)
        print(f"  {pwm_path} の校正値で検証する")
    return MotorConfig.from_yaml(vp, motor, pwm)


# ----------------------------------------------------------------------
def test_roundtrip(core: MotorDriverCore) -> None:
    """1. mppi_node の出力式と往復で一致するか。

    minicar_mppi の mppi_node は omega = v*tan(delta)/L で publish する。
    ここが atan(omega*L/v) で読み戻すので、両者が一致しないと
    「MPPI が要求した舵角」と「車輪が向く角」が食い違う。
    リポジトリ全体で式を揃えていることの回帰テスト。
    """
    L = core.cfg.wheelbase
    for v in (0.3, 1.0, 2.0):
        for delta in np.linspace(-core.cfg.delta_max, core.cfg.delta_max, 9):
            omega = v * math.tan(delta) / L
            got = core.steer_angle(v, omega)
            check(abs(got - delta) < 1e-9,
                  f"1. 往復不一致 v={v} delta={delta:.4f} -> {got:.4f}")


def test_low_speed_holds(core: MotorDriverCore) -> None:
    """2. |v| < v_eps では直前の舵角を保持し、0 割りも NaN も出さない。

    中立へ戻さないのは、safety_node の後退脱出が v を 0 を跨いで
    反転させるため。そこで舵がまっすぐになると切り返しが成立しない。
    """
    eps = core.cfg.v_eps
    for v in (0.0, eps * 0.5, -eps * 0.5):
        out = core.convert(v, 1.0, delta_prev=0.25)
        check(out.valid, f"2. v={v} で valid でない")
        check(abs(out.delta_rad - 0.25) < 1e-12,
              f"2. v={v} で舵角を保持していない: {out.delta_rad}")
        check(math.isfinite(out.u_steer) and math.isfinite(out.u_throttle),
              f"2. v={v} で非有限の指令: {out}")
    # v_eps をわずかに超えたら通常の逆変換に切り替わる
    out = core.convert(eps * 1.01, 0.0, delta_prev=0.25)
    check(abs(out.delta_rad) < 1e-9, f"2. v_eps 超で逆変換していない: {out.delta_rad}")


def test_steer_clamp(core: MotorDriverCore) -> None:
    """3. delta_max を超える omega は車両限界でクランプされる。"""
    L, dmax = core.cfg.wheelbase, core.cfg.delta_max
    for sign in (+1.0, -1.0):
        omega = sign * 10.0 * math.tan(dmax) / L   # delta_max 相当の 10 倍
        out = core.convert(1.0, omega)
        check(abs(abs(out.delta_rad) - dmax) < 1e-12,
              f"3. 舵角がクランプされていない: {out.delta_rad} (dmax={dmax})")
        check(out.steer_saturated, "3. 飽和フラグが立っていない")


def test_speed_clamp(core: MotorDriverCore) -> None:
    """4. v_max / v_min の外はクランプされる。"""
    for v, expect in ((100.0, core.cfg.v_max), (-100.0, core.cfg.v_min)):
        out = core.convert(v, 0.0)
        check(abs(out.v_cmd - expect) < 1e-12,
              f"4. 速度がクランプされていない: {v} -> {out.v_cmd} (期待 {expect})")


def test_nonfinite(core: MotorDriverCore) -> None:
    """5. NaN / inf は中立で返し、例外を投げない。

    例外で落とさないのは、制御ノードが NaN を出した瞬間にドライバまで
    道連れで死ぬと、ESC が最後のパルスをラッチしたまま誰も止められなく
    なるため。止まって生き残るほうが安全。
    """
    bad = (float("nan"), float("inf"), float("-inf"))
    for v in bad:
        out = core.convert(v, 0.0)
        check(out == MotorCommand(0.0, 0.0, 0.0, 0.0, False, False),
              f"5. v={v} で中立を返していない: {out}")
    for omega in bad:
        out = core.convert(1.0, omega)
        check(not out.valid, f"5. omega={omega} で valid のまま: {out}")
    for d in bad:
        out = core.convert(1.0, 0.0, delta_prev=d)
        check(not out.valid, f"5. delta_prev={d} で valid のまま: {out}")
    for kind, mid in (("steer", core.cfg.steer_center),
                      ("throttle", core.cfg.throttle_stop)):
        check(core.to_count(float("nan"), kind) == mid,
              f"5. to_count(NaN, {kind}) が中立でない")


def test_map_shape() -> None:
    """6. 非対称・デッドバンド付きマップが単調で、範囲外はクランプされる。

    校正マップの並べ間違いは走ってみるまで気付けない種類の事故なので、
    読み込み時に落ちることもここで確かめる。
    """
    # 左右非対称 + 中立デッドバンドを持つ現実的な形
    table = as_table([[-0.40, -1.0], [-0.02, -0.15], [0.0, 0.0],
                      [0.02, 0.12], [0.45, 0.9]], "t")
    xs = np.linspace(-0.6, 0.6, 201)
    ys = [piecewise_linear(x, table) for x in xs]
    check(all(b >= a - 1e-12 for a, b in zip(ys, ys[1:])), "6. 補間が単調でない")
    check(abs(piecewise_linear(-99.0, table) - (-1.0)) < 1e-12,
          "6. 下側でクランプしていない")
    check(abs(piecewise_linear(+99.0, table) - 0.9) < 1e-12,
          "6. 上側でクランプしていない")
    # 折れ点を正確に通るか
    for x, y in table:
        check(abs(piecewise_linear(x, table) - y) < 1e-12, f"6. 折れ点 {x} を外した")

    # 壊れたテーブルは受け付けない
    for bad, why in (([[0.1, 0.0], [-0.1, 1.0]], "降順"),
                     ([[0.0, 0.0], [0.0, 1.0]], "重複 x"),
                     ([[0.0, 0.0]], "1 点のみ"),
                     ([[0.0, 0.0], [float("nan"), 1.0]], "NaN")):
        try:
            as_table(bad, "t")
            FAILURES.append(f"6. {why} のテーブルが通ってしまった: {bad}")
        except ValueError:
            pass


def test_count_bounds(core: MotorDriverCore) -> None:
    """7. PWM カウントが端点と count_min/count_max を絶対に超えない。

    ここを越えるとサーボが機械端に突っ込んでギアを舐めるか、ESC が
    異常値として入力を無視する。最後の砦なので入力を極端に振って確かめる。
    """
    c = core.cfg
    # lo が u=-1、hi が u=+1。操舵は u=+1 が left (REP-103)
    for kind, lo, mid, hi in (("steer", c.steer_right, c.steer_center, c.steer_left),
                              ("throttle", c.throttle_back, c.throttle_stop,
                               c.throttle_front)):
        for u in np.linspace(-5.0, 5.0, 401):
            n = core.to_count(float(u), kind)
            check(c.count_min <= n <= c.count_max,
                  f"7. {kind} カウントが count 範囲外: u={u} -> {n}")
            check(min(lo, hi) <= n <= max(lo, hi),
                  f"7. {kind} カウントが端点 {lo}..{hi} の外: u={u} -> {n}")
        # 3 点を正確に通ること
        for u, expect in ((-1.0, lo), (0.0, mid), (1.0, hi)):
            n = core.to_count(u, kind)
            check(n == expect, f"7. {kind} u={u} が {expect} でなく {n}")
    try:
        core.to_count(0.0, "throtle")   # 綴り間違い
        FAILURES.append("7. 未知の kind が通ってしまった")
    except ValueError:
        pass


def test_center_not_midpoint() -> None:
    """7b. 実測中立が左右の中点でなくても、u=0 は実測中立を出す。

    FaBo の実走 Notebook は map_rc(x, 224, 0, pwm_right, pwm_left) という
    left..right の 2 点線形で、実測 center を出力に使っていない。こちらは
    delta=0 を頻繁に出す (停止中・直線・ウォッチドッグの中立・後退の中立
    ギャップ) ので、そこが実測中立でないと「止めたつもりなのに曲がる」。
    その差分の回帰テスト。
    """
    # 左右非対称: center 410 は left 300 と right 560 の中点 430 ではない
    cfg = MotorConfig(steer_left=300, steer_center=410, steer_right=560)
    core = MotorDriverCore(cfg)
    check(core.to_count(0.0, "steer") == 410, "7b. u=0 が実測中立を出していない")
    check(core.to_count(1.0, "steer") == 300, "7b. u=+1 が left でない")
    check(core.to_count(-1.0, "steer") == 560, "7b. u=-1 が right でない")
    # 2 点線形なら u=0 で 430 になる。そうなっていないことを明示しておく。
    check(core.to_count(0.0, "steer") != 430,
          "7b. 2 点線形の中点になっている (FaBo の map_rc と同じ挙動)")
    # 片側だけ見れば傾きは端点まで線形
    check(core.to_count(-0.5, "steer") == 485, "7b. 右半分の線形が合わない")
    check(core.to_count(0.5, "steer") == 355, "7b. 左半分の線形が合わない")


def test_steer_direction() -> None:
    """7d. 左旋回の指令 (omega>0) で left 側のカウントへ動く。

    REP-103 では delta>0 が左。to_count が u=+1 を right に割り当てていた
    ため、実機で「左を指令して右へ切れる」不具合が出た。その回帰テスト。
    left < right と left > right (極性反転) の両方で確かめる。
    """
    for left, right in ((310, 440), (440, 310)):
        cfg = MotorConfig(steer_left=left, steer_center=390, steer_right=right)
        core = MotorDriverCore(cfg)
        L = cfg.wheelbase
        for v in (0.5, 1.0):
            omega = v * math.tan(0.2) / L       # delta=+0.2rad の左旋回
            n_left = core.to_count(core.convert(v, omega).u_steer, "steer")
            n_right = core.to_count(core.convert(v, -omega).u_steer, "steer")
            check(abs(n_left - left) < abs(390 - left),
                  f"7d. 左旋回で left={left} 側へ動かない: {n_left}")
            check(abs(n_right - right) < abs(390 - right),
                  f"7d. 右旋回で right={right} 側へ動かない: {n_right}")


def test_reversed_polarity() -> None:
    """7c. 極性が逆 (left > right) の校正値でも端点を正しく通る。

    サーボの取り付け向きで左右が入れ替わることがある。FaBo の
    01_find_pwm.ipynb にも Reverse チェックボックスがあり、その場合は
    pwm_params.json の left > right になる。
    """
    cfg = MotorConfig(steer_left=510, steer_center=410, steer_right=310)
    core = MotorDriverCore(cfg)
    for u, expect in ((1.0, 510), (0.0, 410), (-1.0, 310)):
        check(core.to_count(u, "steer") == expect,
              f"7c. 極性反転で u={u} が {expect} でない")


def test_full_sweep(core: MotorDriverCore) -> None:
    """8. 実設定の全域で指令が [-1,1] に収まり、カウントが端点内に入る。

    「1 点だけ試して大丈夫だった」を防ぐため、v と omega を車両限界の
    外側まで含めて総当たりする。
    """
    c = core.cfg
    L = c.wheelbase
    vs = np.linspace(c.v_min * 1.5, c.v_max * 1.5, 61)
    omegas = np.linspace(-2.0 * c.v_max * math.tan(c.delta_max) / L,
                         +2.0 * c.v_max * math.tan(c.delta_max) / L, 61)
    worst_s = worst_t = 0
    for v in vs:
        for omega in omegas:
            out = core.convert(float(v), float(omega), delta_prev=0.0)
            check(-1.0 <= out.u_steer <= 1.0, f"8. u_steer 範囲外: {out}")
            check(-1.0 <= out.u_throttle <= 1.0, f"8. u_throttle 範囲外: {out}")
            check(abs(out.delta_rad) <= c.delta_max + 1e-12,
                  f"8. 舵角が delta_max 超: {out}")
            ns = core.to_count(out.u_steer, "steer")
            nt = core.to_count(out.u_throttle, "throttle")
            check(min(c.steer_left, c.steer_right) <= ns
                  <= max(c.steer_left, c.steer_right),
                  f"8. steer カウントが端点外: {ns}")
            check(min(c.throttle_back, c.throttle_front) <= nt
                  <= max(c.throttle_back, c.throttle_front),
                  f"8. throttle カウントが端点外: {nt}")
            worst_s = max(worst_s, abs(ns - c.steer_center))
            worst_t = max(worst_t, abs(nt - c.throttle_stop))
    print(f"  中立からの最大振れ幅 steer {worst_s} / throttle {worst_t} カウント "
          f"(端点まで steer {max(abs(c.steer_left - c.steer_center), abs(c.steer_right - c.steer_center))} / "
          f"throttle {max(abs(c.throttle_back - c.throttle_stop), abs(c.throttle_front - c.throttle_stop))})")


def test_config_validation() -> None:
    """9. 矛盾した設定は起動時に落ちる。

    走行中の「なんとなく曲がらない」は原因の特定に時間がかかる。
    起動時に落とすほうが安い。
    """
    cases = {
        "delta_max を覆わない steer_map":
            dict(steer_map=as_table([[-0.1, -1.0], [0.1, 1.0]], "t")),
        "出力が [-1,1] を超える steer_map":
            dict(steer_map=as_table([[-0.5, -1.5], [0.5, 1.5]], "t")),
        "steer の中立が端点の外": dict(steer_left=300, steer_center=600,
                                    steer_right=520),
        "throttle の中立が端点の外": dict(throttle_back=390, throttle_stop=300,
                                      throttle_front=430),
        "steer の端点が同値": dict(steer_left=410, steer_center=410,
                              steer_right=410),
        "カウントが 12bit 外": dict(steer_left=310, steer_center=410,
                               steer_right=5000),
        "wheelbase が 0": dict(wheelbase=0.0),
        "v_max < 0": dict(v_min=-2.0, v_max=-1.0),
    }
    for why, kw in cases.items():
        try:
            MotorConfig(**kw).validate()
            FAILURES.append(f"9. {why} が通ってしまった")
        except ValueError:
            pass


def test_backend(core: MotorDriverCore) -> None:
    """10. dryrun バックエンドは書き込みを保持し、非有限値を弾く。

    実機バックエンドと同じ検証を通すことで、「dryrun では通るのに
    実機で落ちる」を防ぐ。
    """
    be = make_backend("dryrun")
    check(isinstance(be, DryRunBackend), "10. dryrun バックエンドが作れない")
    be.set_count(0, core.cfg.steer_center)
    check(be.last[0] == core.cfg.steer_center, "10. 書き込みが保持されていない")
    for bad, why in ((float("nan"), "NaN"), (float("inf"), "inf"),
                     (-1, "負のカウント"), (4096, "12bit 超")):
        try:
            be.set_count(0, bad)
            FAILURES.append(f"10. {why} のカウントが通ってしまった: {bad}")
        except ValueError:
            pass
    try:
        make_backend("pca9865")   # 綴り間違い
        FAILURES.append("10. 未知の backend 名が通ってしまった")
    except ValueError:
        pass


# ----------------------------------------------------------------------
def load_speed_params() -> dict:
    """motor_driver_params.yaml の速度 PI の値。無ければノードの既定と同じ値。"""
    p = {"speed_kp": 0.15, "speed_ki": 0.6, "speed_i_min": -0.4,
         "speed_i_max": 0.3, "speed_u_max": 0.8, "speed_u_min": 0.0,
         "speed_control": "off"}
    md_path = os.path.join(CONFIG, "motor_driver_params.yaml")
    if os.path.exists(md_path):
        import yaml
        with open(md_path, encoding="utf-8") as f:
            y = yaml.safe_load(f)["motor_driver"]["ros__parameters"]
        p.update({k: y[k] for k in p if k in y})
    return p


def make_speed(p: dict) -> SpeedController:
    return SpeedController(kp=p["speed_kp"], ki=p["speed_ki"],
                           i_min=p["speed_i_min"], i_max=p["speed_i_max"],
                           u_max=p["speed_u_max"], u_min=p["speed_u_min"])


def test_speed_pi_basic() -> None:
    """11. 速度 PI: 遅ければ足し、速ければ引き、出力は [0, u_max] に収まる。"""
    sc = SpeedController(kp=0.2, ki=1.0, i_min=-0.4, i_max=0.3, u_max=0.8)
    out = sc.update(v_cmd=0.6, v_meas=0.0, u_ff=0.3, dt=0.02)
    check(out.u > 0.3, f"11. 遅いのに足していない: {out}")
    check(abs(out.u_p - 0.2 * 0.6) < 1e-12, f"11. 比例項が kp*e でない: {out.u_p}")
    sc.reset()
    out = sc.update(v_cmd=0.6, v_meas=1.2, u_ff=0.3, dt=0.02)
    check(out.u < 0.3, f"11. 速いのに引いていない: {out}")
    # 大きく速すぎても負 (ブレーキ / 後退) は出さない
    out = sc.update(v_cmd=0.5, v_meas=5.0, u_ff=0.1, dt=0.02)
    check(out.u == 0.0, f"11. PI が負のスロットルを出した: {out.u}")
    # e=0 で u_ff そのまま
    sc.reset()
    out = sc.update(v_cmd=0.6, v_meas=0.6, u_ff=0.37, dt=0.02)
    check(abs(out.u - 0.37) < 1e-12, f"11. 誤差 0 なのに u_ff から動いた: {out.u}")


def test_speed_pi_stall_and_windup() -> None:
    """12. 止まったままなら積分で押し出す。ただし上限で頭打ちし、溜め込まない。

    2026-10-16 の bag では、つなぎ目と舵いっぱいのコーナーで同じカウントのまま
    7〜100 秒以上止まっていた。積分が無いと出力が増えない。
    逆に溜め込みすぎると、動き出した瞬間に飛び出す (アンチワインドアップ)。
    """
    sc = SpeedController(kp=0.15, ki=0.6, i_min=-0.4, i_max=0.3, u_max=0.8)
    u_hist = [sc.update(0.6, 0.0, 0.3, 0.02).u for _ in range(50)]   # 1 秒止まったまま
    check(u_hist[-1] > u_hist[0] + 0.2,
          f"12. 1 秒止まっても出力がほとんど増えない: {u_hist[0]:.3f} -> {u_hist[-1]:.3f}")
    check(all(b >= a - 1e-12 for a, b in zip(u_hist, u_hist[1:])),
          "12. 止まっている間に出力が下がった")
    for _ in range(5000):                                           # 100 秒止まったまま
        out = sc.update(0.6, 0.0, 0.3, 0.02)
    check(out.u_i <= 0.3 + 1e-12, f"12. 積分が i_max を超えた: {out.u_i}")
    check(out.u <= 0.8 + 1e-12, f"12. 出力が u_max を超えた: {out.u}")
    # u_max に張り付く条件では、張り付いた時点で積分を止める
    sc2 = SpeedController(kp=0.15, ki=0.6, i_min=-0.4, i_max=0.3, u_max=0.6)
    for _ in range(5000):
        out2 = sc2.update(0.6, 0.0, 0.45, 0.02)
    check(out2.u_i < 0.3 - 1e-6,
          f"12. u_max に張り付いても積分し続けた (ワインドアップ): u_i={out2.u_i}")
    # 動き出して追いついたら、溜めた分を引きずらず速やかに戻る
    n = 0
    while sc2.update(0.6, 0.9, 0.45, 0.02).u > 0.45 and n < 500:
        n += 1
    check(n * 0.02 < 1.0, f"12. 速すぎるのに出力が下がるまで {n * 0.02:.2f}s かかった")


def test_speed_pi_floor() -> None:
    """12b. 前進中は min(u_min, u_ff) より下げない。下限に張り付いても負側に溜めない。

    2026-10-19 の実走で、指令を追い越したあと中立の手前まで下げて止まりかけた。
    """
    sc = SpeedController(kp=0.15, ki=0.4, i_min=-0.15, i_max=0.3, u_max=0.7,
                         u_min=0.25)
    for _ in range(500):                                    # 大きく追い越したまま 10 秒
        out = sc.update(v_cmd=0.5, v_meas=1.5, u_ff=0.33, dt=0.02)
    check(abs(out.u - 0.25) < 1e-12, f"12b. 下限 0.25 で止まらない: {out.u}")
    check(out.u_i > -0.15 + 1e-6,
          f"12b. 下限に張り付いているのに負側へ積分し続けた: u_i={out.u_i}")
    # 指令に戻ったら速やかに u_ff 付近へ戻る (負の積分を溜めていないので)
    out = sc.update(v_cmd=0.5, v_meas=0.5, u_ff=0.33, dt=0.02)
    check(out.u >= 0.25 and abs(out.u - (0.33 + out.u_i)) < 1e-12,
          f"12b. 追いついた後の出力がおかしい: {out}")
    # u_ff が下限より小さい指令では u_ff が下限
    sc.reset()
    out = sc.update(v_cmd=0.1, v_meas=1.0, u_ff=0.21, dt=0.02)
    check(abs(out.u - 0.21) < 1e-12, f"12b. 小さい指令で u_ff より上に持ち上げた: {out.u}")


def test_speed_pi_reset_and_nonfinite() -> None:
    """13. reset で積分を捨てる。NaN 入力は積分を捨てて u_ff だけ返す。"""
    sc = SpeedController(kp=0.15, ki=0.6, i_min=-0.4, i_max=0.3, u_max=0.8)
    for _ in range(100):
        sc.update(0.6, 0.0, 0.3, 0.02)
    sc.reset()
    check(sc.u_i == 0.0, f"13. reset で積分が残った: {sc.u_i}")
    for _ in range(100):
        sc.update(0.6, 0.0, 0.3, 0.02)
    for bad in ((math.nan, 0.0, 0.3, 0.02), (0.6, math.inf, 0.3, 0.02),
                (0.6, 0.0, 0.3, 0.0), (0.6, 0.0, 0.3, -0.02)):
        out = sc.update(*bad)
        check(sc.u_i == 0.0, f"13. 非有限 / dt<=0 {bad} で積分が残った")
        check(abs(out.u - 0.3) < 1e-12, f"13. 非有限 {bad} で u_ff 以外を出した: {out.u}")
    out = sc.update(0.6, 0.0, math.nan, 0.02)
    check(out.u == 0.0, f"13. u_ff が NaN で 0 以外を出した: {out.u}")


def test_speed_pi_validation() -> None:
    """14. 矛盾するゲイン・範囲は構築時に落とす。"""
    for kw in ({"kp": -0.1}, {"ki": -1.0}, {"i_min": 0.1}, {"i_max": -0.1},
               {"u_max": 0.0}, {"u_max": 1.5}, {"u_min": -0.1}, {"u_min": 0.8}):
        args = dict(kp=0.15, ki=0.6, i_min=-0.4, i_max=0.3, u_max=0.8)
        args.update(kw)
        try:
            SpeedController(**args)
            check(False, f"14. {kw} を受け付けた")
        except ValueError:
            pass


def test_speed_pi_real_config(core: MotorDriverCore) -> None:
    """15. 実設定の yaml で、最悪でも出力が front..stop の間に収まる。

    /odom が壊れて 0 を返し続けても (積分が i_max まで溜まっても)、
    出力カウントは speed_u_max で頭打ちし、front を越えない。
    """
    p = load_speed_params()
    check(p["speed_control"] in ("off", "pi"),
          f"15. speed_control が off / pi でない: {p['speed_control']!r}")
    sc = make_speed(p)
    worst = 0.0
    for v_cmd in np.linspace(core.cfg.v_eps * 1.01, core.cfg.v_max, 30):
        sc.reset()
        u_ff = core.throttle_to_u(v_cmd)
        for _ in range(2000):
            out = sc.update(v_cmd, 0.0, u_ff, 0.02)
        worst = max(worst, out.u)
    c_worst = core.to_count(worst, "throttle")
    lo, hi = sorted((core.cfg.throttle_front, core.cfg.throttle_stop))
    check(lo <= c_worst <= hi,
          f"15. 最悪出力 {c_worst} カウントが front..stop ({lo}..{hi}) の外")
    check(worst <= p["speed_u_max"] + 1e-12, f"15. 最悪出力 u={worst} が u_max を超えた")
    print(f"  速度 PI の最悪出力 u={worst:.3f} = {c_worst} カウント "
          f"(kp={p['speed_kp']} ki={p['speed_ki']} i={p['speed_i_min']}..{p['speed_i_max']})")


def main() -> int:
    print("motor_driver_core の単体テスト")
    cfg = load_cfg()
    core = MotorDriverCore(cfg)
    print(f"  L={cfg.wheelbase}m delta_max={cfg.delta_max}rad "
          f"v={cfg.v_min:+.1f}..{cfg.v_max:+.1f}m/s v_eps={cfg.v_eps}m/s")
    print(f"  PWM カウント steer {cfg.steer_left}/{cfg.steer_center}/{cfg.steer_right} "
          f"throttle {cfg.throttle_back}/{cfg.throttle_stop}/{cfg.throttle_front}")

    tests = [
        ("1. mppi_node の出力式との往復", lambda: test_roundtrip(core)),
        ("2. 低速域で舵角を保持", lambda: test_low_speed_holds(core)),
        ("3. 舵角クランプ", lambda: test_steer_clamp(core)),
        ("4. 速度クランプ", lambda: test_speed_clamp(core)),
        ("5. NaN / inf で中立", lambda: test_nonfinite(core)),
        ("6. 校正マップの形状と検証", test_map_shape),
        ("7. PWM カウントの範囲と端点", lambda: test_count_bounds(core)),
        ("7b. 実測中立を使う (FaBo の map_rc との差分)", test_center_not_midpoint),
        ("7c. 極性反転の校正値", test_reversed_polarity),
        ("7d. 舵の向き (左旋回で left 側)", test_steer_direction),
        ("8. 実設定の全域スイープ", lambda: test_full_sweep(core)),
        ("9. 設定の矛盾検出", test_config_validation),
        ("10. PWM バックエンド", lambda: test_backend(core)),
        ("11. 速度 PI の向きと範囲", test_speed_pi_basic),
        ("12. 速度 PI の停止からの押し出しとワインドアップ", test_speed_pi_stall_and_windup),
        ("12b. 速度 PI の出力下限", test_speed_pi_floor),
        ("13. 速度 PI の reset と非有限入力", test_speed_pi_reset_and_nonfinite),
        ("14. 速度 PI の設定検証", test_speed_pi_validation),
        ("15. 実設定の速度 PI の最悪出力", lambda: test_speed_pi_real_config(core)),
    ]
    for name, fn in tests:
        before = len(FAILURES)
        fn()
        print(f"  [{'NG' if len(FAILURES) > before else 'ok'}] {name}")

    if FAILURES:
        print(f"\n{len(FAILURES)} 件失敗:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("\nすべて通過")
    return 0


if __name__ == "__main__":
    sys.exit(main())
