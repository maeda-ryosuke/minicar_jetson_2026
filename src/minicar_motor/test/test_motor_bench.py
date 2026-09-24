#!/usr/bin/env python3
"""motor_bench の純関数部分の単体テスト。ROS も I2C も車両も不要。

    cd /ws && colcon test --packages-select minicar_motor \
      --event-handlers console_direct+ && colcon test-result --verbose

    # ソースツリーから直接
    PYTHONPATH=jetson_src/minicar_motor python3 \
      jetson_src/minicar_motor/test/test_motor_bench.py

ここで確定させたいのは 2 点。

  1. **実機へ出す前に落とす。** delta_max を超える舵角や 12bit を外れた
     カウントは、指令として出た時点でサーボが機械端に当たる。
     出す前のパース段で落ちていること。
  2. **保存した JSON で motor_driver が起動できる。** 校正セッションの
     成果物が起動時に弾かれると、実機に通電した後で気付くことになる。

test_motor_driver_core.py と違い、pytest から呼ばれる test_* 関数に
引数を持たせていない (fixture 扱いされて collection error になるため)。
"""

import functools
import math
import os
import sys

# colcon test はソースツリーで走る。パッケージの親を通しておくと
# `python3 test/test_motor_bench.py` でも同じ import で通る。
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from minicar_motor.motor_bench import (  # noqa: E402
    build_pwm_json, delta_from_radius, describe_run, omega_from_delta,
    parse_count_input, parse_sweep, turn_radius,
)
from minicar_motor.motor_driver_core import MotorConfig, MotorDriverCore  # noqa: E402
from minicar_motor.pwm_backend import COUNT_MAX_HW  # noqa: E402

L = 0.257          # [m] vehicle_params.yaml の wheelbase
DELTA_MAX = 0.42   # [rad] 同 limits.delta_max

FAILURES: list[str] = []


def check(condition: bool, message: str) -> None:
    """失敗を溜めて**続行する**。1 回の実行で全件を洗い出すため。"""
    if not condition:
        FAILURES.append(message)


def reports(fn):
    """pytest から呼ばれたとき、この関数で増えた失敗を assert で伝える。

    check() は溜めるだけなので、素のままだと pytest からは常に pass に
    見える。**colcon test が緑なのに壊れている**状態が一番危ないので、
    関数の出口で増分を見て落とす。
    main() は集計のために .raw で元の関数を呼ぶ (途中で止めない)。
    """
    @functools.wraps(fn)
    def wrapper(*a, **kw):
        start = len(FAILURES)
        fn(*a, **kw)
        new = FAILURES[start:]
        assert not new, "\n".join(new)

    wrapper.raw = fn
    return wrapper


def expect_raises(fn, message: str) -> None:
    """例外が出ることを確かめる。**出ないことがバグ**の箇所で使う。"""
    try:
        fn()
    except (ValueError, TypeError):
        return
    FAILURES.append(message)


# ----------------------------------------------------------------------
@reports
def test_roundtrip_with_core() -> None:
    """delta -> omega -> delta が motor_driver_core と往復一致すること。

    **同じ式を 2 箇所に書いた結果ずれる事故**を縛る。ベンチが
    omega = v*tan(delta)/L で出したものを、ドライバが
    delta = atan(omega*L/v) で読み戻す。片方だけ直すと、指令した舵角と
    車輪が向く角が食い違ったまま測定してしまう。
    """
    core = MotorDriverCore(MotorConfig(wheelbase=L, delta_max=DELTA_MAX))
    for v in (0.3, 0.4, 1.0, 2.0):
        for delta in (-0.42, -0.30, -0.10, 0.0, 0.10, 0.30, 0.42):
            omega = omega_from_delta(v, delta, L)
            back = core.steer_angle(v, omega, delta_prev=0.0)
            check(abs(back - delta) < 1e-9,
                  f"往復不一致 v={v} delta={delta}: {back}")

    # 後退側も同じ式で通ること。safety_node のスタック脱出は v を負で出す。
    omega = omega_from_delta(-0.3, 0.30, L)
    back = MotorDriverCore(
        MotorConfig(wheelbase=L, delta_max=DELTA_MAX)).steer_angle(-0.3, omega)
    check(abs(back - 0.30) < 1e-9, f"後退で往復不一致: {back}")

    expect_raises(lambda: omega_from_delta(float("nan"), 0.1, L),
                  "NaN の v を受け付けた")
    expect_raises(lambda: omega_from_delta(0.4, 0.1, 0.0),
                  "wheelbase=0 を受け付けた")


@reports
def test_turn_radius() -> None:
    """R = L/tan(delta)。delta=0 で 0 除算せず inf を返すこと。

    直進を測定メニューに入れる (中立確認) ので、ここで落ちると
    手順 B が実行できない。
    """
    check(math.isinf(turn_radius(0.0, L)), "delta=0 が inf でない")
    check(math.isinf(turn_radius(1e-12, L)), "極小 delta が inf でない")

    # 手順書に載せる値。ここがずれると必要な広さの記述が嘘になる。
    for delta, expect in ((0.42, 0.5755), (0.30, 0.8308), (0.20, 1.2678),
                          (0.10, 2.5614), (0.05, 5.1357)):
        r = turn_radius(delta, L)
        check(abs(r - expect) < 0.001,
              f"R(delta={delta}) = {r:.3f}, 期待 {expect}")
        # 左右で同じ半径になること (符号だけの違い)。
        check(abs(turn_radius(-delta, L) - r) < 1e-12,
              f"左右で半径が違う: delta={delta}")

    # 測った R から舵角を逆算する式との往復。現場で使うのはこちら向き。
    for delta in (0.05, 0.10, 0.20, 0.30, 0.42):
        back = delta_from_radius(turn_radius(delta, L), L)
        check(abs(back - delta) < 1e-9,
              f"R -> delta の往復不一致 delta={delta}: {back}")

    expect_raises(lambda: delta_from_radius(0.0, L), "R=0 を受け付けた")
    expect_raises(lambda: delta_from_radius(-1.0, L), "負の R を受け付けた")


@reports
def test_sweep_parse() -> None:
    """--sweep の順序保持と、delta_max 超過をその場で落とすこと。

    **クランプに頼らない。** motor_driver 側のクランプに任せると
    「0.60 を指令したのに 0.42 の円が描かれた」データを測ってしまい、
    steer_map に嘘が入る。
    """
    got = parse_sweep("0.42,0.30,0,-0.30,-0.42", DELTA_MAX)
    check(got == [0.42, 0.30, 0.0, -0.30, -0.42], f"順序が崩れた: {got}")

    # 昇順に並べ替えないこと。測定は大舵角から降りる順に意味がある。
    got = parse_sweep("-0.10,0.42,0.0", DELTA_MAX)
    check(got == [-0.10, 0.42, 0.0], f"並べ替えられた: {got}")

    check(parse_sweep(" 0.20 , 0.10 ", DELTA_MAX) == [0.20, 0.10],
          "空白を含む要素を読めない")
    check(parse_sweep("0.20,,0.10", DELTA_MAX) == [0.20, 0.10],
          "空要素で落ちた")

    expect_raises(lambda: parse_sweep("0.43", DELTA_MAX),
                  "delta_max 超過を受け付けた")
    expect_raises(lambda: parse_sweep("-0.50,0.10", DELTA_MAX),
                  "負側の delta_max 超過を受け付けた")
    expect_raises(lambda: parse_sweep("0.10,abc", DELTA_MAX),
                  "数値でない要素を受け付けた")
    expect_raises(lambda: parse_sweep("nan", DELTA_MAX), "NaN を受け付けた")
    expect_raises(lambda: parse_sweep("", DELTA_MAX), "空の sweep を受け付けた")

    # 境界そのものは通ること。delta_max ちょうどが測れないと
    # 最大舵角の校正点が取れない。
    check(parse_sweep(str(DELTA_MAX), DELTA_MAX) == [DELTA_MAX],
          "delta_max ちょうどが弾かれた")


@reports
def test_count_input() -> None:
    """絶対 / 相対の解釈と、12bit を外れた入力を落とすこと。"""
    check(parse_count_input("420", 410) == 420, "絶対入力を読めない")
    check(parse_count_input("+10", 410) == 420, "相対 + を読めない")
    check(parse_count_input("-5", 410) == 405, "相対 - を読めない")
    check(parse_count_input(" +1 ", 410) == 411, "空白付きを読めない")
    check(parse_count_input("410.6", 0) == 411, "小数の丸めが違う")

    # 0 と上限そのものは書ける。端点が書けないと可動端を探せない。
    check(parse_count_input("0", 10) == 0, "0 が弾かれた")
    check(parse_count_input(str(COUNT_MAX_HW), 0) == COUNT_MAX_HW,
          "上限が弾かれた")

    expect_raises(lambda: parse_count_input("-1", 0), "負のカウントを受け付けた")
    expect_raises(lambda: parse_count_input("-20", 10),
                  "相対で負に振り切ったのを受け付けた")
    expect_raises(lambda: parse_count_input(str(COUNT_MAX_HW + 1), 0),
                  "12bit 超過を受け付けた")
    expect_raises(lambda: parse_count_input("+100", COUNT_MAX_HW),
                  "相対で上限を超えたのを受け付けた")
    expect_raises(lambda: parse_count_input("abc", 410),
                  "数値でない入力を受け付けた")
    expect_raises(lambda: parse_count_input("", 410), "空入力を受け付けた")


@reports
def test_pwm_json_build() -> None:
    """保存する JSON が motor_driver の起動時検証を通ること。

    **不正な JSON を保存させないのがここの仕事。** 保存できてしまうと、
    次に motor_driver を起動したときに初めて落ちる。それは実機に通電した
    後なので高くつく。
    """
    data = build_pwm_json({"left": 310, "center": 410, "right": 510},
                          {"back": 390, "stop": 410, "front": 430})
    check(set(data) == {"pwm_steering", "pwm_speed"},
          f"キー構成が pwm_params.json と違う: {list(data)}")
    check(data["pwm_steering"] == {"left": 310, "center": 410, "right": 510},
          f"操舵の値が変わった: {data['pwm_steering']}")
    check(data["pwm_speed"] == {"back": 390, "stop": 410, "front": 430},
          f"速度の値が変わった: {data['pwm_speed']}")

    # 極性が逆のサーボ (left > right)。FaBo の Reverse チェックボックス相当。
    # 中立が端点の**間**にあればよく、left < right である必要は無い。
    rev = build_pwm_json({"left": 510, "center": 410, "right": 310},
                         {"back": 430, "stop": 410, "front": 390})
    check(rev["pwm_steering"]["left"] == 510, "極性反転の校正値が弾かれた")

    # 中立が実測で非対称 (left と right の中点でない) 場合も通ること。
    asym = build_pwm_json({"left": 300, "center": 395, "right": 520},
                          {"back": 385, "stop": 408, "front": 445})
    check(asym["pwm_steering"]["center"] == 395, "非対称な中立が弾かれた")

    # --- ここから先は「保存させてはいけない」形 ---
    expect_raises(
        lambda: build_pwm_json({"left": 310, "center": 410},
                               {"back": 390, "stop": 410, "front": 430}),
        "right を記録せずに保存できた")
    expect_raises(
        lambda: build_pwm_json({"left": 310, "center": 410, "right": 510}, {}),
        "スロットルを記録せずに保存できた")
    expect_raises(
        lambda: build_pwm_json({"left": 310, "center": 520, "right": 510},
                               {"back": 390, "stop": 410, "front": 430}),
        "中立が端点の外にある値を保存できた (u=0 が左いっぱいになる)")
    expect_raises(
        lambda: build_pwm_json({"left": 410, "center": 410, "right": 410},
                               {"back": 390, "stop": 410, "front": 430}),
        "端点が同値 (振れ幅 0) の値を保存できた")
    expect_raises(
        lambda: build_pwm_json({"left": 310, "center": 410, "right": 510},
                               {"back": 390, "stop": 450, "front": 430}),
        "スロットルの中立が端点の外にある値を保存できた (停止指令で全開)")
    expect_raises(
        lambda: build_pwm_json({"left": -1, "center": 410, "right": 510},
                               {"back": 390, "stop": 410, "front": 430}),
        "12bit の範囲外を保存できた")


@reports
def test_describe_run() -> None:
    """実行前の読み上げが全経路で例外を出さないこと。

    ここで落ちると「何が起きるか分からないまま実行する」か
    「実行できない」かのどちらかになる。直進 (R=inf) を含む。
    """
    for delta in (0.0, 0.42, -0.42, 0.10):
        for v in (0.0, 0.4, -0.3, 2.0):
            try:
                text = describe_run(v, delta, 5.0, L)
            except Exception as e:  # noqa: BLE001
                FAILURES.append(f"describe_run が落ちた v={v} delta={delta}: {e}")
                continue
            check("m/s" in text and "rad" in text,
                  f"読み上げに単位が無い: {text!r}")
            if delta == 0.0:
                check("直進" in text, f"delta=0 が直進と出ない: {text!r}")
            else:
                check("旋回半径" in text, f"旋回半径が出ない: {text!r}")


# ----------------------------------------------------------------------
def main() -> int:
    print("motor_bench の単体テスト")
    print(f"  L={L}m delta_max={DELTA_MAX}rad")

    tests = [
        ("1. motor_driver_core との往復 (delta <-> omega)", test_roundtrip_with_core.raw),
        ("2. 旋回半径と逆算", test_turn_radius.raw),
        ("3. --sweep のパースと delta_max 超過の拒否", test_sweep_parse.raw),
        ("4. PWM カウント入力 (絶対 / 相対 / 範囲外)", test_count_input.raw),
        ("5. 保存する JSON が起動時検証を通る", test_pwm_json_build.raw),
        ("6. 実行前の読み上げ", test_describe_run.raw),
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
