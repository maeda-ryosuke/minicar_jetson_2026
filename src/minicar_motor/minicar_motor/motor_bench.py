#!/usr/bin/env python3
"""実機ベンチツール。motor_driver を単体で校正・検証する。

    ros2 run minicar_motor motor_bench pwm --backend fabo_pca9685 --bus 7
    ros2 run minicar_motor motor_bench cmd --v 0.4 --delta 0.30 --duration 6.0

手順は docs/MOTOR_BENCH.md。サブコマンドは 2 つで、使う順も 2 つの順。

  1. pwm — PCA9685 のカウント 6 点 (left/center/right, back/stop/front) を
           目で見ながら決めて pwm_params.json へ書く。FaBo の
           notebooks/01_find_pwm.ipynb に相当する段。**車輪を浮かせて行う。**
  2. cmd — /cmd_vel へ決め打ちの Twist を決め打ちの秒数だけ流す。
           一定舵で円を描かせて半径 R をメジャーで測り、
           delta = atan(L/R) で実舵角を逆算する (handoff【5-9】ステップ 1)。

=== なぜ 1 が先か ===

u∈[-1,1] -> カウントの変換は 6 点を折れ点に持つ (motor_driver_core.to_count)。
6 点が暫定値のままだと u の基準そのものが動くので、先に舵角マップを測っても
後から全部ずれる。とくに center/stop がずれていると **v=0 を指令しても車が
動く**ので、2 の測定が成立しない。

=== なぜ safety_node を通さないか ===

単体テストなので、加速度制限・スタック脱出・上流死活監視が挟まると
「指令どおりに出なかった」のがドライバのせいか safety のせいか分からなくなる。
このツールは /cmd_vel へ直接 publish する。**裏を返すと加速度制限が無い**ので、
v を急に上げない (--v は 1 回の実行で 1 値、変化はステップ)。

=== 物理的な最後の砦はソフトではない ===

FaBo の基板は RC サーボマルチプレクサでプロポ手動と AI を切り替える。
プロポ側が AI モードでないと何を出しても車は動かず、逆に暴走したらプロポで
切れば止まる。**通電のときは必ずプロポを手元に置く。**
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys

# パッケージとして入っている場合 (ros2 run) と、ファイルを直接叩く場合
# (実機に colcon build 前のソースを置いて試す場合) の両方で動かす。
# 実機で最初に触るツールなので、ここで import に失敗して止まるのが一番困る。
try:
    from minicar_motor.motor_driver_core import MotorConfig
    from minicar_motor.pwm_backend import COUNT_MAX_HW, make_backend
except ImportError:  # pragma: no cover - 実行経路の違いだけ
    from motor_driver_core import MotorConfig
    from pwm_backend import COUNT_MAX_HW, make_backend


# pwm_params.json のキー構成。FaBo の 01_find_pwm.ipynb の出力と同一。
# 変換せずそのまま置けるようにするため、ここで形を変えない。
STEER_KEYS = ("left", "center", "right")
THROTTLE_KEYS = ("back", "stop", "front")


# ======================================================================
# 純粋な計算部分。実機も I2C も ROS も要らない。test_motor_bench.py が叩く。
# ======================================================================

def omega_from_delta(v: float, delta: float, wheelbase: float) -> float:
    """舵角[rad] から角速度[rad/s] を作る。

    omega = v * tan(delta) / L。リポジトリ全体がこの式で一致している
    (motor_driver_core.steer_angle が delta = atan(omega*L/v) で読み戻す)。
    **式を片側だけ変えると、指令した舵角と車輪が向く角が食い違う。**
    test_motor_bench.py の往復テストがここを縛っている。
    """
    if not (math.isfinite(v) and math.isfinite(delta) and wheelbase > 0.0):
        raise ValueError(
            f"v, delta は有限、wheelbase は正であること: "
            f"v={v} delta={delta} L={wheelbase}")
    return v * math.tan(delta) / wheelbase


def turn_radius(delta: float, wheelbase: float) -> float:
    """舵角[rad] から旋回半径[m]。直進は inf を返す。

    メジャーで測るのはこの R。delta=0 で 0 除算しないよう inf にする
    (表示側が inf を見て「直進」と出す)。
    """
    t = math.tan(delta)
    if abs(t) < 1e-9:
        return math.inf
    return abs(wheelbase / t)


def delta_from_radius(radius: float, wheelbase: float) -> float:
    """測った旋回半径[m] から実舵角[rad]。handoff【5-9】ステップ 1 の式。

    測定結果を steer_map へ写すときに使う。ツール自身は使わないが、
    現場で電卓代わりに呼べるよう cmd の表示に出している。
    """
    if not (radius > 0.0 and math.isfinite(radius)):
        raise ValueError(f"旋回半径は正の有限値であること: {radius}")
    return math.atan(wheelbase / radius)


def parse_sweep(text: str, delta_max: float) -> list[float]:
    """"0.42,0.30,0,-0.30" -> [0.42, 0.30, 0.0, -0.30]。

    順序を入れ替えない。測定は大舵角から小舵角へ順に降りるなど、
    現場の都合で並べた順に意味がある (チョークの円を内側から外側へ描く)。

    delta_max を超える値はここで落とす。**サーボを機械端へ突っ込ませない**
    ための門で、motor_driver 側のクランプに頼らない。クランプに頼ると
    「0.6 を指令したのに 0.42 の円が描かれた」データを測ってしまう。
    """
    items = [s.strip() for s in str(text).split(",")]
    items = [s for s in items if s]
    if not items:
        raise ValueError("--sweep が空。カンマ区切りで舵角[rad]を並べること")
    out = []
    for s in items:
        try:
            d = float(s)
        except ValueError as e:
            raise ValueError(f"--sweep に数値でない要素: {s!r}") from e
        if not math.isfinite(d):
            raise ValueError(f"--sweep に有限でない要素: {s!r}")
        if abs(d) > delta_max + 1e-9:
            raise ValueError(
                f"--sweep の {d} が delta_max=±{delta_max} を超えている。"
                "サーボが機械端に当たるので実行しない")
        out.append(d)
    return out


def parse_count_input(text: str, current: int) -> int:
    """PWM 校正の 1 行入力を解釈する。"420" は絶対、"+10" / "-5" は相対。

    相対を持たせているのは、中立探しが「今の値から少しずつずらして、
    サーボが動き出す / 車が動き出す境目を見る」作業だから。絶対値だけだと
    毎回現在値を足し算することになり、暗算ミスがそのまま出力になる。

    範囲チェックはここでする。pwm_backend._check() も同じ検証をするが、
    あちらは例外で、こちらは「入力し直させる」ために ValueError を投げる。
    """
    s = str(text).strip()
    if not s:
        raise ValueError("空の入力")
    relative = s[0] in "+-"
    try:
        value = int(round(float(s)))
    except ValueError as e:
        raise ValueError(f"数値として読めない: {s!r}") from e
    count = current + value if relative else value
    if not (0 <= count <= COUNT_MAX_HW):
        raise ValueError(
            f"カウント {count} が 12bit の範囲外 (0..{COUNT_MAX_HW})")
    return count


def build_pwm_json(steer: dict, throttle: dict) -> dict:
    """記録した 6 点を pwm_params.json の形にして検証する。

    **不正な JSON を保存させないのがここの仕事。** 保存してしまうと、
    次に motor_driver を起動したときに初めて落ちる (あるいは落ちずに
    中立が端点にずれたまま走る)。それは実機に通電した後なので高くつく。

    検証は MotorConfig.validate() に委ねる。ドライバ本体が起動時に通すのと
    同じ検証をここで通しておけば、「保存できたのに起動できない」が起きない。
    """
    missing = ([k for k in STEER_KEYS if k not in steer]
               + [k for k in THROTTLE_KEYS if k not in throttle])
    if missing:
        raise ValueError(f"まだ記録していない点がある: {', '.join(missing)}")

    data = {
        "pwm_steering": {k: int(steer[k]) for k in STEER_KEYS},
        "pwm_speed": {k: int(throttle[k]) for k in THROTTLE_KEYS},
    }
    # ドライバ本体と同じ検証。中立が端点の内側にあるか、端点が同値でないか。
    # 車両諸元は既定のままでよい (ここで見たいのは PWM 6 点だけ)。
    MotorConfig.from_yaml(
        {"vehicle": {"wheelbase": 0.257},
         "limits": {"delta_max": 0.42, "v_min": -1.0, "v_max": 2.0}},
        {}, data)
    return data


def load_wheelbase(path: str) -> tuple[float, float]:
    """vehicle_params.yaml から (wheelbase[m], delta_max[rad])。

    ここへ値を書き写さないのは AGENTS.md の「同一の車両諸元を複数ファイルへ
    コピーしない」。既定値を持たせないのも同じ理由で、決め打ちにすると
    「別の諸元で測っているつもりで既定を読んでいた」に気付けない。
    """
    import yaml
    with open(path) as f:
        vp = yaml.safe_load(f)
    return float(vp["vehicle"]["wheelbase"]), float(vp["limits"]["delta_max"])


def describe_run(v: float, delta: float, duration: float,
                 wheelbase: float) -> str:
    """実行前に読み上げる 1 ブロック。**押す前に何が起きるか分かること。**

    距離を出しているのは、接地で走らせるときに必要な空間を先に知るため。
    「6 秒くらいなら大丈夫だろう」で v=1.5 を出すと 9m 進む。
    """
    omega = omega_from_delta(v, delta, wheelbase)
    dist = abs(v) * duration
    r = turn_radius(delta, wheelbase)
    lines = [
        f"  v     = {v:+.2f} m/s",
        f"  delta = {delta:+.3f} rad ({math.degrees(delta):+.1f} deg)",
        f"  omega = {omega:+.3f} rad/s",
        f"  時間  = {duration:.1f} s",
    ]
    if math.isinf(r):
        lines.append(f"  -> 直進。走行距離 {dist:.2f} m")
    else:
        circ = 2.0 * math.pi * r
        lines.append(
            f"  -> 旋回半径 R = {r:.2f} m (直径 {2 * r:.2f} m / "
            f"円周 {circ:.2f} m)")
        lines.append(
            f"  -> 走行距離 {dist:.2f} m = {dist / circ:.2f} 周")
    lines.append(
        f"  車輪を接地させる場合、{max(dist, 2 * r if math.isfinite(r) else dist):.1f} m "
        "四方の空間と、即時に電源を切れる体勢が要る")
    return "\n".join(lines)


# ======================================================================
# サブコマンド: pwm
# ======================================================================

PWM_HELP = """\
--- コマンド -----------------------------------------------------------
  s / t          操舵チャネル / スロットルチャネルへ切替
  420            そのカウントを書く
  +10  -5        現在値からの相対で書く
  left center right    今のカウントを操舵の3点として記録
  back stop front      今のカウントをスロットルの3点として記録
  show           記録済みの点と現在値を表示
  save           pwm_params.json へ書き出す (既存は .bak へ退避)
  q              中立を書いて終了
------------------------------------------------------------------------
測り方 (FaBo notebooks/01_find_pwm.ipynb と同じ):
  操舵   中立 -> 左端 -> 右端 の順。**機械端に当てない。**
         タイヤが止まったのにサーボが唸る位置は行き過ぎ。1 段戻して記録する
  速度   stop は「車輪が回り出さない上限」ではなく「完全に止まる値」。
         front / back は測定に使う低速側でよい (全開を記録する必要は無い)
"""


def cmd_pwm(args: argparse.Namespace) -> int:
    """PWM 6 点を対話で決めて JSON に書く。

    1 行入力 + Enter 方式にしてあるのは、teleop.sh が書いているとおり
    termios の raw 入力が ExecuteProcess 配下や docker exec で TTY に
    ならないと届かないため。行入力なら経路を選ばない。
    """
    existing = {}
    if os.path.exists(args.out):
        with open(args.out) as f:
            existing = json.load(f)
    st = existing.get("pwm_steering", {})
    sp = existing.get("pwm_speed", {})
    # 起動時に書く中立。既存 JSON の実測中立があればそれを使う。
    # 無ければ PCA9685 の中央付近 (FaBo の初期値と同じ 410)。
    neutral = {"steer": int(st.get("center", 410)),
               "throttle": int(sp.get("stop", 410))}

    backend = make_backend(args.backend, bus=args.bus, address=args.address,
                           freq_hz=args.freq, initial_value=neutral["throttle"])
    print(f"backend={backend.name} freq={args.freq}Hz "
          f"ch steer={args.steering_channel} throttle={args.throttle_channel}")
    if backend.name == "dryrun":
        print("*** dryrun。PWM は出ていない。実機では --backend fabo_pca9685 ***")
    print(f"出力先: {args.out}"
          + (" (既存あり。save で .bak へ退避してから上書き)" if existing else ""))

    channel = {"steer": args.steering_channel,
               "throttle": args.throttle_channel}
    recorded = {"steer": dict(st), "throttle": dict(sp)}
    current = dict(neutral)
    kind = "steer"
    throttle_confirmed = False

    def write(k: str, count: int) -> None:
        backend.set_count(channel[k], count)
        current[k] = count

    try:
        # 両チャネルへ中立を書いてから始める。片方を放置すると、前回の
        # セッションで書いた値を PCA9685 が保持したままになる。
        write("steer", neutral["steer"])
        write("throttle", neutral["throttle"])
        print(f"\n中立を書いた: steer={neutral['steer']} "
              f"throttle={neutral['throttle']}")
        print(PWM_HELP)

        while True:
            keys = STEER_KEYS if kind == "steer" else THROTTLE_KEYS
            done = [k for k in keys if k in recorded[kind]]
            prompt = (f"[{kind} ch{channel[kind]}] count={current[kind]} "
                      f"記録済み={','.join(done) if done else '-'} > ")
            try:
                line = input(prompt).strip()
            except EOFError:
                print()
                break
            if not line:
                continue
            low = line.lower()

            if low == "q":
                break
            if low == "s":
                kind = "steer"
                continue
            if low == "t":
                if not throttle_confirmed:
                    # 一度だけ聞く。毎回聞くと惰性で y を押すようになる。
                    ans = input(
                        "  スロットルを触る。車輪は浮いているか? [y/N] ").strip()
                    if ans.lower() != "y":
                        print("  中止。車輪を浮かせてから t を入れること")
                        continue
                    throttle_confirmed = True
                kind = "throttle"
                continue
            if low == "show":
                print(f"  steer    {recorded['steer']}")
                print(f"  throttle {recorded['throttle']}")
                print(f"  現在値   steer={current['steer']} "
                      f"throttle={current['throttle']}")
                continue
            if low in ("help", "?", "h"):
                print(PWM_HELP)
                continue
            if low == "save":
                try:
                    data = build_pwm_json(recorded["steer"], recorded["throttle"])
                except ValueError as e:
                    # 保存できない理由をその場で出して、入力へ戻る。
                    # ここで落とさないのは、セッション中の記録を失わないため。
                    print(f"  保存しない: {e}")
                    continue
                if os.path.exists(args.out):
                    shutil.copyfile(args.out, args.out + ".bak")
                    print(f"  既存を {args.out}.bak へ退避した")
                with open(args.out, "w") as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)
                    f.write("\n")
                print(f"  保存した: {json.dumps(data, ensure_ascii=False)}")
                continue
            if low in keys:
                recorded[kind][low] = current[kind]
                print(f"  {kind}.{low} = {current[kind]} を記録")
                continue
            if low in (STEER_KEYS + THROTTLE_KEYS):
                print(f"  {low} は今のチャネル({kind})の点ではない。"
                      f"{'s' if low in STEER_KEYS else 't'} で切り替えること")
                continue

            try:
                count = parse_count_input(line, current[kind])
            except ValueError as e:
                print(f"  {e}")
                continue
            write(kind, count)

    except KeyboardInterrupt:
        print()
    finally:
        # 落ちるときこそ中立を書く。PCA9685 は最後のデューティを保持するので、
        # 書かずに終わると舵を切ったまま / 走ったままになる。
        try:
            backend.set_count(channel["steer"], neutral["steer"])
            backend.set_count(channel["throttle"], neutral["throttle"])
            print(f"中立を書いて終了: steer={neutral['steer']} "
                  f"throttle={neutral['throttle']}")
        except Exception as e:  # noqa: BLE001 - 終了経路で握り潰す
            print(f"!!! 中立書き込みに失敗: {e}  電源を切ること")
        try:
            backend.close()
        except Exception:
            pass
    return 0


# ======================================================================
# サブコマンド: cmd
# ======================================================================

def cmd_cmd(args: argparse.Namespace) -> int:
    """/cmd_vel へ決め打ちの Twist を決め打ちの秒数だけ流す。

    rclpy の import を関数内まで遅らせてあるのは、pwm サブコマンドを
    ROS 無しで動かせるようにするため (実機で最初に触るのが pwm 側)。

    **引数の検証を rclpy より先に済ませる。** --sweep の誤記や delta_max
    超過に気付くのが ROS 起動後だと、その頃には車の前に人が立っている。
    """
    wheelbase, delta_max = load_wheelbase(args.vehicle_params_file)

    # 舵角の出し方は 3 通り。deg 指定を持たせているのは、現場でメジャーと
    # 分度器を持っているときに rad へ暗算し直さずに済ませるため。
    if args.sweep:
        deltas = parse_sweep(args.sweep, delta_max)
    elif args.omega is not None:
        if abs(args.v) < 1e-9:
            raise SystemExit(
                "--omega 指定には v が要る。v=0 では delta = atan(omega*L/v) が "
                "定義できない (motor_driver は直前の舵角を保持する)")
        deltas = [math.atan(args.omega * wheelbase / args.v)]
        if abs(deltas[0]) > delta_max + 1e-9:
            raise SystemExit(
                f"--omega {args.omega} は舵角 {deltas[0]:.3f} rad に相当し、"
                f"delta_max=±{delta_max} を超える")
    else:
        d = (math.radians(args.delta_deg) if args.delta_deg is not None
             else args.delta)
        deltas = parse_sweep(str(d), delta_max)

    import time

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    rclpy.init()
    node = Node("motor_bench")
    pub = node.create_publisher(Twist, args.cmd_topic, 10)

    # 観測トピックを購読して、実際に出た u_steer を画面へ出す。
    # 測定シートに書くのは指令 delta ではなく **u**。マップは u で引くため。
    latest = {"delta": None, "u": None}
    try:
        from std_msgs.msg import Float64, Float64MultiArray
        node.create_subscription(
            Float64, "/motor/steer_angle",
            lambda m: latest.__setitem__("delta", m.data), 10)
        node.create_subscription(
            Float64MultiArray, "/motor/cmd_normalized",
            lambda m: latest.__setitem__("u", list(m.data)), 10)
    except ImportError:  # pragma: no cover
        pass

    period = 1.0 / args.rate
    zero = Twist()

    def spin_publish(msg: Twist, seconds: float, label: str) -> None:
        """seconds の間 msg を rate で publish し続ける。

        publish し続けるのは motor_driver の cmd_timeout (既定 0.3s) を
        満たすため。1 回投げて待つと、0.3s 後に中立へ落ちる。
        """
        t_end = time.monotonic() + seconds
        last_print = 0.0
        while rclpy.ok() and time.monotonic() < t_end:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            if now - last_print >= 0.5:
                last_print = now
                remain = t_end - now
                obs = ""
                if latest["delta"] is not None:
                    obs += f" 実舵角 {latest['delta']:+.3f}rad"
                if latest["u"] is not None and len(latest["u"]) >= 2:
                    obs += (f" u=[{latest['u'][0]:+.3f}, "
                            f"{latest['u'][1]:+.3f}]")
                print(f"  {label} 残り {remain:4.1f}s{obs}", flush=True)
            time.sleep(period)

    def stop() -> None:
        """ゼロ Twist を 0.5 秒出す。**1 回では足りない。**

        motor_driver は最後に受けた指令を cmd_timeout まで保持し、
        publish を止めただけでは ESC が最後のパルスをラッチし続ける。
        取りこぼしを見越して、明示的なゼロを timeout より長く出す。
        """
        spin_publish(zero, 0.5, "停止指令")

    try:
        # 他に /cmd_vel を出している者がいないか数える。safety_node や
        # nav2 が上がっていると publisher が複数になって殴り合う
        # (teleop.sh が同じ理由で警告している)。
        time.sleep(0.5)          # discovery を待つ
        n = node.count_publishers(args.cmd_topic)
        if n > 1:
            print(f"!!! {args.cmd_topic} の publisher が {n} 個ある。"
                  "safety_node / nav2 / teleop を止めること")
            print("!!! 指令が混ざると測定値が意味を持たない")
            if not args.yes and input("続行するか? [y/N] ").strip().lower() != "y":
                return 1

        for i, delta in enumerate(deltas):
            print(f"\n=== {i + 1}/{len(deltas)} ===")
            print(describe_run(args.v, delta, args.duration, wheelbase))
            r = turn_radius(delta, wheelbase)
            if math.isfinite(r):
                print(f"  測ったら: delta_actual = atan({wheelbase:.3f}/R_measured)")
            if not args.yes:
                try:
                    if input("  Enter で実行 (s でスキップ, q で中止) > "
                             ).strip().lower() in ("q", "quit"):
                        break
                except EOFError:
                    break

            msg = Twist()
            msg.linear.x = float(args.v)
            msg.angular.z = float(omega_from_delta(args.v, delta, wheelbase))
            spin_publish(msg, args.duration, f"delta={delta:+.3f}")
            stop()
            if latest["u"] is not None and len(latest["u"]) >= 2:
                print(f"  記録用: delta_cmd={delta:+.4f} "
                      f"u_steer={latest['u'][0]:+.4f} "
                      f"u_throttle={latest['u'][1]:+.4f}  R_measured=____ m")
            else:
                print(f"  記録用: delta_cmd={delta:+.4f}  R_measured=____ m")
                print("  (u が出ていない。motor_driver が起動していないか "
                      "/motor/cmd_normalized が届いていない)")

    except KeyboardInterrupt:
        print("\n中断。停止指令を出す")
    finally:
        try:
            stop()
        except Exception as e:  # noqa: BLE001 - 終了経路で握り潰す
            print(f"!!! 停止指令に失敗: {e}  プロポか電源で止めること")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


# ======================================================================

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="motor_bench",
        description="モータドライバの実機ベンチ。手順は docs/MOTOR_BENCH.md",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="sub", required=True)

    q = sub.add_parser("pwm", help="PWM 6 点を対話で校正して JSON に書く")
    q.add_argument("--backend", default="dryrun",
                   help="dryrun / fabo_pca9685 (既定 dryrun)")
    q.add_argument("--bus", type=int, default=7,
                   help="I2C バス。Orin Nano は 7 (既定 7)")
    q.add_argument("--address", type=lambda s: int(s, 0), default=0x40,
                   help="PCA9685 の I2C アドレス (既定 0x40)")
    q.add_argument("--freq", type=float, default=60.0,
                   help="PWM 周波数[Hz]。FaBo の実走 Notebook は 60 (既定 60)")
    q.add_argument("--steering-channel", type=int, default=0)
    q.add_argument("--throttle-channel", type=int, default=1)
    q.add_argument("--out", default="pwm_params.json",
                   help="書き出す JSON (既定 ./pwm_params.json)")
    q.set_defaults(func=cmd_pwm)

    c = sub.add_parser("cmd", help="/cmd_vel へ決め打ちの Twist を流す")
    c.add_argument("--v", type=float, default=0.4, help="速度[m/s] (既定 0.4)")
    g = c.add_mutually_exclusive_group()
    g.add_argument("--delta", type=float, default=0.0, help="舵角[rad]")
    g.add_argument("--delta-deg", type=float, default=None, help="舵角[deg]")
    g.add_argument("--omega", type=float, default=None, help="角速度[rad/s]")
    g.add_argument("--sweep", default=None,
                   help="舵角[rad]をカンマ区切りで並べ、1 点ずつ順に実行")
    c.add_argument("--duration", type=float, default=5.0,
                   help="1 点あたりの実行時間[s] (既定 5.0)")
    c.add_argument("--rate", type=float, default=50.0,
                   help="publish 周期[Hz] (既定 50)")
    c.add_argument("--cmd-topic", default="/cmd_vel")
    c.add_argument("--vehicle-params-file", required=True,
                   help="車両諸元 YAML。wheelbase と delta_max の出所")
    c.add_argument("--yes", action="store_true",
                   help="実行前の確認を省く。接地状態では使わない")
    c.set_defaults(func=cmd_cmd)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except (ValueError, FileNotFoundError, KeyError) as e:
        # 設定ミスをトレースバックで出さない。現場で読むのは 1 行目だけで、
        # スタックが出ると「ツールが壊れた」のか「入力が悪い」のか分からない。
        print(f"エラー: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
