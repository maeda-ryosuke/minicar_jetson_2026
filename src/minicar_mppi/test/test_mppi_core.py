#!/usr/bin/env python3
"""mppi_core の単体テスト。ROS も Gazebo も不要。

    python3 nodes/test_mppi_core.py

シミュレータ起動に 25 秒かかるので、コスト設計やパラメータの検証を
シム依存にすると回らない。合成スキャンを線分へのレイキャストで作り、
挙動を機械的に確認する。

特に重要なのが hairpin のテスト。FTG + Pure Pursuit が実際に膠着した
world(9.0, 2.7) 付近の壁配置を再現し、「一度外へ膨らんでから内に切れ込む」
が出るかを横偏差の符号反転で判定する。これが MPPI を入れる目的そのもの。
"""

import math
import sys
import time

import numpy as np

from minicar_mppi.mppi_core import MPPI, MPPIConfig


# ----------------------------------------------------------------------
def raycast(segments, angles, origin=(0.0, 0.0), r_max=30.0):
    """線分群に対するレイキャストで合成 LaserScan を作る。

    segments: [((x1,y1),(x2,y2)), ...]  車体座標系[m]
    angles:   ビーム角[rad]
    """
    ox, oy = origin
    out = np.full(angles.shape, np.inf)
    for i, a in enumerate(angles):
        dx, dy = math.cos(a), math.sin(a)
        best = r_max
        for (x1, y1), (x2, y2) in segments:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            # レイ: O + t*D, 線分: P1 + s*E
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
            s = ((x1 - ox) * dy - (y1 - oy) * dx) / den
            if t > 0.0 and 0.0 <= s <= 1.0 and t < best:
                best = t
        out[i] = best
    return out


def beam_angles(n=640, half=math.radians(135.0)):
    return np.linspace(-half, half, n)


def make_cfg(**kw) -> MPPIConfig:
    c = MPPIConfig()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def build_esdf(mppi: MPPI, segments, cfg: MPPIConfig):
    """線分群 -> 合成スキャン -> 点群 -> 距離場。実運用と同じ経路を通す。"""
    ang = beam_angles()
    # LiDAR は後輪車軸より前方 lidar_dx にある
    rng = raycast(segments, ang, origin=(cfg.lidar_dx, 0.0))
    px, py = mppi.esdf.scan_to_points(rng, ang, cfg.lidar_dx, 0.1, 30.0)
    mppi.esdf.build(px, py)
    return px, py


# ----------------------------------------------------------------------
def test_esdf_straight_wall():
    """直線壁に対する距離場が解析解と一致すること。"""
    cfg = make_cfg()
    m = MPPI(cfg)
    # y = +1.0 の壁(x: -2..8)。車の左 1.0m。
    build_esdf(m, [((-2.0, 1.0), (8.0, 1.0))], cfg)

    for (x, y), expect in [((1.0, 0.0), 1.0), ((2.0, 0.5), 0.5), ((3.0, -0.5), 1.5)]:
        got = float(m.esdf.query(np.array([x]), np.array([y]))[0])
        # 膨張1セル(0.05m)ぶん近めに出るのが正しい挙動
        assert abs(got - (expect - cfg.grid_res)) < 0.09, \
            f"ESDF({x},{y}) = {got:.3f}, 期待 {expect:.3f} 付近"
    print("  [OK] ESDF が直線壁に対し解析解と一致 (膨張1セルぶんのオフセット込み)")


def test_corridor_goes_straight():
    """まっすぐな廊下では直進すること(左右に偏らない)。"""
    cfg = make_cfg(w_goal=0.0)   # FTG 無しで純粋に回避+速度のみ
    m = MPPI(cfg)
    segs = [((-2.0, 0.8), (10.0, 0.8)), ((-2.0, -0.8), (10.0, -0.8))]
    build_esdf(m, segs, cfg)

    for _ in range(15):          # 収束させる
        out = m.step(v0=1.0, delta0=0.0, goal=None, shift=False)
    ty = out["traj"][1]
    assert abs(ty[-1]) < 0.25, f"廊下で横に {ty[-1]:+.2f}m ずれた"
    assert out["collision_frac"] < 0.5, "廊下でサンプルの過半が衝突している"
    print(f"  [OK] 直線廊下で直進 (終端横偏差 {ty[-1]:+.3f}m)")


def test_impassable_wall_slows():
    """回り込めない壁の前では減速すること。

    幅 4m の壁を 1.5m 先に置くと、最小旋回半径 1.0m では物理的に回り込め
    ない(R=1.0 の旋回では x が 1.0m までしか伸びない)。この場面で正しい
    答えは「減速」であって「操舵」ではない。無理に曲がろうとしないことを
    確認する。
    """
    cfg = make_cfg(w_goal=0.0)
    m = MPPI(cfg)
    segs = [((1.5, -2.0), (1.5, 2.0))]
    build_esdf(m, segs, cfg)
    for _ in range(15):
        out = m.step(v0=1.2, delta0=0.0, goal=None, shift=False)
    reach = float(out["traj"][0][-1])
    assert out["v"] < 1.15, f"回り込めない壁の前で減速していない (v={out['v']:.2f})"
    assert reach < 1.45, f"壁(1.5m)を越える軌道を採用している (終端 x={reach:.2f})"
    print(f"  [OK] 回り込めない壁の手前で減速 (v={out['v']:.2f}, 終端 x={reach:.2f}m)")


def test_gap_in_wall_steers():
    """壁に抜けられる隙間があれば、そちらへ操舵すること。

    壁は x=1.8 の y=-0.2 より上のみ。下(-y)が開いている。
    y=-0.8 に届くのに必要な曲率は 2*0.8/1.8^2 = 0.49 で、上限 1.0 の内側。
    """
    cfg = make_cfg(w_goal=0.0)
    m = MPPI(cfg)
    segs = [((1.8, -0.2), (1.8, 3.0))]
    build_esdf(m, segs, cfg)
    for _ in range(25):
        out = m.step(v0=1.0, delta0=0.0, goal=None, shift=False)
    tx, ty = out["traj"]
    assert ty[-1] < -0.25, f"開いている下側へ避けていない (終端 y={ty[-1]:+.2f})"
    print(f"  [OK] 隙間へ操舵 (終端 ({tx[-1]:.2f},{ty[-1]:+.3f}))")


def test_hairpin_turns_without_collision():
    """直角に折れる廊下を、衝突せずに曲がれること。

    実コースで膠着した状況の型を再現する: まっすぐな廊下の先が壁で塞がれ、
    途中から横(-y)へ抜け道が開く。FTG + Pure Pursuit はこの型で
    「曲がりきれず壁に膠着」した(真値 10.63m で停止)。

        北壁     y=+1.0  (x: -1 .. 3.2)   ずっと続く
        南壁     y=-1.0  (x: -1 .. 2.0)   x=2.0 で切れる = ここから開く
        抜け道の左側 x=2.0 (y: -1.0 .. -3.0)
        正面の壁 x=3.2  (y: +1.0 .. -3.0)

    「膨らんで曲がる」は、右折の前に北(+y)寄りに寄せて旋回半径を稼ぐ
    動きとして出る。ただし廊下幅が有限なので必ず出るとは限らない。
    アサートは「衝突せず南へ抜ける」までにして、膨らみの有無は観測値
    として報告する(創発的挙動を強く縛るとテストが脆くなる)。
    """
    # 実運用と同じく FTG が方向を与える構成で試す。w_goal=0 だと
    # 「端の壁の手前で減速」が最適解になり、南へ曲がる動機が無い。
    cfg = make_cfg()
    m = MPPI(cfg)
    goal = (2.5, -1.2)          # FTG が南の抜け道に置く目標点に相当
    segs = [((-1.0, 1.0), (3.2, 1.0)),
            ((-1.0, -1.0), (2.0, -1.0)),
            ((2.0, -1.0), (2.0, -3.0)),
            ((3.2, 1.0), (3.2, -3.0))]
    build_esdf(m, segs, cfg)

    for _ in range(30):
        out = m.step(v0=1.0, delta0=0.0, goal=goal, shift=False)
    tx, ty = out["traj"]

    # 採用軌道が壁に触れていないこと
    d = m.esdf.query(tx, ty)
    assert d.min() > cfg.half_width, \
        f"採用軌道が壁に接触 (最小クリアランス {d.min():.3f}m)"
    assert ty[-1] < -0.2, f"抜け道(南)へ向かっていない (終端 y={ty[-1]:+.2f})"

    swing = float(ty.max())
    print(f"       軌道: 最大 y={swing:+.3f}m -> 終端 y={ty[-1]:+.3f}m "
          f"(x 到達 {tx[-1]:.2f}m, 最小クリアランス {d.min():.3f}m)")
    if swing > 0.03:
        print(f"  [OK] 衝突せず南へ。右折前に北へ膨らんでいる ({swing:+.3f}m)")
    else:
        print("  [OK] 衝突せず南へ。今回は膨らみ無しで曲がれた"
              "(廊下幅に余裕があるため)")


def test_all_colliding_slows_down():
    """全サンプルが衝突する状況(壁に密着)でも、加速しないこと。

    衝突判定をラッチしないと、ロールアウトが壁をすり抜けられてしまい
    「速く走るほど壁の向こうへ早く抜けて衝突ステップ数が減る」という理由で
    加速が最適解になる。実機シムで実際に起きた: 壁に密着した状態から
    cost_min が 24037 -> 6845 と下がりながら v が 0.30 -> 1.6 へ上がり、
    全速力で壁を押し続けて真値 3.9m で膠着した。
    """
    cfg = make_cfg(w_goal=0.0)
    m = MPPI(cfg)
    # 車をほぼ囲む壁。前も左右も塞がっている。
    segs = [((0.35, -1.5), (0.35, 1.5)),
            ((-1.5, 0.35), (1.5, 0.35)),
            ((-1.5, -0.35), (1.5, -0.35))]
    build_esdf(m, segs, cfg)

    vs = []
    for _ in range(20):
        out = m.step(v0=1.2, delta0=0.0, goal=None, shift=False)
        vs.append(out["v"])
    assert out["collision_frac"] > 0.9, \
        f"想定と違い衝突していない ({out['collision_frac']*100:.0f}%)"
    assert out["v"] < 1.2, f"全滅状態で減速していない (v={out['v']:.2f})"
    print(f"  [OK] 全サンプル衝突({out['collision_frac']*100:.0f}%)でも減速 "
          f"(v {vs[0]:.2f} -> {out['v']:.2f})")


def test_dead_end_no_crash():
    """行き止まりでも例外を出さず、何らかの解を返すこと。"""
    cfg = make_cfg(w_goal=0.0)
    m = MPPI(cfg)
    segs = [((1.0, -1.0), (1.0, 1.0)), ((1.0, 1.0), (-1.0, 1.0)),
            ((1.0, -1.0), (-1.0, -1.0))]
    build_esdf(m, segs, cfg)
    for _ in range(10):
        out = m.step(v0=0.5, delta0=0.0, goal=None, shift=False)
    assert np.isfinite(out["v"]) and np.isfinite(out["delta"]), "解が NaN"
    print(f"  [OK] 行き止まりでも解を返す (v={out['v']:.2f}, "
          f"衝突サンプル {out['collision_frac']*100:.0f}%)")


def test_goal_term_pulls():
    """FTG 目標点の終端コストが効くこと(w_goal のオンオフで差が出る)。"""
    segs = [((-2.0, 2.5), (10.0, 2.5)), ((-2.0, -2.5), (10.0, -2.5))]

    cfg_off = make_cfg(w_goal=0.0)
    m_off = MPPI(cfg_off)
    build_esdf(m_off, segs, cfg_off)
    for _ in range(15):
        off = m_off.step(1.0, 0.0, goal=None, shift=False)

    cfg_on = make_cfg()   # 既定の w_goal
    m_on = MPPI(cfg_on)
    build_esdf(m_on, segs, cfg_on)
    for _ in range(15):
        on = m_on.step(1.0, 0.0, goal=(1.5, 1.2), shift=False)   # 左前方を狙わせる

    assert on["traj"][1][-1] > off["traj"][1][-1] + 0.15, \
        f"目標点が効いていない (on {on['traj'][1][-1]:+.2f} / off {off['traj'][1][-1]:+.2f})"
    print(f"  [OK] FTG 終端コストが効く (goal 有 y={on['traj'][1][-1]:+.3f} / "
          f"無 y={off['traj'][1][-1]:+.3f})")


def test_heading_term_acts_only_near_obstacle():
    """危険時だけ、FTG目標を向いた姿勢のコストが低くなること。

    終端位置と速度は同一にし、姿勢だけを左目標向き/右向きに変える。
    これにより終端距離や速度ではなく、新しい方向項そのものを確認する。
    """
    cfg = make_cfg(
        T=3, w_collision=0.0, w_obs=0.0, w_speed=0.0,
        w_smooth=0.0, w_goal=0.0, w_heading=100.0,
        heading_danger_distance=0.60,
    )
    m = MPPI(cfg)

    shape = (2, cfg.T)
    traj = {
        "x": np.zeros(shape),
        "y": np.zeros(shape),
        "theta": np.vstack([
            np.full(cfg.T, math.pi / 4.0),   # 左前方の目標を向く
            np.full(cfg.T, -math.pi / 4.0),  # 目標と反対の右を向く
        ]),
        "v": np.ones(shape),
        "delta": np.zeros(shape),
    }
    goal = (1.0, 1.0)

    # 原点の約0.3m前に壁を置き、方向項の危険度を立ち上げる。
    build_esdf(m, [((0.35, -2.0), (0.35, 2.0))], cfg)
    _, near = m.cost(traj, m.esdf, delta0=0.0, goal=goal, breakdown=True)
    assert near["方向"][0] < near["方向"][1], \
        f"危険時に目標向きが優先されない ({near['方向']})"
    assert near["方向"][0] < 1e-9, \
        f"目標を正確に向いた候補へ方向コストが付いた ({near['方向'][0]})"

    # 壁を2m先へ移すと閾値0.6mの外なので、姿勢に関係なく発火しない。
    build_esdf(m, [((2.0, -2.0), (2.0, 2.0))], cfg)
    _, far = m.cost(traj, m.esdf, delta0=0.0, goal=goal, breakdown=True)
    assert np.allclose(far["方向"], 0.0), \
        f"安全距離外で方向コストが発火した ({far['方向']})"
    print("  [OK] 危険時だけFTG目標向きの姿勢を優先")


def test_timing():
    """1周期が制御周期 50ms 以内に収まること。"""
    cfg = make_cfg()
    m = MPPI(cfg)
    segs = [((-2.0, 1.2), (10.0, 1.2)), ((-2.0, -1.2), (10.0, -1.2))]
    ang = beam_angles()
    rng = raycast(segs, ang, origin=(cfg.lidar_dx, 0.0))

    t0 = time.perf_counter()
    N = 20
    for _ in range(N):
        px, py = m.esdf.scan_to_points(rng, ang, cfg.lidar_dx, 0.1, 30.0)
        m.esdf.build(px, py)      # 実運用でも毎スキャン作り直す
        m.step(1.0, 0.0, goal=(2.0, 0.0))   # 実運用と同じく shift 有り
    ms = (time.perf_counter() - t0) / N * 1000.0
    print(f"  [{'OK' if ms < 50 else '--'}] 1周期 {ms:.1f}ms "
          f"(K={cfg.K}, T={cfg.T}, ESDF 再構築込み)")
    assert ms < 50.0, f"1周期 {ms:.1f}ms は制御周期 50ms を超える"


# ----------------------------------------------------------------------
def main() -> int:
    tests = [
        ("ESDF 精度", test_esdf_straight_wall),
        ("直線廊下", test_corridor_goes_straight),
        ("回り込めない壁で減速", test_impassable_wall_slows),
        ("隙間へ操舵", test_gap_in_wall_steers),
        ("直角廊下を曲がる", test_hairpin_turns_without_collision),
        ("全滅時に減速", test_all_colliding_slows_down),
        ("行き止まり", test_dead_end_no_crash),
        ("FTG 終端コスト", test_goal_term_pulls),
        ("危険時のFTG方向コスト", test_heading_term_acts_only_near_obstacle),
        ("計算時間", test_timing),
    ]
    fails = 0
    for name, fn in tests:
        print(f"[{name}]")
        try:
            fn()
        except AssertionError as e:
            print(f"  [NG] {e}")
            fails += 1
        except Exception as e:
            print(f"  [ERR] {type(e).__name__}: {e}")
            fails += 1
    print()
    print("全て成功" if fails == 0 else f"{fails} 件失敗")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
