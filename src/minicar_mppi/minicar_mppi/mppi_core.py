#!/usr/bin/env python3
"""MPPI のコア — ROS を一切使わない純 numpy 実装。

ROS から切り離してあるのは、合成スキャンを与えてシミュレータ抜きで挙動を
確定できるようにするため。Gazebo 起動に 25 秒かかるので、チューニングの
たびにシムを立てていると回らない。単体テストは test_mppi_core.py。

構成:
    LocalESDF     スキャン点群 -> 局所距離場(最近傍障害物までの距離)
    VehicleModel  自転車モデル + 操舵一次遅れ。K 本を同時に進める
    MPPICost      コスト項
    MPPI          サンプリング・重み付け・warm start

座標系は全て「後輪車軸原点・車体前方が +x」。自己位置推定は使わない。
毎周期その瞬間のスキャンから作り直す(receding horizon)ので、
積算された姿勢は必要ない。
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt


# ----------------------------------------------------------------------
@dataclass
class MPPIConfig:
    # --- 車両諸元 (vehicle_params.yaml から入れる) ---
    wheelbase: float = 0.257
    half_width: float = 0.095      # タイヤ外側まで。ボディ箱 0.075 ではない
    lidar_dx: float = 0.2285       # 後輪車軸から見た LiDAR の前方オフセット
    v_min: float = 0.3             # 前進サンプリングの下限。0 にすると
    v_max: float = 2.0             # 「止まって向きを変える」実行不能解が出る
    a_max: float = 2.0
    delta_max: float = 0.2516
    steer_tau: float = 0.067       # 操舵一次遅れ[s]。実機で要校正

    # --- ホライゾンとサンプリング ---
    K: int = 800                   # サンプル数
    T: int = 30                    # ステップ数
    dt: float = 0.06               # 1ステップ[s]。T*dt = 1.8s ≒ 2.1m @1.2m/s
    # 温度。コストのサンプル間 sd と同程度にすること。両側に失敗モードがある:
    #   小さすぎる -> 重みが最良サンプル1本に集中して実質 argmax。平均化が
    #                効かず更新が暴れる(lam=1 で ESS が 800中2.5)
    #   大きすぎる -> 重みがほぼ一様になり Σw*eps が平均 0 に潰れて舵が
    #                切れなくなる(lam=3000 では隙間回避の終端 y が
    #                -0.73m -> -0.02m まで消えた。論文 Fig.3 の右端の挙動)
    # 実測した合計コスト sd は廊下 375 / コーナー進入 430 / 壁に密着 363 で、
    # w_collision を下げてあるおかげでどの場面もほぼ同じ。300 で全域に合う。
    lam: float = 300.0
    sigma_a: float = 1.2           # 加速度ノイズ[m/s^2]
    sigma_delta: float = 0.10      # 舵角ノイズ[rad]

    # --- 局所グリッド ---
    grid_x_min: float = -1.0
    grid_x_max: float = 6.0
    grid_y_half: float = 3.5
    grid_res: float = 0.05
    grid_dilate: int = 1           # 占有セルの膨張[cell]

    # --- コスト重み (0 で無効化できること) ---
    # 衝突ペナルティ。有限値にすること。大きすぎると「衝突項だけがコストの
    # ばらつきを支配する」状態になり、場面によって sd が何倍も振れて lam を
    # 一つに決められなくなる(3000 ではコーナー進入で衝突項 sd 437、合計 sd が
    # 廊下 375 に対し 789 と 2.1 倍に跳ねた)。1000 なら合計 sd は
    # 363〜430 でほぼ一定。衝突はラッチするので t 歩目で当たると
    # w_collision*(T-t) 効き、抑止力としてはこれで十分足りる。
    w_collision: float = 1000.0
    w_obs: float = 300.0           # ソフト回避
    d_safe: float = 0.40           # [m] これより壁に近いとコストが立ち上がる
    # FTG 目標点への終端コスト。0 で FTG 無効(寄与度の切り分け用)。
    # 他項と釣り合う大きさが要る。w_goal=8 では終端誤差 1m でも 8 にしか
    # ならず、経路コスト ~900 に埋もれて全く効かなかった(実測)。
    # 200 なら候補間の終端差 0.5m が 100 の差になり、方向づけとして効く。
    w_goal: float = 200.0
    # 速度コストは衝突コストと釣り合う大きさにすること。ここが小さいと
    # 「壁の手前まで這って止まる」が最適解になり、横に避けなくなる
    # (w_speed=1 では 30 step 這っても 24 にしかならず、当時の衝突 1 step
    #  3000 に対し 1/125 でしかなかった。実測で確認済み)。
    #   這い続けるコスト = w_speed * (v_target - v_min)^2 * T
    #                    = 150 * 0.81 * 30 = 3645
    # 比較対象は「衝突 1step」ではなくラッチ後の総額 w_collision*(T-t)。
    # ホライゾン中程(t=15)で当たれば 1000*15 = 15000 で、這うより 4 倍高い。
    # 逆に終盤(t=27)なら 3000 で這うのと同等 — この釣り合いが「ぎりぎりまで
    # 攻めるが最後は避ける」を生む。単体テスト「回り込めない壁で減速」が
    # この均衡を見張っていて、w_collision を 500 まで下げると落ちる。
    w_speed: float = 150.0
    v_target: float = 1.2
    w_smooth: float = 20.0
    # 壁へ近い候補では、FTG 目標点を向く姿勢を早い時刻ほど優先する。
    # 終端距離だけでは「前進したので左前方の点へ近づいた直進軌道」と
    # 「実際に左へ向いた軌道」を区別できないため、姿勢を独立に評価する。
    w_heading: float = 200.0
    heading_danger_distance: float = 0.60  # [m] 方向コストが立ち上がる距離
    heading_time_power: float = 1.0        # 0で全時刻同率、正なら早い旋回を優先

    rng_seed: int = 0

    @classmethod
    def from_yaml(cls, vehicle: dict, mppi: dict) -> "MPPIConfig":
        """vehicle_params.yaml と mppi_params.yaml から組み立てる。"""
        c = cls()
        v, lim = vehicle["vehicle"], vehicle["limits"]
        c.wheelbase = float(v["wheelbase"])
        c.half_width = float(v["half_width"])
        c.lidar_dx = float(v["lidar_x_from_rear_axle"])
        c.v_min = float(lim["v_min_forward"])
        c.v_max = float(lim["v_max"])
        c.a_max = float(lim["a_max"])
        c.delta_max = float(lim["delta_max"])
        for k, val in mppi.items():
            if hasattr(c, k):
                setattr(c, k, type(getattr(c, k))(val))
        return c


# ----------------------------------------------------------------------
class LocalESDF:
    """スキャン点群から「最近傍障害物までの距離」の場を作る。

    レイ参照(角度ビン)ではなく距離場にしたのは、壁からの距離が滑らかな
    連続量として効くようにするため。「今わずかに外へ出る -> 1.5秒後に
    曲がりきれる」という軌道が「まっすぐ行って衝突」より低コストになるのは
    この勾配があるからで、二値の衝突判定だと差が出ない。
    """

    def __init__(self, cfg: MPPIConfig):
        self.res = cfg.grid_res
        self.x0, self.x1 = cfg.grid_x_min, cfg.grid_x_max
        self.y0, self.y1 = -cfg.grid_y_half, cfg.grid_y_half
        self.nx = int(round((self.x1 - self.x0) / self.res))
        self.ny = int(round((self.y1 - self.y0) / self.res))
        self.dilate = int(cfg.grid_dilate)
        # 障害物が1つも無いときの既定値。グリッド対角より大きく取る。
        self._far = float(np.hypot(self.x1 - self.x0, self.y1 - self.y0))
        self.dist = np.full((self.nx, self.ny), self._far, dtype=np.float32)

    def build(self, px: np.ndarray, py: np.ndarray) -> None:
        """点群(後輪車軸座標)から距離場を作る。"""
        occ = np.zeros((self.nx, self.ny), dtype=bool)
        inside = ((px >= self.x0) & (px < self.x1)
                  & (py >= self.y0) & (py < self.y1))
        if inside.any():
            ix = ((px[inside] - self.x0) / self.res).astype(np.int32)
            iy = ((py[inside] - self.y0) / self.res).astype(np.int32)
            occ[ix, iy] = True

        # 占有セルを膨張させてから距離変換する。ビーム間隔は 5m 地点で
        # 3.7cm、セルは 5cm なので、遠方ほど壁が離散化で穴あきになり、
        # ロールアウトがすり抜ける。1セル膨らませて連続な壁にする。
        if self.dilate > 0 and occ.any():
            occ = binary_dilation(occ, iterations=self.dilate)

        if occ.any():
            # 空きセルについて最近傍占有セルまでの距離[cell] -> [m]
            self.dist = (distance_transform_edt(~occ) * self.res).astype(np.float32)
        else:
            self.dist.fill(self._far)

    def query(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """任意の点(配列可)における最近傍障害物までの距離[m]。

        グリッド外は端の値で代用する。ホライゾン 2m 程度に対しグリッドは
        6m あるので通常は起きないが、外に出た瞬間コスト 0 になって
        「グリッド外へ逃げる」解が最適になるのを防ぐ意味がある。
        """
        ix = np.clip(((x - self.x0) / self.res).astype(np.int32), 0, self.nx - 1)
        iy = np.clip(((y - self.y0) / self.res).astype(np.int32), 0, self.ny - 1)
        return self.dist[ix, iy]

    def scan_to_points(self, ranges: np.ndarray, angles: np.ndarray,
                       lidar_dx: float, r_min: float, r_max: float) -> tuple:
        """LaserScan を後輪車軸座標の点群に直す。無効値は落とす。"""
        r = np.asarray(ranges, dtype=np.float64)
        ok = np.isfinite(r) & (r > r_min) & (r < r_max)
        a = angles[ok]
        r = r[ok]
        return r * np.cos(a) + lidar_dx, r * np.sin(a)


# ----------------------------------------------------------------------
class VehicleModel:
    """自転車モデル + 操舵一次遅れ。K 本のロールアウトを同時に進める。

    操舵遅れを入れてあるのは、Gazebo の AckermannSteering が
    「関節速度 = gain x (目標角 - 現在角)」の比例制御で、定義上そのまま
    時定数 tau = 1/gain の一次遅れになるため(実測と理論が誤差1.1%で一致)。
    tau=0 で遅れ無しに退化する。実機サーボが遅い場合の保険でもある。
    """

    def __init__(self, cfg: MPPIConfig):
        self.c = cfg
        # dt/tau が 1 を超えると発散するので頭打ちにする。
        self.alpha = 1.0 if cfg.steer_tau <= 1e-6 else min(cfg.dt / cfg.steer_tau, 1.0)

    def rollout(self, v0: float, delta0: float, u: np.ndarray) -> dict:
        """u: (K, T, 2) = (加速度, 目標舵角) -> 各ステップの状態。

        時間方向はループ、サンプル方向はベクトル化。T=30 回の
        K 要素ベクトル演算で済むので numpy でも十分速い。
        """
        c = self.c
        K, T = u.shape[0], u.shape[1]
        x = np.zeros(K)
        y = np.zeros(K)
        th = np.zeros(K)
        v = np.full(K, float(v0))
        d = np.full(K, float(delta0))

        xs = np.empty((K, T))
        ys = np.empty((K, T))
        vs = np.empty((K, T))
        ds = np.empty((K, T))
        ths = np.empty((K, T))

        for t in range(T):
            a = u[:, t, 0]
            d_cmd = u[:, t, 1]
            # 操舵は指令に一次遅れで追従する
            d = d + (d_cmd - d) * self.alpha
            np.clip(d, -c.delta_max, c.delta_max, out=d)
            v = np.clip(v + a * c.dt, c.v_min, c.v_max)
            # 自転車モデル。原点は後輪車軸。
            th = th + v * np.tan(d) / c.wheelbase * c.dt
            x = x + v * np.cos(th) * c.dt
            y = y + v * np.sin(th) * c.dt
            xs[:, t], ys[:, t], vs[:, t], ds[:, t], ths[:, t] = x, y, v, d, th

        return {"x": xs, "y": ys, "theta": ths, "v": vs, "delta": ds}


# ----------------------------------------------------------------------
class MPPICost:
    """コスト項。各重みは 0 で無効化できる(寄与度を切り分けるため)。"""

    def __init__(self, cfg: MPPIConfig):
        self.c = cfg

    def __call__(self, traj: dict, esdf: LocalESDF, delta0: float,
                 goal: tuple | None, breakdown: bool = False):
        """合計コスト (K,) を返す。breakdown=True なら (S, 項ごとの (K,) 辞書)。

        内訳を出せるようにしてあるのは、この実装で踏んだ不具合が4件とも
        「項どうしのスケールが釣り合っていない」だったため:
          w_speed=1   -> 前進の動機が衝突の 1/125 しかなく壁の手前で停止
          w_goal=8    -> 経路コスト ~900 に埋没して FTG の指示が無効
          lam=1       -> 有効サンプル数が 800中2.5 で実質 argmax
          lam=3000    -> 逆に重みがほぼ一様になり Σw*eps が 0 に潰れて無舵
          衝突ラッチ無し -> 壁をすり抜けて「速いほど安い」

        平均だけでなく**サンプル間の標準偏差**を見ること。MPPI は項の絶対値
        ではなくサンプル間の差で最適化するので、値が大きくてもばらつきが
        無い項は何も動かさない。最後の不具合はまさにそれだった。
        """
        c = self.c
        xs, ys = traj["x"], traj["y"]
        ths, vs, ds = traj["theta"], traj["v"], traj["delta"]

        # --- 障害物 ---
        dist = esdf.query(xs, ys)

        # 一度衝突したら以降のステップも全て衝突として数える(ラッチする)。
        # これが無いとロールアウトが壁をすり抜けられてしまい、
        # 「速く走るほど壁の向こうへ早く抜けて衝突ステップ数が減る」
        # という理由で加速が最適解になる。実測で確認: 壁に密着した状態から
        # cost_min が 24037 -> 9679 -> 6845 と下がりながら v が 0.30 -> 1.6
        # へ上がり、全速力で壁を押し続けた。
        # ラッチすれば「衝突を遅らせる = 減速・回避」が最も安くなる。
        hit = np.maximum.accumulate(dist < c.half_width, axis=1)

        # 衝突は有限のペナルティにする。inf にすると全サンプルが脱落した
        # ときに重みが全て 0 になって解が出ない。
        collide = hit.astype(np.float64) * c.w_collision
        # 壁に近いほど滑らかに立ち上がるコスト。この勾配が「膨らんで
        # 曲がる」を生む。二値の衝突判定だけだと膨らむ動機が無い。
        soft = c.w_obs * np.square(np.maximum(0.0, c.d_safe - dist))

        speed = (c.w_speed * np.square(c.v_target - vs) if c.w_speed
                 else np.zeros_like(vs))

        if c.w_smooth:
            prev = np.concatenate(
                [np.full((ds.shape[0], 1), delta0), ds[:, :-1]], axis=1)
            smooth = c.w_smooth * np.square(ds - prev)
        else:
            smooth = np.zeros_like(ds)

        # --- 終端: FTG 目標点 ---
        # w_goal=0 で FTG 無しになる。寄与度を実測で切り分けられるように
        # しておくのが目的。
        if goal is not None and c.w_goal:
            gx, gy = goal
            goal_term = c.w_goal * np.hypot(xs[:, -1] - gx, ys[:, -1] - gy)
        else:
            goal_term = np.zeros(xs.shape[0])

        # --- 危険時の目標方位 ---
        # 終端距離だけでは、左前方の目標に対して「左を向く」と「直進して x
        # だけ近づく」を区別できない。壁へ近い区間では、各予測位置から見た
        # FTG 目標方位と車体姿勢の誤差を加える。危険度は候補ごとの ESDF
        # 距離から連続値で作り、接触直前で 1、指定距離以上で 0 とする。
        # 時間重みは早い時刻ほど大きくし、旋回をホライゾン後半へ先送りする
        # 解を不利にする。goal が無い場合は方向を捏造せず完全に無効化する。
        if goal is not None and c.w_heading:
            gx, gy = goal
            desired = np.arctan2(gy - ys, gx - xs)
            heading_error = (ths - desired + np.pi) % (2.0 * np.pi) - np.pi

            danger_span = max(c.heading_danger_distance - c.half_width, 1e-9)
            danger = np.clip(
                (c.heading_danger_distance - dist) / danger_span, 0.0, 1.0)
            remaining = (np.arange(xs.shape[1], 0, -1, dtype=np.float64)
                         / xs.shape[1])
            time_weight = np.power(remaining, max(c.heading_time_power, 0.0))
            heading = c.w_heading * danger * time_weight[None, :] \
                * np.square(heading_error)
        else:
            heading = np.zeros_like(xs)

        parts = {
            "衝突": collide.sum(axis=1),
            "回避": soft.sum(axis=1),
            "速度": speed.sum(axis=1),
            "平滑": smooth.sum(axis=1),
            "目標": goal_term,
            "方向": heading.sum(axis=1),
        }
        S = sum(parts.values())
        return (S, parts) if breakdown else S


# ----------------------------------------------------------------------
class MPPI:
    """サンプリング・重み付け・warm start。"""

    def __init__(self, cfg: MPPIConfig):
        self.c = cfg
        self.model = VehicleModel(cfg)
        self.cost = MPPICost(cfg)
        self.esdf = LocalESDF(cfg)
        self.rng = np.random.default_rng(cfg.rng_seed)
        # 名目入力列。前回解を1ステップずらして使い回す(warm start)。
        self.u_nom = np.zeros((cfg.T, 2))

    def reset(self) -> None:
        self.u_nom[:] = 0.0

    def step(self, v0: float, delta0: float, goal: tuple | None,
             shift: bool = True) -> dict:
        """1周期ぶんの最適化。esdf は事前に build 済みであること。

        返り値の "v", "delta_cmd" が今出すべき指令(論文 Alg.1 の
        SendToActuators(u_0) に相当)。"delta" は指令ではなく、その指令を
        1ステップ入れた後の**実舵角の推定値**で、次周期の初期舵角に使う。
        "traj" は採用軌道、"samples" は可視化用のサンプル軌道。

        shift: warm start のため解を1ステップ前へずらすか。実運用では
            車が進むので True。同一状態から収束させたい場合(オフライン
            テスト)は False にする。True のまま凍結状態で反復すると
            毎回先頭の制御を捨てることになり、収束が進まない。
        """
        c = self.c
        sigma = np.array([c.sigma_a, c.sigma_delta])
        eps = self.rng.normal(size=(c.K, c.T, 2)) * sigma
        u = self.u_nom[None, :, :] + eps
        np.clip(u[:, :, 0], -c.a_max, c.a_max, out=u[:, :, 0])
        np.clip(u[:, :, 1], -c.delta_max, c.delta_max, out=u[:, :, 1])
        # クリップ後の実効ノイズを使わないと、重み更新が制約と矛盾する。
        eps = u - self.u_nom[None, :, :]

        traj = self.model.rollout(v0, delta0, u)
        S, parts = self.cost(traj, self.esdf, delta0, goal, breakdown=True)

        # 重み。S_min を引くのは exp のオーバーフロー対策。
        w = np.exp(-(S - S.min()) / max(c.lam, 1e-9))
        w_sum = w.sum()
        if not np.isfinite(w_sum) or w_sum <= 0.0:
            # 数値的に破綻した場合は更新しない(前回解を維持)。
            w = np.zeros_like(w)
        else:
            w = w / w_sum

        self.u_nom = self.u_nom + np.einsum("k,ktj->tj", w, eps)
        np.clip(self.u_nom[:, 0], -c.a_max, c.a_max, out=self.u_nom[:, 0])
        np.clip(self.u_nom[:, 1], -c.delta_max, c.delta_max, out=self.u_nom[:, 1])

        # 採用軌道(名目入力を1本流す)
        nom = self.model.rollout(v0, delta0, self.u_nom[None, :, :])

        out = {
            "v": float(nom["v"][0, 0]),
            # 指令舵角。warm start の roll はこの後なので u_nom[0] はまだ
            # 「今出すべき制御」を指している。これをそのまま車へ出す。
            "delta_cmd": float(self.u_nom[0, 1]),
            # 上の指令に操舵一次遅れを1ステップ適用した「実舵角の推定値」。
            # 舵角は実測できないので、次周期の rollout 初期値として使う。
            # 指令ではない — これを publish すると gz 側の遅れと二重に掛かる。
            "delta": float(nom["delta"][0, 0]),
            "traj": (nom["x"][0], nom["y"][0]),
            "cost_min": float(S.min()),
            "cost_mean": float(S.mean()),
            "collision_frac": float(
                (self.esdf.query(traj["x"], traj["y"]) < c.half_width)
                .any(axis=1).mean()),
            "samples": (traj["x"], traj["y"]),
            "u_nom": self.u_nom.copy(),
            # 項ごとの 平均 / サンプル間の標準偏差 / 占有率。
            # sd が 0 に近い項は「値は大きいが差が無い = 最適化を動かして
            # いない」。平均だけ見ていると効いているように誤読する。
            "cost_parts": {
                k: (float(x.mean()), float(x.std()),
                    float(x.mean() / S.mean() * 100.0) if S.mean() > 0 else 0.0)
                for k, x in parts.items()
            },
            # 有効サンプル数。温度 lam が適切かの指標。K に対して極端に
            # 小さいと実質 argmax になっていて平均化が効いていない。
            "ess": float(1.0 / np.square(w).sum()) if w.sum() > 0 else 0.0,
        }

        # warm start: 1ステップずらして次周期の初期値にする。
        if shift:
            self.u_nom = np.roll(self.u_nom, -1, axis=0)
            self.u_nom[-1] = self.u_nom[-2]
        return out
