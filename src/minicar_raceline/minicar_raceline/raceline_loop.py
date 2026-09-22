"""周回路としての参照経路 (raceline CSV)。ROS 非依存。

実機用。Gazebo 側リポジトリ (minicar_2026/nodes/raceline_loop.py) と
**別物として独立に持っている**。共有していないので、片方を直したときは
もう片方も見ること。違いは 2 点だけ:

  * こちらは world 座標の概念を持たない。**CSV は最初から map 座標**。
    シム側は gz の world -> map 静的TF があるので原点を引く処理が要るが、
    実機の地図は slam_toolbox が作る map そのもので、world は存在しない。
  * こちらは v_ref を扱わない。速度は controller_server の vx_max 固定。

=== CSV の形 ===

    x, y, yaw                 <- これだけ

単位は x, y が [m]、yaw が [rad]。座標系は **map**(slam_toolbox の地図原点)。

s(弧長)と kappa(曲率)は **読み込み時に x,y,yaw から導出する**。
幾何から一意に決まる量をファイルに持たせると、片方だけ編集したときに
矛盾した CSV ができるため。docs/LIDDER_MAPII.PNG の構成図が
「Race Line CSV (x, y, th リスト)」としているのに合わせてある。

v_ref(目標速度)の列も持たない。速度は横加速度の上限と路面摩擦で決まるが、
どちらも実機で走らせる前には測れない。事前計算した値を経路ファイルへ
焼き込むと、根拠のない数字が固定されてしまう。**速度は全周 vx_max 一定**で、
その値は実走で決める (nav2 の FollowPath.vx_max)。

帰結: **コーナーで自動的に減速しない。** vx_max を上げるときは、
最小旋回半径の一番きついコーナーで曲がり切れるかを先に確かめること。

旧 4 列 (v_ref つき) や旧 6 列 (s, kappa つき) の CSV も読める。
余分な列の値は無視して導出し直すので、経路の幾何が唯一の出所になる。
"""

import csv
import math

import numpy as np


def read_raceline_csv(path, closed=None):
    """レースライン CSV を読み、(s, x, y, yaw, kappa) を返す。

    closed=None のときは始点と終点の距離で周回かどうかを判定する
    (点間隔の 3 倍以内なら周回)。周回なら kappa は端を巻いて出す。
    """
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path} が空")
    for k in ("x", "y", "yaw"):
        if not rows or k not in rows[0]:
            raise ValueError(f"{path} に {k} 列が無い (必要な列は x, y, yaw)")
    g = lambda k: np.array([float(r[k]) for r in rows])  # noqa: E731
    x, y, yaw = g("x"), g("y"), g("yaw")

    # 弧長。点間の直線距離の累積。s[0] = 0。
    step = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate([[0.0], np.cumsum(step)])

    if closed is None:
        ds = float(np.median(step)) if len(step) else 1.0
        closed = bool(math.hypot(x[0] - x[-1], y[0] - y[-1]) < 3.0 * ds)

    # 曲率 = 進行方向の変化率 dyaw/ds。yaw は連続でないので差を [-pi, pi) に畳む。
    # 周回なら端を巻き、片道なら端を複製して長さを合わせる。
    if closed:
        dyaw = np.angle(np.exp(1j * (np.roll(yaw, -1) - yaw)))
        dseg = np.concatenate([step, [math.hypot(x[0] - x[-1], y[0] - y[-1])]])
    else:
        dyaw = np.angle(np.exp(1j * np.diff(yaw)))
        dyaw = np.concatenate([dyaw, dyaw[-1:]])
        dseg = np.concatenate([step, step[-1:]])
    kappa = dyaw / np.maximum(dseg, 1e-9)
    return s, x, y, yaw, kappa


def write_raceline_csv(path, x, y, yaw):
    """レースライン CSV を書く。列は x, y, yaw の3つだけ。

    csv.writer は既定で CRLF を書く(RFC 4180)。読み手を選ばないよう LF に
    揃える。
    """
    with open(path, "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["x", "y", "yaw"])
        for i in range(len(x)):
            w.writerow(["%.4f" % x[i], "%.4f" % y[i], "%.5f" % yaw[i]])


class LoopPath:
    """周回をまたげる参照経路。添字を mod N で扱う。

    周回路なので終点と始点が 1 刻みで繋がっている前提。片道経路を渡すと
    終点から始点へ瞬間移動する扱いになるので、その用途には使わないこと。
    """

    def __init__(self, s, x, y, yaw, kappa):
        self.s = np.asarray(s, dtype=float)
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        self.yaw = np.asarray(yaw, dtype=float)
        self.kappa = np.asarray(kappa, dtype=float)
        self.n = len(self.s)
        # 周長 = 最後の弧長 + 終点から始点へ戻る 1 区間。
        gap = math.hypot(self.x[0] - self.x[-1], self.y[0] - self.y[-1])
        self.length = float(self.s[-1] + gap)
        self.ds = self.length / self.n

    @classmethod
    def from_csv(cls, path):
        return cls(*read_raceline_csv(path, closed=True))

    def nearest(self, x, y, hint_idx=None, window=2.0):
        """(x,y) に最も近い点の添字と距離[m]。

        hint_idx を与えると弧長 ±window だけを周回込みで探す。全探索は
        蛇行コースで隣の通路の点に吸着するので、初回か経路から大きく
        外れたときだけにすること。
        """
        if hint_idx is None:
            idx = np.arange(self.n)
        else:
            w = int(math.ceil(window / self.ds))
            idx = np.arange(hint_idx - w, hint_idx + w + 1) % self.n
        d = np.hypot(self.x[idx] - x, self.y[idx] - y)
        k = int(np.argmin(d))
        return int(idx[k]), float(d[k])

    def ahead(self, idx, distance):
        """idx から前方 distance[m] ぶんの添字列(周回をまたいで続く)。"""
        m = max(2, int(math.ceil(distance / self.ds)) + 1)
        return (idx + np.arange(m)) % self.n

    def lateral_error(self, idx, x, y):
        """符号付き横偏差[m]。進行方向の左が正。"""
        th = self.yaw[idx]
        return -math.sin(th) * (x - self.x[idx]) + math.cos(th) * (y - self.y[idx])

    def step(self, i_prev, i_new):
        """添字 i_prev -> i_new の符号付き進み[点数]。周回の境目を正しく数える。"""
        d = (i_new - i_prev) % self.n
        return d - self.n if d > self.n // 2 else d
