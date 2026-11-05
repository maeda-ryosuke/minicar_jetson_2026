#!/usr/bin/env python3
"""地図の上に手で線を描き、それを種にきれいなレースラインを作る。

    python tools/raceline/draw_raceline.py          # ブラウザで http://127.0.0.1:8765 が開く

ROS 不要(numpy / scipy / Pillow / PyYAML。matplotlib も要らない)。ホストの python で叩く。
サーバは標準ライブラリの http.server で、127.0.0.1 にだけ開く。

=== 使い方 ===

1. 地図の上で点を打つ(クリック)か、フリーハンドで 1 周描く。点の順が走る向き。
   既存の線(maps/*.csv)を読み込んで、そこから点をいじってもよい。
2. 「きれいにする」で解く。結果は壁余裕で色分けして重ねる(赤 = 下限ぎりぎり)。
3. 名前を付けて保存すると maps/<名前>.csv に書く(列は x, y, yaw / 0.05m 間隔)。
   maps/raceline.csv は上書きできない。既存のファイル名は確認してから上書きする。

=== smooth_raceline.py との違い ===

目的関数は smooth_raceline.objective をそのまま使い、次だけ変えている。

* 初期値は通路の中心線ではなく、描いた点を周期 3 次スプラインで結んだ線。
* 描いた線からの横ずれの二乗積分(w_stay)を足す。どの通路を通るか、どこで
  インに付くかといった描いた線の性格を残し、くねり・曲率の上限超え・
  壁とマットへの接近だけを直す。w_stay を下げるほど描いた線から離れて丸くなる。
* clear_target(広い所で壁から離す)は既定で切ってある(w_clear 0)。描いた線と
  引っ張り合うため。clear_near(狭い所で真ん中を通す)と壁余裕の下限は残す。
* 対話で使うので既定のパス数を 2 に減らし、収束判定を緩めてある
  (smooth_raceline は 3 パスで約 1 分。こちらは 20〜40 秒)。

s=0 は maps/raceline.csv の先頭に一番近い点に揃える(他の線と s を比べやすいように)。

=== 合否 ===

画面に出すだけで、NG でも保存はできる(確認が出る)。
* |kappa| 最大(0.25m 移動平均)が 1/r_target を超える
* 壁余裕が half_width + wall_margin を下回る
* 避けるマットまでの距離が half_width + マット余裕を下回る
* maps/raceline.csv と逆回り(警告のみ)
"""

import argparse
import base64
import io
import json
import os
import re
import sys
import threading
import time
import uuid
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace
from urllib.parse import parse_qs, urlparse

import numpy as np
import yaml
from PIL import Image
from scipy import optimize, spatial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from relax_corners import (  # noqa: E402
    DS, ROOT, WallField, dist_to_poly, kappa_avg, read_raceline_csv,
    resample_closed, tangents, write_raceline_csv, yaw_of)
from smooth_raceline import (  # noqa: E402
    MAT_MARGIN, MAT_MARGIN_DEFAULT, Field, add_walls, load_polys, objective,
    periodic_basis, read_csv_rows, start_at)

HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "draw_raceline.html")
MAPS = os.path.join(ROOT, "maps")
PROTECTED = {"raceline.csv"}          # 上書きさせない
NAME_RE = re.compile(r"^[A-Za-z0-9_\-]+$")

# 画面の初期値。smooth_raceline の既定に合わせ、w_clear だけ切る。
DEFAULTS = {
    "w_stay": 1000.0, "w_dk": 300.0, "w_k": 1.0, "r_target": 1.0,
    "wall_margin": 0.15, "clear_near": 0.32, "w_near": 1e6,
    "clear_target": 0.45, "w_clear": 0.0, "knot": 0.4, "passes": 2,
}


class Stay:
    """描いた線 D からの符号付き横ずれ。最寄り点の法線へ射影する。"""

    def __init__(self, D):
        self.D = D
        t = tangents(D)
        self.n = np.c_[-t[:, 1], t[:, 0]]
        self.tree = spatial.cKDTree(D)

    def __call__(self, Q):
        _, j = self.tree.query(Q)
        return np.sum((Q - self.D[j]) * self.n[j], axis=1), self.n[j]


def solve(C, a, stay, nc):
    """smooth_raceline.solve に横ずれの項を足したもの。"""
    a.mask = np.zeros(len(C))
    t = tangents(C)
    nrm = np.c_[-t[:, 1], t[:, 0]]
    B = periodic_basis(len(C), nc)

    def fun(c):
        Q = C + (B @ c)[:, None] * nrm
        J, gQ = objective(Q, a)
        if a.w_stay > 0:
            lat, n = stay(Q)
            J += a.w_stay * DS * np.sum(lat * lat)
            gQ = gQ + (2 * a.w_stay * DS * lat)[:, None] * n
        return J, B.T @ np.sum(gQ * nrm, axis=1)

    res = optimize.minimize(fun, np.zeros(nc), jac=True, method="L-BFGS-B",
                            options={"maxiter": 20000, "maxfun": 10 ** 5,
                                     "ftol": 1e-12, "gtol": 1e-8})
    return C + (B @ res.x)[:, None] * nrm, res


class Track:
    """地図・壁・マット・車両の寸法。起動時に 1 回だけ読む。"""

    def __init__(self, a):
        with open(a.vehicle, encoding="utf-8") as f:
            veh = yaml.safe_load(f)
        self.hw = float(veh["vehicle"]["half_width"])
        self.r_min = float(veh["limits"]["r_min"])
        self.wf = WallField(a.map, a.min_blob)
        add_walls(self.wf, a.extra_walls)
        self.field = Field(self.wf)
        self.polys = load_polys(a.low_mu)
        self.walls = [[float(r[k]) for k in ("x1", "y1", "x2", "y2")]
                      for r in (read_csv_rows(a.extra_walls) if a.extra_walls else [])]
        _, x, y, _, _ = read_raceline_csv(a.reference, closed=True)
        self.ref = np.c_[x, y]
        self.lock = threading.Lock()
        self.results = {}

    def init_payload(self):
        buf = io.BytesIO()
        Image.fromarray(self.wf.raw).save(buf, format="PNG")
        return {
            "image": "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode(),
            "res": self.wf.res, "ox": self.wf.ox, "oy": self.wf.oy,
            "w": self.wf.w, "h": self.wf.h,
            "walls": self.walls,
            "mats": {n: p.tolist() for n, p in self.polys.items()},
            "racelines": list_racelines(),
            "hw": self.hw, "r_min": self.r_min, "defaults": DEFAULTS,
            "ref_start": self.ref[0].tolist(),
        }

    def optimize(self, pts, prm, avoid):
        t0 = time.time()
        p = dict(DEFAULTS)
        p.update({k: float(v) for k, v in prm.items() if k in DEFAULTS})
        if p["r_target"] < self.r_min:
            raise ValueError(f"最小半径 {p['r_target']} m は r_min {self.r_min} m より小さい")
        a = SimpleNamespace(
            field=self.field, k_lim=1.0 / p["r_target"], clear_min=self.hw + p["wall_margin"],
            clear_target=p["clear_target"], w_clear=p["w_clear"],
            clear_near=p["clear_near"], w_near=p["w_near"],
            w_k=p["w_k"], w_dk=p["w_dk"], w_over=1e6, w_stay=p["w_stay"],
            w_straight=0.0, pulls=[], pull_masks=[])
        mats = {n: (poly, self.hw + MAT_MARGIN.get(n, MAT_MARGIN_DEFAULT))
                for n, poly in self.polys.items() if n in avoid}
        a.mats = list(mats.values())

        S = seed(pts)
        stay = Stay(resample_closed(S, ds=0.01))
        P = S
        log = []
        for it in range(int(p["passes"])):
            nc = max(8, int(round(len(P) * DS / p["knot"])))
            # 上限は罰金なのでわずかに超える。解く側だけ 2% 内側を狙う
            k_lim, a.k_lim = a.k_lim, a.k_lim * 0.98
            Q, res = solve(P, a, stay, nc)
            a.k_lim = k_lim
            P = resample_closed(Q)
            log.append(f"pass {it + 1}: {res.message} (評価 {res.nfev} 回)")
        P = start_at(P, self.ref[0])

        k = kappa_avg(P)
        clr = self.field(P)
        lat, _ = stay(P)
        checks = []
        kmax = float(np.abs(k).max())
        i = int(np.argmax(np.abs(k)))
        checks.append({"ok": kmax <= a.k_lim * 1.005,
                       "msg": f"最小 R {1 / kmax:.3f} m (s={i * DS:.1f}m) / 目標 {p['r_target']} m"})
        i = int(np.argmin(clr))
        checks.append({"ok": bool(clr[i] >= a.clear_min),
                       "msg": f"壁余裕 最小 {clr[i]:.3f} m (s={i * DS:.1f}m) / 下限 {a.clear_min:.3f} m"})
        for n, (poly, need) in mats.items():
            v = float((dist_to_poly(P, poly) - need).min())
            checks.append({"ok": v >= 0.0, "msg": f"{n} 必要距離に対する余り {v:+.3f} m"})
        # 走る向き: maps/raceline.csv の先頭での接線と比べる
        tr = self.ref[1] - self.ref[0]
        same = float(tangents(P)[0] @ tr) > 0
        checks.append({"ok": same, "warn": True,
                       "msg": "maps/raceline.csv と同じ回り" if same else
                       "maps/raceline.csv と逆回り(点の順が走る向き)"})

        sg = np.sign(k[np.abs(k) > 0.15])
        dk = np.diff(np.r_[k, k[:1]]) / DS
        stats = {
            "length": float(np.linalg.norm(np.roll(P, -1, 0) - P, axis=1).sum()),
            "k2": float(np.sum(k * k) * DS),
            "dk": float(np.sum(np.abs(dk)) * DS),
            "flips": int(np.sum(sg != np.roll(sg, 1))),
            "lat_max": float(np.abs(lat).max()),
            "lat_rms": float(np.sqrt(np.mean(lat * lat))),
            "time": time.time() - t0,
        }
        rid = uuid.uuid4().hex[:12]
        self.results = {rid: P}               # 保存できるのは最後の結果だけ
        return {
            "id": rid, "points": np.round(P, 4).tolist(),
            "clr": np.round(clr, 3).tolist(), "kappa": np.round(k, 3).tolist(),
            "seed": np.round(S[::2], 4).tolist(),
            "checks": checks, "stats": stats, "log": log,
            "clear_min": a.clear_min, "k_lim": a.k_lim,
        }

    def save(self, rid, name, overwrite):
        if rid not in self.results:
            return 400, {"error": "保存する結果が無い(きれいにし直してから保存する)"}
        if not NAME_RE.match(name):
            return 400, {"error": "名前は英数字と _ - だけ(拡張子なし)"}
        fn = name + ".csv"
        if fn in PROTECTED:
            return 403, {"error": f"maps/{fn} は上書きしない。別の名前にする"}
        path = os.path.join(MAPS, fn)
        if os.path.exists(path) and not overwrite:
            return 409, {"exists": True, "error": f"maps/{fn} は既にある"}
        P = self.results[rid]
        write_raceline_csv(path, P[:, 0], P[:, 1], yaw_of(P))
        return 200, {"path": f"maps/{fn}", "jetson": f"raceline_file:=/maps/{fn}"}


def seed(pts):
    """描いた点を周期 3 次スプラインで結び、0.05m 間隔に取り直す。"""
    P = np.asarray(pts, dtype=float)
    if P.ndim != 2 or P.shape[1] != 2:
        raise ValueError("点の形式が違う")
    P = P[np.r_[True, np.linalg.norm(np.diff(P, axis=0), axis=1) > 1e-3]]
    if len(P) > 1 and np.linalg.norm(P[0] - P[-1]) < 1e-3:
        P = P[:-1]
    if len(P) < 4:
        raise ValueError("点が 4 つ以上要る")
    return resample_closed(P)


def list_racelines():
    out = []
    for fn in sorted(os.listdir(MAPS)):
        if not fn.endswith(".csv"):
            continue
        with open(os.path.join(MAPS, fn), encoding="utf-8") as f:
            head = f.readline().strip()
        if head == "x,y,yaw":
            out.append(fn[:-4])
    return out


def make_handler(track):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def send(self, code, body, ctype="application/json; charset=utf-8"):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path in ("/", "/index.html"):
                with open(HTML, "rb") as f:
                    self.send(200, f.read(), "text/html; charset=utf-8")
            elif u.path == "/api/init":
                self.send(200, track.init_payload())
            elif u.path == "/api/raceline":
                name = parse_qs(u.query).get("name", [""])[0]
                if name not in list_racelines():
                    return self.send(404, {"error": f"maps/{name}.csv が無い"})
                _, x, y, _, _ = read_raceline_csv(os.path.join(MAPS, name + ".csv"))
                self.send(200, {"points": np.round(np.c_[x, y], 4).tolist()})
            else:
                self.send(404, {"error": "not found"})

        def do_POST(self):
            u = urlparse(self.path)
            try:
                req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            except ValueError:
                return self.send(400, {"error": "JSON が読めない"})
            if u.path == "/api/optimize":
                with track.lock:
                    try:
                        out = track.optimize(req.get("points", []), req.get("params", {}),
                                             set(req.get("avoid", [])))
                    except ValueError as e:
                        return self.send(400, {"error": str(e)})
                print(f"[optimize] {out['stats']['time']:.1f}s  " +
                      " / ".join(c["msg"] for c in out["checks"]))
                self.send(200, out)
            elif u.path == "/api/save":
                code, out = track.save(req.get("id", ""), req.get("name", ""),
                                       bool(req.get("overwrite")))
                if code == 200:
                    print(f"[save] {out['path']}")
                self.send(code, out)
            else:
                self.send(404, {"error": "not found"})

    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--map", default=os.path.join(ROOT, "maps/track_v1.yaml"))
    ap.add_argument("--extra-walls", default=os.path.join(ROOT, "maps/track_v1_extra_walls.csv"),
                    help="地図に足す壁線 CSV (name,x1,y1,x2,y2 / map 座標)。'' で無し")
    ap.add_argument("--low-mu", default=os.path.join(ROOT, "maps/track_v1_low_mu.csv"),
                    help="マットの輪郭 CSV (name,i,x,y / map 座標)。どれを避けるかは画面で選ぶ")
    ap.add_argument("--reference", default=os.path.join(ROOT, "maps/raceline.csv"),
                    help="s=0 の位置と走る向きの基準にする線")
    ap.add_argument("--vehicle", default=os.path.join(
        ROOT, "src/minicar_bringup/config/vehicle_params.yaml"))
    ap.add_argument("--min-blob", type=int, default=20)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args(argv)

    track = Track(a)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(track))
    url = f"http://127.0.0.1:{a.port}/"
    print(f"{url} を開く (Ctrl+C で終了)")
    if not a.no_browser:
        threading.Timer(0.5, webbrowser.open, args=(url,)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
