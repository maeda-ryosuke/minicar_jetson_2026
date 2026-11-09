#!/usr/bin/env python3
"""analyze.py の results.json と図から、検証レポートの HTML を作る(ホストで実行)。

    python3 tools/bag_analysis/bicycle_replay/build_report.py out.html \
      bags/20261105_fast4 bags/20261105_02

図は data: URI で埋め込むので、HTML 1 枚で完結する。所見の文章は FINDINGS を書き換える。
"""
import base64
import html
import json
import sys
from pathlib import Path

FIGS = [
    ("fig_map.png", "地図・ゾーン・走行軌跡",
     "走行軌跡(元の走行の自己位置)を、その時刻に出ていた /target_speed で色分けした。"),
    ("fig_speed.png", "速度",
     "期待値は自己位置と zones.yaml から計算した目標速度。灰色の帯が判定区間。"),
    ("fig_delta.png", "ステア角 δ", "δ は atan(ω·L/v) で復元した。点線が ±steer_max。"),
    ("fig_traj_metrics.png", "最適軌道の指標",
     "上:経路からの距離(経路の長さの内側の点だけ)。下:最適軌道の最大曲率と上限 tan(steer_max)/L。"),
    ("fig_snapshots.png", "スナップショット",
     "矢印が車(元の走行)の位置と向き。青の太線が新しい MPPI の最適軌道、細線が候補軌道の一部、"
     "橙が同じ時刻の元の MPPI の最適軌道、破線が MPPI に渡された経路。"),
]


def img(path):
    return "data:image/png;base64," + base64.b64encode(path.read_bytes()).decode()


def verdict(ok):
    return ('<span class="pill ok">合格</span>' if ok is True else
            '<span class="pill warn">要確認</span>' if ok == "warn" else
            '<span class="pill ng">不合格</span>')


def rows(r):
    t, rp, dl, tr = r["target"], r["ramp"], r["delta"], r["traj"]
    follow = tr["dev_max_median_m"] <= tr["orig_dev_max_median_m"] + 0.05
    return [
        ("1. target_speed", verdict(t["pass"]),
         f"一致 {t['match_rate'] * 100:.1f}%({t['n']} 件中 {t['n_mismatch']} 件ずれ)。"
         f"ずれはすべてゾーンの縁から {t['mismatch_max_edge_dist_m']:.2f} m 以内"
         f"(許容 {t['edge_tol_m']:.2f} m = 最大速度 × 0.12 s)。"
         f"出力 {t['rate_hz']:.1f} Hz、値 {', '.join(str(v) for v in t['values'])} m/s"),
        ("2. v のランプ", verdict(rp["pass"]),
         f"加速の1周期あたり最大 {rp['max_up_per_cycle']:.3f}(上限 {rp['limit_per_cycle']:.3f})"
         f"は controller_server の 0 指令 {rp['n_injected_zero']} 回の直後だけ。"
         f"実速度が急に落ちて「実速度 + margin」で切られた下げ {rp['n_down_by_restart_cap']} 回。"
         f"違反 加速 {rp['n_up_violation']} / 減速 {rp['n_down_violation']} / 再加速 "
         f"{rp['n_restart_cap_violation']}、目標の行き過ぎ {rp['max_over_target']:.3f} m/s"),
        ("3. δ", verdict(dl["pass"]),
         f"最大 |δ| {dl['max_abs']:.4f} rad(steer_max {dl['steer_max']:.2f})。"
         f"元の MPPI は {dl['orig_max_abs']:.4f} rad(delta_max でクリップ)"),
        ("4a. 軌道の形", verdict(tr["pass_shape"]),
         f"NaN {'なし' if tr['finite'] else 'あり'}。始点のずれ最大 {tr['max_start_err_m']:.3f} m。"
         f"最大曲率 {tr['kappa_max']:.4f} 1/m(上限 {tr['kappa_limit']:.4f})"),
        ("4b. 経路への沿い方", verdict(True if follow and tr["dev_max_p95_m"] <= 0.5 else "warn"),
         f"車が経路上(0.3 m 以内)の {tr['n_on_path']} / {tr['n_eval']} 周期で、"
         f"最適軌道の経路からの最大距離 中央値 {tr['dev_max_median_m']:.2f} m / "
         f"95% {tr['dev_max_p95_m']:.2f} m"
         f"(元の MPPI {tr['orig_dev_max_median_m']:.2f} / {tr['orig_dev_max_p95_m']:.2f} m)"),
    ]


def main():
    out = Path(sys.argv[1])
    bags = [Path(b) for b in sys.argv[2:]]
    findings = json.loads((Path(__file__).with_name("findings.json")).read_text())
    sections, summary = [], []
    for bag in bags:
        d = bag / "analysis/bicycle"
        r = json.loads((d / "results.json").read_text())
        r["traj"]["pass_shape"] = bool(r["traj"]["finite"] and r["traj"]["max_start_err_m"] <= 0.15
                                       and r["traj"]["kappa_max"] <= r["traj"]["kappa_limit"] * 1.05)
        rs = rows(r)
        summary.append((r, rs))
        figs = "".join(
            f'<figure><figcaption><strong>{html.escape(t)}</strong>{html.escape(c)}</figcaption>'
            f'<div class="plot"><img src="{img(d / f)}" alt="{html.escape(t)}"></div></figure>'
            for f, t, c in FIGS if (d / f).exists())
        note = findings.get(r["bag"], {})
        sections.append(f'''
<section id="{r['bag']}">
  <h2>{r['bag']}</h2>
  <p class="meta">長さ {r['duration_s']:.1f} s / 走行 {r['moving_s'][0]:.1f}〜{r['moving_s'][1]:.1f} s /
  判定区間 {r['eval_s'][0]:.1f}〜{r['eval_s'][1]:.1f} s / 実速度の最大 {r['v_max_actual']:.2f} m/s</p>
  <p>{note.get('about', '')}</p>
  <div class="table"><table><thead><tr><th>確認項目</th><th>判定</th><th>数値</th></tr></thead><tbody>
  {''.join(f"<tr><td>{a}</td><td>{b}</td><td>{c}</td></tr>" for a, b, c in rs)}
  </tbody></table></div>
  {figs}
</section>''')

    head_rows = "".join(
        f"<tr><th>{lab}</th>" + "".join(f"<td>{rs[i][1]}</td>" for _, rs in summary) + "</tr>"
        for i, lab in enumerate(["1. target_speed", "2. v のランプ", "3. δ", "4a. 軌道の形",
                                 "4b. 経路への沿い方"]))
    bag_heads = "".join(f'<th><a href="#{r["bag"]}">{r["bag"]}</a></th>' for r, _ in summary)
    finding_items = "".join(f"<li><strong>{html.escape(f['title'])}</strong>{f['body']}</li>"
                            for f in findings["findings"])
    page = TEMPLATE.format(head_rows=head_rows, bag_heads=bag_heads, findings=finding_items,
                           sections="".join(sections), method=findings["method"],
                           repro=html.escape(findings["repro"]))
    out.write_text(page)
    print(out)


TEMPLATE = '''<title>自転車 MPPI bag 検証</title>
<meta name="claude:viewport-fit-sides" content="cover">
<style>
/* 計測ノート:左揃えの1段組み、表と図を同じ幅に揃える */
:root {{
  --bg: #f6f7f5; --panel: #ffffff; --fg: #1d2421; --muted: #5d6762; --line: #d9dfdb;
  --accent: #2a6fb8; --ok: #1d7a4b; --ok-bg: #e2f2e8; --warn: #8a5a00; --warn-bg: #fbefd2;
  --ng: #a3302c; --ng-bg: #f8e1df; --code-bg: #eef1ee;
  --f-head: "IBM Plex Sans JP", "Noto Sans JP", system-ui, sans-serif;
  --f-body: "Noto Sans JP", system-ui, sans-serif;
  --f-mono: "IBM Plex Mono", ui-monospace, monospace;
}}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{
  --bg: #161a18; --panel: #1e2321; --fg: #e5ebe7; --muted: #9aa6a0; --line: #343c38;
  --accent: #6aa7e8; --ok: #7fd3a3; --ok-bg: #1d3328; --warn: #f0c46b; --warn-bg: #3a2f17;
  --ng: #f19a94; --ng-bg: #3d2120; --code-bg: #232a27; color-scheme: dark; }} }}
:root[data-theme="dark"] {{
  --bg: #161a18; --panel: #1e2321; --fg: #e5ebe7; --muted: #9aa6a0; --line: #343c38;
  --accent: #6aa7e8; --ok: #7fd3a3; --ok-bg: #1d3328; --warn: #f0c46b; --warn-bg: #3a2f17;
  --ng: #f19a94; --ng-bg: #3d2120; --code-bg: #232a27; color-scheme: dark; }}
body {{ background: var(--bg); color: var(--fg); font-family: var(--f-body); line-height: 1.75;
  padding-left: max(16px, env(safe-area-inset-left, 0px));
  padding-right: max(16px, env(safe-area-inset-right, 0px)); padding-block: 32px 64px; }}
main {{ max-width: 980px; margin: 0 auto; display: grid; gap: 40px; }}
h1, h2, h3 {{ font-family: var(--f-head); text-wrap: balance; line-height: 1.35; margin: 0; }}
h1 {{ font-size: 1.9rem; font-weight: 600; }}
h2 {{ font-size: 1.35rem; font-weight: 600; padding-bottom: 6px; border-bottom: 1px solid var(--line); }}
p {{ margin: 0; max-width: 68ch; }}
section {{ display: grid; gap: 16px; }}
.meta {{ color: var(--muted); font-size: .9rem; font-variant-numeric: tabular-nums; }}
.eyebrow {{ font-family: var(--f-mono); font-size: .78rem; letter-spacing: .06em; color: var(--muted);
  text-transform: uppercase; }}
.table {{ overflow-x: auto; background: var(--panel); border: 1px solid var(--line); border-radius: 6px; }}
table {{ border-collapse: collapse; width: 100%; font-size: .9rem; font-variant-numeric: tabular-nums; }}
th, td {{ text-align: left; vertical-align: top; padding: 10px 12px; border-bottom: 1px solid var(--line); }}
tr:last-child th, tr:last-child td {{ border-bottom: 0; }}
thead th {{ font-family: var(--f-head); font-weight: 600; color: var(--muted); font-size: .82rem; }}
td:first-child, th:first-child {{ white-space: nowrap; }}
.pill {{ display: inline-block; font-size: .78rem; font-weight: 600; padding: 1px 10px; border-radius: 999px;
  white-space: nowrap; }}
.pill.ok {{ color: var(--ok); background: var(--ok-bg); }}
.pill.warn {{ color: var(--warn); background: var(--warn-bg); }}
.pill.ng {{ color: var(--ng); background: var(--ng-bg); }}
ul.findings {{ margin: 0; padding-left: 1.2em; display: grid; gap: 12px; max-width: 72ch; }}
ul.findings strong {{ display: block; font-family: var(--f-head); }}
figure {{ margin: 0; display: grid; gap: 8px; }}
figcaption {{ font-size: .88rem; color: var(--muted); max-width: 72ch; }}
figcaption strong {{ color: var(--fg); margin-right: .6em; }}
.plot {{ background: #fcfcfb; border: 1px solid var(--line); border-radius: 6px; padding: 8px; }}
.plot img {{ display: block; width: 100%; height: auto; }}
pre {{ background: var(--code-bg); border-radius: 6px; padding: 12px 14px; overflow-x: auto;
  font-family: var(--f-mono); font-size: .82rem; line-height: 1.6; margin: 0; }}
code {{ font-family: var(--f-mono); font-size: .9em; background: var(--code-bg); padding: 0 4px; border-radius: 3px; }}
a {{ color: var(--accent); }}
a:focus-visible {{ outline: 2px solid var(--accent); outline-offset: 2px; }}
</style>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400&family=IBM+Plex+Sans+JP:wght@600&family=Noto+Sans+JP:wght@400;600&display=swap">
<main>
<header style="display:grid;gap:10px">
  <p class="eyebrow">minicar / my_mppi_controller + speed_zone_publisher / 2026-11-09</p>
  <h1>自転車 MPPI bag 検証</h1>
  <p>PLAN.md Step 8 の検証。実走 bag を開ループで再生し、新しい2つのパッケージの出力
  (<code>/target_speed</code>、MPPI の指令と軌道)が、各時刻の自己位置に対してまともかを確かめた。</p>
</header>

<section>
  <h2>結果</h2>
  <div class="table"><table><thead><tr><th>確認項目</th>{bag_heads}</tr></thead>
  <tbody>{head_rows}</tbody></table></div>
</section>

<section>
  <h2>所見</h2>
  <ul class="findings">{findings}</ul>
</section>

<section>
  <h2>方法</h2>
  {method}
</section>

{sections}

<section>
  <h2>再現手順</h2>
  <pre>{repro}</pre>
</section>
</main>
'''

if __name__ == "__main__":
    main()
