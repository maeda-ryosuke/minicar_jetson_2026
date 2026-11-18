"""地図・レースライン・速度エリアを重ねて PNG に出す。ROS も docker も要らない。

    python3 plot_zones.py [plot_zones.yaml]

レースラインは各点の目標速度(zones.py の target_speed)で色を付ける。
"""

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from speed_zone_publisher.zones import load_zones, target_speed  # noqa: E402


def main():
    cfg_path = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else
                               os.path.join(os.path.dirname(__file__), "plot_zones.yaml"))
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    path = {k: os.path.join(os.path.dirname(cfg_path), v) for k, v in cfg.items()}

    with open(path["map"]) as f:
        m = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(path["map"]), m["image"])))
    x0, y0 = m["origin"][:2]
    h, w = img.shape
    extent = [x0, x0 + w * m["resolution"], y0, y0 + h * m["resolution"]]

    default_speed, zones = load_zones(path["zones"])
    line = np.genfromtxt(path["raceline"], delimiter=",", names=True)
    speed = [target_speed(default_speed, zones, x, y) for x, y in zip(line["x"], line["y"])]

    fig, ax = plt.subplots(figsize=(12, 10))
    ax.imshow(img, cmap="gray", extent=extent)
    for z in zones:
        p = np.array(z.polygon)
        ax.fill(p[:, 0], p[:, 1], alpha=0.25)
        ax.text(*p.mean(axis=0), f"{z.name}\n{z.speed} m/s", ha="center", va="center", fontsize=9)
    sc = ax.scatter(line["x"], line["y"], c=speed, cmap="coolwarm_r", s=4)
    ax.plot(line["x"][0], line["y"][0], "k*", ms=12)  # 始点
    fig.colorbar(sc, ax=ax, label="target speed [m/s]")
    lx, ly = line["x"], line["y"]
    ax.set_xlim(lx.min() - 1, lx.max() + 1)
    ax.set_ylim(ly.min() - 1, ly.max() + 1)
    ax.set_aspect("equal")
    ax.set_title(f"{os.path.basename(path['raceline'])} / {os.path.basename(path['zones'])}"
                 f"  (default {default_speed} m/s)")
    fig.savefig(path["output"], dpi=120, bbox_inches="tight")
    print(path["output"])


if __name__ == "__main__":
    main()
