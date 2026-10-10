"""vehicle_params.yaml から MPPI の旋回制約を導出する。ROS 非依存。

車両の舵角は minicar_bringup の vehicle_params.yaml が唯一の出所。
nav2_mppi_params.yaml に値を写すと、片方だけ直したときに MPPI の予測と
モータドライバの舵角クリップ(同じ delta_max を使う)が食い違う。
raceline_mppi.launch.py はこの関数の値で AckermannConstraints.min_turning_r を
上書きする。
"""

import math

import yaml


def min_turning_radius(vehicle_params_file):
    """bicycle model の最小旋回半径 wheelbase / tan(delta_max) [m]。"""
    with open(vehicle_params_file) as f:
        vp = yaml.safe_load(f)
    wheelbase = float(vp["vehicle"]["wheelbase"])
    delta_max = float(vp["limits"]["delta_max"])
    if not 0.0 < delta_max < math.pi / 2:
        raise ValueError(f"delta_max は (0, pi/2) の範囲であること: {delta_max}")
    return wheelbase / math.tan(delta_max)
