#!/usr/bin/env python3
"""PWM 出力のハード層。PCA9685 のカウント値を実際のアクチュエータへ渡す。

バックエンドを差し替え式にしてあるのは 2 つの理由から。

 1. **車体に通電せずに変換ロジックを検証したい。** backend:=dryrun なら
    PWM を出さずに /motor/pwm_count だけ流れるので、校正値と変換式を
    車輪を回さずに確かめられる。CI や開発機 (I2C が無い) でも動く。
 2. PWM 経路が変わった場合、差し替えるのはこのファイルだけで済み、
    motor_driver_core.py と motor_driver_node.py は変わらない。

=== なぜパルス幅[us] ではなくカウント値か ===

FaBo の JetRacer が実際に使っている経路に合わせたため。
FaBo のリポジトリには駆動系が 2 つあるが、

  * jetracer/nvidia_racecar.py … Adafruit ServoKit。**実走 Notebook から
    import されていない**(上流 NVIDIA 由来のクラスがそのまま残っているだけ)
  * Notebook 内の Fabo_PCA9685 直叩き … 04/05/17/40 の実走 Notebook が全部これ

実走側は次のとおり (notebooks/17_run.ipynb セル10、01_find_pwm.ipynb セル7):

    import Fabo_PCA9685, smbus
    bus = smbus.SMBus(BUSNUM)
    PCA9685 = Fabo_PCA9685.PCA9685(bus, INITIAL_VALUE, address=0x40)
    PCA9685.set_hz(60)
    PCA9685.set_channel_value(STEERING_CH, pwm_center)

つまり **60Hz・12bit カウント値**で、us 指定ではない。校正値 (pwm_params.json)
も中立 410 / 振れ幅 ±100 のようなカウント値で保存される。ここを us に翻訳して
持つと、実機で測った数字をそのまま入れられなくなる。

周波数が 50Hz ではなく 60Hz なのも効く。カウント値はデューティ比なので、
同じカウントでも周波数を変えるとパルス幅が比例して変わる。60 と 50 を
取り違えると舵角と速度が一様に 1.2 倍ずれる。

I2C バス番号はボードで違う (notebooks/17_run.ipynb セル2 の board_settings):

    JETSON_ORIN_NANO: 7   JETSON_ORIN: 7   JETSON_NANO: 1   JETSON_NX/XAVIER: 8

我々の Orin Nano は **7**。実機では `i2cdetect -y -r 7` に 0x40 が出るかで確定する
(同ノートの I2C 一覧: 0x08 プロポ値の吸い上げ / 0x3c OLED / 0x40, 0x70 PCA9685)。
"""

from __future__ import annotations

import math

# PCA9685 の分解能。12bit なので 0..4095。
COUNT_MAX_HW = 4095


class PwmBackend:
    """PWM カウント値を書き込む先。

    set_count は「その値を保持し続けろ」という意味。サーボも ESC も
    最後のパルスをラッチするので、呼ぶのを止めても出力は止まらない。
    止めたいときは中立を明示的に書く。
    """

    name = "base"

    def set_count(self, channel: int, count: int) -> None:
        raise NotImplementedError

    def close(self) -> None:
        """後始末。中立出力は呼び出し側 (ノード) の責務。"""


def _check(channel: int, count) -> int:
    """チャネルとカウント値を検証して整数にする。

    実機バックエンドと dryrun で同じ検証を通すために切り出してある。
    dryrun でだけ通る値があると、「シムで確認済み」が実機で嘘になる。
    """
    if not isinstance(count, (int, float)) or not math.isfinite(count):
        raise ValueError(f"PWM カウントが有限でない: ch={channel} count={count}")
    c = int(round(count))
    if not (0 <= c <= COUNT_MAX_HW):
        raise ValueError(
            f"PWM カウントが 12bit の範囲外: ch={channel} count={c} "
            f"(0..{COUNT_MAX_HW})")
    return c


class DryRunBackend(PwmBackend):
    """何も出力しない。直近値だけ保持する。

    通電せずに確かめるための既定。「出力したことになっている」と誤解しない
    よう、実機バックエンドと同じ検証 (チャネル・範囲) はここでも通す。
    """

    name = "dryrun"

    def __init__(self, logger=None):
        self._logger = logger
        self.last: dict[int, int] = {}

    def set_count(self, channel: int, count: int) -> None:
        self.last[int(channel)] = _check(channel, count)


class FaboPCA9685Backend(PwmBackend):
    """FaBo の Fabo_PCA9685 へ I2C で書く。実機専用。

    import を __init__ まで遅らせてあるので、ライブラリが入っていない環境では
    このクラスを作らない限り何も起きない。おかげで backend:=dryrun なら
    Fabo_PCA9685 も smbus も無いまま起動できる。
    """

    name = "fabo_pca9685"

    def __init__(self, bus: int, address: int, freq_hz: float,
                 initial_value: int, logger=None):
        try:
            import smbus
            import Fabo_PCA9685
        except ImportError as e:
            # 何を入れれば直るかをここで言い切る。実機で初めて踏む経路なので、
            # ImportError のまま投げると原因を追う時間が無駄になる。
            # 手順は FaBo の notebooks/98_setting.ipynb セル1/セル3 と同じ。
            raise ImportError(
                "fabo_pca9685 バックエンドには Fabo_PCA9685 と smbus が要る。実機で\n"
                "  git clone -b jupyterlab "
                "https://github.com/FaBoPlatform/FaBoPWM-PCA9685-Python\n"
                "  cd FaBoPWM-PCA9685-Python && pip3 install .\n"
                "  pip3 install smbus\n"
                f"を実行するか backend:=dryrun で起動すること ({e})"
            ) from e

        self._logger = logger
        self._bus = smbus.SMBus(int(bus))
        # 第2引数は初期値。中立(pwm_center / pwm_stop)を渡す。
        # FaBo の 01_find_pwm.ipynb が pwm_stop を渡しているのと同じ扱い。
        self._pca = Fabo_PCA9685.PCA9685(
            self._bus, int(initial_value), address=int(address))
        self._pca.set_hz(float(freq_hz))

    def set_count(self, channel: int, count: int) -> None:
        self._pca.set_channel_value(int(channel), _check(channel, count))

    def close(self) -> None:
        # PCA9685 は最後のデューティを保持する。中立を書くのはノード側の
        # 責務で、ここでやると二重管理になる。
        try:
            self._bus.close()
        except Exception:
            pass


def make_backend(kind: str, *, bus: int = 7, address: int = 0x40,
                 freq_hz: float = 60.0, initial_value: int = 410,
                 logger=None) -> PwmBackend:
    """名前からバックエンドを作る。未知の名前は落とす。

    既定へ黙って落とさないのは、実機で backend の綴りを間違えたときに
    「dryrun で走っているのに出力しているつもり」になるのが一番危ないため。
    """
    kind = str(kind).lower()
    if kind == "dryrun":
        return DryRunBackend(logger=logger)
    if kind == "fabo_pca9685":
        return FaboPCA9685Backend(bus=bus, address=address, freq_hz=freq_hz,
                                  initial_value=initial_value, logger=logger)
    raise ValueError(f"未知の backend: {kind!r} (dryrun / fabo_pca9685)")
