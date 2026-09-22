# jetson_src — 実機 (Jetson) 用パッケージ

`docs/LIDDER_MAPII.PNG` の構成図のうち **Race Line Manager** と **Motor Driver**。

| パッケージ | 内容 | 実行ファイル |
| --- | --- | --- |
| `minicar_raceline` | raceline CSV 読込 → FollowPath 送信 → ラップ管理 | `raceline_manager_node` |
| `minicar_motor` | `/cmd_vel` → 舵角/スロットル → PCA9685 の PWM | `motor_driver_node` |

## レースライン

![コース図の上に重ねたレースライン](../docs/raceline_on_drawing.png)

`maps/raceline.csv` をコース図（`docs/posts_cm.png`）の上に重ねたもの。
低ミューのマット（水色 / 灰色）とオレンジ物体を避けて通る線になっている。

壁と経路だけの図はこちら。

![壁とレースライン](../docs/raceline_map.png)

> 重ね描きは支柱 46 個の最小二乗で相似変換したもので、**図面そのものに
> 歪みがある**（支柱の残差 平均 14cm / 最大 46cm）。位置の確認用であって、
> この図から寸法を読まないこと。正となる座標は `maps/raceline.csv`。

CSV の列は `x, y, yaw` の3つだけ。単位は m と rad。

| 列 | 意味 |
| --- | --- |
| `x`, `y` | 位置。弧長 0.05m 等間隔。**座標系は `map`** |
| `yaw` | 進行方向 |

`s`（弧長）と `kappa`（曲率）は幾何から一意に決まるので列に持たせず、
`minicar_raceline/raceline_loop.py` が読み込み時に導出する。
`v_ref`（目標速度）も持たない（下の「速度」を参照）。

## 導入

```bash
cp -r jetson_src/minicar_raceline jetson_src/minicar_motor \
      ~/Docker/jetson_humble/src/
cd ~/Docker/jetson_humble && docker compose up -d --build
```

`backend:=fabo_pca9685` で実際に PWM を出す場合のみ、FaBo の
`notebooks/98_setting.ipynb` と同じライブラリが要る。

```bash
git clone -b jupyterlab https://github.com/FaBoPlatform/FaBoPWM-PCA9685-Python
cd FaBoPWM-PCA9685-Python && pip3 install .
pip3 install smbus
```

コンテナから I2C を叩くので `docker-compose.yaml` にデバイス割り当ても要る
（`--device /dev/i2c-7`、または `privileged` と `i2c` グループ）。
既定の `backend:=dryrun` ならどちらも不要。

## 起動

Nav2 MPPI、safety、Race Line Manager、motor driverを一括起動する通常経路:

```bash
# 観測のみ。PWMは出ない
docker compose exec jetson bash -c \
  "ros2 launch minicar_nav2 raceline_mppi.launch.py \
   raceline_file:=/maps/raceline.csv"

# 実機PWM（校正とI2C設定を完了し、必ず車輪を浮かせてから）
docker compose exec jetson bash -c \
  "ros2 launch minicar_nav2 raceline_mppi.launch.py \
   raceline_file:=/maps/raceline.csv backend:=fabo_pca9685"
```

`minicar_raceline`だけのlaunchは、controller serverとsafetyを別途起動する
デバッグ用途に使う。

```bash
# 観測のみ。PWM は出ない
docker compose exec jetson bash -c \
  "ros2 launch minicar_raceline raceline.launch.py raceline_file:=/maps/raceline.csv"

# PWM を実際に出す（必ず車輪を浮かせてから）
docker compose exec jetson bash -c \
  "ros2 launch minicar_raceline raceline.launch.py \
     raceline_file:=/maps/raceline.csv backend:=fabo_pca9685"
```

launch 引数:

| 引数 | 既定 | 内容 |
| --- | --- | --- |
| `raceline_file` | `/maps/raceline.csv` | 参照経路 CSV。**列は `x,y,yaw`、座標系は `map`** |
| `backend` | `dryrun` | `dryrun` / `fabo_pca9685` |
| `use_sim_time` | `false` | |
| `vehicle_params_file` | `minicar_bringup` の `vehicle_params.yaml` | |

個別に起動する場合:

```bash
ros2 run minicar_raceline raceline_manager_node --ros-args \
  --params-file "$(ros2 pkg prefix --share minicar_raceline)/config/raceline_params.yaml" \
  -p raceline_file:=/maps/raceline.csv

ros2 run minicar_motor motor_driver_node --ros-args \
  --params-file "$(ros2 pkg prefix --share minicar_motor)/config/motor_driver_params.yaml" \
  -p vehicle_params_file:="$(ros2 pkg prefix --share minicar_bringup)/config/vehicle_params.yaml" \
  -p pwm_params_file:="$(ros2 pkg prefix --share minicar_motor)/config/pwm_params.json"
```

**この launch は nav2 の `controller_server` と `safety_node` を起動しない。**
前者は nav2 側、後者は `minicar_safety` が持つ。起動する側を 1 箇所にしないと
同じノードが二重に上がり、`/cmd_vel` の publisher が複数になる。

## オフラインテスト（車両も I2C も不要）

```bash
docker compose exec jetson bash -c \
  "cd /ws && colcon test --packages-select minicar_motor \
   --event-handlers console_direct+ && colcon test-result --verbose"
```

## トピック

```text
                        TF map->base_link
                               |
raceline.csv --> [raceline_manager] --FollowPath--> controller_server(MPPI)
                               |                          | /cmd_vel_raw
                               +--> /raceline/lap (Int32) v
                                              [safety_node] --/cmd_vel-->
                                                     [motor_driver] --PWM--> 車両
```

`motor_driver` の観測用トピック（`backend:=dryrun` でも流れる）:

| トピック | 型 | 内容 |
| --- | --- | --- |
| `/motor/steer_angle` | `Float64` | 実際にサーボへ送った舵角 [rad] |
| `/motor/cmd_normalized` | `Float64MultiArray` | `[u_steer, u_throttle]` ∈[-1,1] |
| `/motor/pwm_count` | `Float64MultiArray` | `[steer, throttle]` の 12bit カウント |

## 校正

`minicar_motor/config/pwm_params.json` は暫定値で、**実車で測っていない**。

```json
{"pwm_steering": {"left": 310, "center": 410, "right": 510},
 "pwm_speed":    {"back": 390, "stop": 410,   "front": 430}}
```

FaBo の `notebooks/01_find_pwm.ipynb` で測り、出来た `pwm_params.json` を
そのまま上書きする（**キー構成が同じなので変換不要**）。
それまでは車輪を浮かせるか、即時に電源を切れる状態でしか走らせない。

`config/motor_driver_params.yaml` の `steer_map_*` / `throttle_map_*`
（舵角[rad] → u、速度[m/s] → u）も未校正。スロットルは上端を 0.50 に
抑えてあるので `v_max=2.0` を指令しても半分しか出ない。

初通電の前に確認する 3 点:

1. `i2cdetect -y -r 7` に `0x40` が出るか（**Orin Nano の I2C バスは 7**）
2. サーボの可動端と `delta_max` の対応
3. throttle に負値を入れて後退するか（既定 `reverse_mode` は FaBo の
   `set_back` と同じ「中立 → 0.1s → 中立 → 0.1s → 後退」）

**プロポ側が AI モードでないと車は動かない**（RC サーボマルチプレクサ）。
逆に言えば暴走したらプロポで切れる。初通電のときは必ずプロポを手元に置く。

## 速度

**全周 `vx_max` 固定**（nav2 の `FollowPath.vx_max`）。raceline CSV に `v_ref` 列は
無く、`/speed_limit` も出さない。**コーナーで自動的に減速しない。**
`vx_max` は実走で決める。
