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

### 実車の最小旋回半径に合わせた修正（2026-10-14）

シムで作った線は実車の `r_min` 0.83m を考えておらず、ヘアピン 3 つが
R 0.62 / 0.62 / 0.78m で曲がり切れなかった（2 コーナー目 = 西ヘアピンで発覚）。
`tools/raceline/relax_corners.py` で、きつい区間の前後 2.5m だけを
SLAM 地図の壁から離しつつ **R 1.0m 以上**に引き直してある（`r_min` に 2 割の余裕。MPPI が修正舵を
切る余地を残すため）。上の図は修正前の線。

| | 修正前 | 修正後 |
| --- | --- | --- |
| 最小 R（0.25m で均した曲率） | 0.62m | 1.03m |
| 周長 | 35.5m | 37.5m |
| 動かした区間の壁余裕（車体中心） | — | 0.247m 以上 |

スタート直前（s≈37.4m）に数 cm だけ点ごとの曲率が 1.17 になる小さな S 字が
残っている（元の線のまま。0.25m で均すと R 1.03m）。

地図を作り直したり `r_min` を測り直したりしたら、このツールを掛け直すこと
（使い方はスクリプト冒頭）。

### ヘアピン1の壁余裕を均した修正（2026-10-16）

上の修正後も、ヘアピン1（南東、s 0.3〜9.5m）は入口で南の壁、出口で北の壁の
際を通っていた。実車は出口側で壁に当たって復帰できなかった。
`tools/raceline/balance_clearance.py` で、R 1.0m 以上を保ったまま東側の空きを
使って大回りさせ、南・東・北の壁から同じだけ離してある。区間の外は動かしていない。

| | 修正前 | 修正後 |
| --- | --- | --- |
| ヘアピン1の壁余裕（車体中心） | 0.25m（北） / 0.28m（南） | 0.32〜0.35m（南・東・北） |
| 最小 R（0.25m で均した曲率） | 1.03m | 1.02m |
| 周長 | 37.5m | 38.8m |

南の壁から北の壁まで約 2.85m しかないので、R 1.0m で回る限り 0.33m 前後が上限。

### 全周の引き直し（2026-10-23）

上の 2 つは部分修正なので、シムの線が持っていた小さな S 字や曲率の段差
（s≈2, 7, 15, 25〜28, 38m）と、壁際を通る区間（s≈12, 15, 20, 24, 29m）が
残っていた。速度を上げると舵を左右へ振り、コーナーで内側の壁の端を刈りにいく。
`tools/raceline/smooth_raceline.py` で全周をまとめて引き直し、
**`maps/raceline_smooth.csv` に別ファイルとして置いた**（`maps/raceline.csv` は
そのまま）。通路の中心線から始めて、曲率・曲率の変化率・壁との距離を同時に
最適化している（使い方と目的関数はスクリプト冒頭）。走らせるときは
`raceline_file:=/maps/raceline_smooth.csv` を渡す。

| | 修正前 | 修正後 |
| --- | --- | --- |
| 最小 R（0.25m で均した曲率） | 1.02m | 1.01m |
| 点ごとの最大曲率 | 1.18（R 0.85m） | 1.01（R 0.99m） |
| 曲率の変化の総量 ∫\|dκ/ds\|ds | 20.6 | 10.4 |
| 曲率の二乗積分 ∫κ²ds | 13.6 | 11.3 |
| 壁余裕の最小（車体中心） | 0.21m（s 29.1m） | 0.27m（s 28.8m） |
| 壁余裕 0.29m 未満の長さ | 2.2m | 0.3m |
| 周長 | 38.8m | 38.6m |

* オレンジ物体（支柱21-24）は LiDAR にほとんど写らず、地図の中段の壁が 1m ほど
  途切れている。`maps/track_v1_extra_walls.csv` で壁線を足してから最適化している。
* マットは `maps/track_v1_low_mu.csv`（シムの `surfaces.csv` を map へ載せたもの）
  で避けている。余裕もシムと同じ（水色 0.18m、灰色 0.03m）。
* ヘアピンは R 1.0m で回ると内外とも 0.3m 前後、s≈27〜30m の狭い通路は真ん中を
  通っても 0.28〜0.33m が上限（通路の形で決まる）。
* スタート姿勢 (-0.09, 0.02) からは 0.31m 北を通る（修正前は 0.12m）。

### センター寄せの線（2026-10-26）

`raceline_smooth.csv` は R 1.0m を稼ぐためにヘアピンで外へ膨らみ、灰色マットを
避けて外壁際を通っていた。舵の PWM 端点を広げて（500cb04）左右とも以前より切れる
ようになったのと、灰色マットは実走で踏んでも安定していたので、通路の真ん中を
通す線を **`maps/raceline_center.csv`** に別ファイルで置いた（smooth はそのまま）。

```bash
python tools/raceline/smooth_raceline.py --raceline maps/raceline_smooth.csv     --out maps/raceline_center.csv --allow-mat gray_mat --r-target 0.9     --clear-target 0.7 --w-clear 1e4 --w-dk 1000 --w-k 0.1
```

| | smooth | center |
| --- | --- | --- |
| 最小 R（0.25m で均した曲率） | 1.01m | 0.91m |
| 曲率の変化の総量 | 10.4 | 10.9 |
| 壁余裕の最小（車体中心） | 0.27m | 0.26m |
| 壁余裕 0.40m 未満の長さ | 11.1m | 4.0m |
| 周長 | 38.6m | 36.0m |

* 灰色マットは踏む（`--allow-mat gray_mat`）。水色マットは従来どおり避ける。
* 完全な中心線はヘアピンで R 0.62〜0.65m を要求し、r_min 0.83m では曲がれない。
  R 0.9m を下限に、取れる範囲で真ん中へ寄せてある。
* `vehicle_params.yaml` の r_min / MPPI の min_turning_r は 0.83m のまま、
  `steer_map` も端点を広げる前の実測。舵が切れるようになった分を線や MPPI に
  使うなら、r_min と steer_map を測り直す。

### 通路を直線にした線（2026-10-26）

`raceline_center.csv` を元に、下の通路（スタートを含む）・2 本目・3 本目の通路を
直線にし、上の通路は北の板 7-8（分岐の仕切り、東端 x≈-0.7, y≈4.42）の 1.7m 手前で
仕切りの下の通路の高さ（y≈4.08）に揃えた線を **`maps/raceline_straight.csv`** に
置いた（center はそのまま）。center は北東ヘアピンの出口から仕切りの端へ斜めに
寄っていき、実走で仕切りに当たることがあった。

```bash
python tools/raceline/smooth_raceline.py --raceline maps/raceline_center.csv     --out maps/raceline_straight.csv --allow-mat gray_mat --r-target 0.9     --clear-target 0.7 --w-clear 1e4 --w-dk 1000 --w-k 0.1     --straights maps/track_v1_straights.csv --w-straight 1e5
```

直線にする区間は `maps/track_v1_straights.csv`（map 座標の線分。`pull` 列が正の
区間だけ線分の上へ引き寄せる）。

| | center | straight |
| --- | --- | --- |
| 直線区間の最大曲率（下 / 2 本目 / 3 本目 / 上） | — | 0.03 / 0.12 / 0.09 / 0.10 |
| 線の y：x=1.0（仕切りの 1.7m 手前）/ x=0 / x=-0.7（仕切りの東端） | 4.32 / 4.16 / 4.09 | 4.10 / 4.08 / 4.08 |
| 最小 R（0.25m で均した曲率） | 0.91m | 0.91m |
| 曲率の変化の総量 | 10.9 | 10.0 |
| 壁余裕の最小（車体中心） | 0.26m | 0.26m |
| 周長 | 36.0m | 36.9m |

* 通路を直線にした分、西の大ループ（s 32 付近）と北東ヘアピンが少し外へ膨らむ。
* 上の狭い通路（幅 0.6〜0.7m）の壁余裕は 0.26〜0.30m で、通路の幅で決まる上限。

## 導入

```bash
# リポジトリのルート（例: ~/workspace/workspace_2026/minicar_jetson_2026）で
cd <リポジトリのルート> && docker compose up -d --build
```

`backend:=fabo_pca9685` 用の FaBo PCA9685 ライブラリと `smbus` は Dockerfile に、
`/dev/i2c-7` の割り当てと `src/minicar_motor/config` のマウントは
`docker-compose.yaml` に入っている。PWM の校正手順は `MOTOR_BENCH.md`。

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
