# モータドライバ 実機テスト手順

`/cmd_vel` → `motor_driver` → PCA9685 → サーボ / ESC が正しく動くかを実機で確かめる。
道具は `motor_bench`（`src/minicar_motor`）。

| 部 | 内容 | 車輪 | 所要 | 必須か |
| --- | --- | --- | --- | --- |
| 準備 | ビルド・ライブラリと I2C の確認 | — | 10 分 | 必須 |
| 第1部 | PWM 6 点の決定と動作確認 | **浮かせる** | 30 分 | 必須 |
| 第2部 | 舵角マップ・速度マップの校正 | 接地（舵角は浮かせても可） | 1〜2 時間 | 後日でよい |

**第1部が終われば、ドライバが正しく動くことは確認できる。** 指令どおりの向きに
舵が切れ、車輪が回り、止めれば止まる。これは車輪を浮かせたままで確認できる。

第2部をやらないと困るのは、「0.5 m/s を指令したら実際は 0.8 m/s 出ていた」の
ように**指令と実際の量が合わない**ことだけ（MPPI の追従が甘くなる）。暫定マップは
スロットル上限を ±0.5 に抑えてあるので、低速で走らせる分には危なくない。
実際に走らせるようになってから、必要に応じて測ればよい。

**プロポを手元に置き、AI モードにしておく。** FaBo の基板は RC サーボ
マルチプレクサでプロポ手動と AI を切り替える。プロポ側が AI モードでないと
何を出しても車は動かず、逆に**暴走したらプロポで切れば止まる**。
ソフトではなくこれが最後の砦。

---

## 準備

### 0-1. ビルドと起動（初回とコード更新時のみ）

```bash
# Jetson ホストで、リポジトリのルート（例: ~/workspace/workspace_2026/minicar_jetson_2026）へ移動
cd <リポジトリのルート>
git pull
docker compose up -d --build
```

- `jetson` コンテナは起動時に `sensors.launch.py`（TG30 LiDAR）を自動起動する。
  **LiDAR の USB が挿さっていないとコンテナが起動しない**ので、ベンチ中も挿しておく
- FaBo の JupyterLab など、PCA9685 を触る別プロセスは止めておく
  （両方が書き込むと出力を取り合う）

### 0-2. FaBo ライブラリの確認

FaBo の PCA9685 ライブラリ・`smbus`・`i2c-tools` は Dockerfile に、
`/dev/i2c-7` の割り当ては `docker-compose.yaml` に入っている。入っていることだけ確かめる。

```bash
# Jetson ホストで
docker compose exec jetson python3 -c "import Fabo_PCA9685, smbus; print('ok')"
```

`ok` が出なければ、イメージが古い。`docker compose up -d --build` をやり直す。
起動中のコンテナへ手で `pip3 install` しない（再作成で消える）。

### 0-3. 校正ファイルはホストと共有されている

`src/minicar_motor/config/` はコンテナの
`/ws/install/minicar_motor/share/minicar_motor/config/` にマウントしてある。

- `motor_bench pwm` の `save` は**ホストの `src/minicar_motor/config/pwm_params.json` を直接上書き**する
  （`git diff` で確認・コミットできる）
- `motor_driver_params.yaml` はホスト側で編集し、**ドライバを再起動するだけで反映**される（再ビルド不要）
- `raceline_mppi.launch.py` などの既存 launch も同じファイルを読むので、校正後そのまま使える

### 0-4. 端末の開き方（以降すべての端末で共通）

`docker compose exec jetson bash -c "..."` は使わず、コンテナのシェルに入ってから打つ。
変数はコンテナ内で定義する（ホストで定義しても `bash -c` の中には届かない）。

```bash
# Jetson ホストで、リポジトリのルートへ移動
cd <リポジトリのルート>
docker compose exec jetson bash

# ここからコンテナ内
VP=$(ros2 pkg prefix --share minicar_bringup)/config/vehicle_params.yaml   # 車両諸元
CFG=$(ros2 pkg prefix --share minicar_motor)/config                        # = ホストの src/minicar_motor/config
```

以下のコマンドは、特に書いていない限りこの状態のコンテナ内で打つ。

### 0-5. 通電前チェック

1. プロポが AI モードで、手元にあること
2. I2C に PCA9685 が見えること。**Orin Nano のバスは 7**（Nano=1, NX/Xavier=8）。
   コンテナ内で打つので、コンテナから見えることまで一度に確認できる。

   ```bash
   i2cdetect -y -r 7      # 0x40 が出ること
   ```

   FaBo の `17_run.ipynb` による I2C 一覧: `0x08` プロポ値の吸い上げ /
   `0x3c` OLED / `0x40`, `0x70` PCA9685。

3. `/cmd_vel` を出すものが上がっていないこと。
   `raceline_mppi.launch.py` / `mppi.launch.py` / `teleop_twist_keyboard` は起動しない
   （publisher が複数になって指令が混ざる。`motor_bench` も起動時に数えて警告する）

---

## 第1部: 動作確認（車輪を浮かせる）

車体を台に載せ、車輪が空転できる状態にする。浮かせていても、スロットルを急に
上げると車体が台から跳ねる。**値は小さい方から上げる。**

### 1-1. PWM 6 点を決める（端末1）

FaBo の `notebooks/01_find_pwm.ipynb` に相当する段。**このツールは ROS を
使わない**ので、ドライバより先に実行できる。

```bash
ros2 run minicar_motor motor_bench pwm --backend fabo_pca9685 --bus 7 \
  --out $CFG/pwm_params.json
```

| 入力 | 動作 |
| --- | --- |
| `s` / `t` | 操舵チャネル / スロットルチャネルへ切替 |
| `420` | そのカウントを書く |
| `+10` `-5` | 現在値からの相対で書く |
| `left` `center` `right` | 今のカウントを操舵の 3 点として記録 |
| `back` `stop` `front` | 今のカウントをスロットルの 3 点として記録 |
| `show` | 記録済みの点と現在値 |
| `save` | JSON へ書き出す（既存は `.bak` へ退避） |
| `q` | 中立を書いて終了 |

起動時と終了時に必ず中立を書く。PCA9685 は最後のデューティを保持するので、
書かずに終わると舵を切ったまま／走ったままになる。

```
中立を書いた: steer=410 throttle=410
[steer ch0] count=410 記録済み=- > +10
[steer ch0] count=420 記録済み=- > +10
[steer ch0] count=430 記録済み=- > center
  steer.center = 430 を記録
```

`+10` を打つたびにサーボが動くので、**タイヤを見ながら**進める。

**操舵（`s`）— 中立 → 左端 → 右端 の順**

1. まっすぐを向く値で `center`
2. 左へ振っていき、**タイヤが止まったのにサーボが唸り出す手前**で 1 段戻して `left`
3. 同じく右で `right`

- 機械端に当てたまま保持するとギアを舐める。唸ったら必ず戻す
- 左右の振れ幅は `center` に対して非対称でよい（実測中立を折れ点に使う設計なので、
  `left` と `right` の中点である必要はない）
- **サーボの極性が逆なら `left` と `right` を入れ替えて記録する。**
  `left > right` でも保存できる（FaBo の Reverse チェックボックス相当）

**スロットル（`t`）** — 切り替え時に一度だけ確認が入る。

```
  スロットルを触る。車輪は浮いているか? [y/N]
```

- `stop` は「車輪が回り出さない上限」ではなく**完全に止まる値**
- `front` / `back` は低速側でよい。全開を記録する必要はない
  （記録した値が u=±1 の基準になるので、大きく取るほど指令が過敏になる）

**保存** — `save` は書き出す前に `motor_driver` と同じ検証を通すので、
**起動できない値は保存されない。** 拒否されるとその場で理由が出る。

```
  保存しない: pwm_steering の中立 520 が端点 310..410 の間に無い
```

通れば `.bak` を残して上書きする。`q` で中立を書いて終了。
ホスト側で `git diff src/minicar_motor/config/pwm_params.json` を見て、値が入ったことを確認する。

### 1-2. ドライバを起動する（端末1、以降起動しっぱなし）

```bash
ros2 run minicar_motor motor_driver_node --ros-args \
  --params-file $CFG/motor_driver_params.yaml \
  -p vehicle_params_file:=$VP \
  -p pwm_params_file:=$CFG/pwm_params.json \
  -p backend:=fabo_pca9685 -p use_sim_time:=false
```

起動直後**2 秒はアーミング**（ESC に中立を認識させる）で指令が通らない。
ログに `アーミング完了` が出てから次へ進む。
`backend=dryrun` の警告が出ていたら、PWM は出ていない（`-p backend:=` を見直す）。

### 1-3. 動かして確かめる（端末2）

`motor_bench cmd` は実行前に何が起きるかを表示し、Enter で動き出す。
コマンドはすべて次の形で、表の「引数」を後ろに付ける。

```bash
ros2 run minicar_motor motor_bench cmd --vehicle-params-file $VP <引数>
```

| # | 確認 | 引数 | 正しい動き | 違ったら |
| --- | --- | --- | --- | --- |
| a | 中立 | `--v 0.0 --delta 0.0 --duration 3` | 車輪が回らず、舵がまっすぐ | 1-1 の `stop` / `center` をやり直す |
| b | 舵の向き | `--v 0.3 --sweep 0.2,-0.2 --duration 3` | 1 点目で左、2 点目で右に切れる | `pwm_params.json` の `left` と `right` を入れ替える |
| c | 前進 | `--v 0.3 --delta 0.0 --duration 3` | 前進方向に回る | `front` と `back` を入れ替える |
| d | 後退 | `--v -0.3 --delta 0.0 --duration 3` | 後退方向に回る | 下記 |

- b で `--v` を 0 にしないのは、v≈0 の Twist には舵角の情報が乗らないため
  （omega = v·tan(delta)/L。ドライバは直前の舵角を保持する）
- `pwm_params.json` や `motor_driver_params.yaml` を直したら、**端末1 のドライバを
  Ctrl-C → 再起動**する

**d（後退）について。** 既定の `reverse_mode: "neutral_neutral_reverse"` は FaBo の
`set_back` と同じ「中立 → 0.1s → 中立 → 0.1s → 後退」を自動で踏む。TT-02 系の ESC は
前進から直接後退へ行けないため。端末1 のログに
`後退シーケンス開始` → `後退シーケンス完了` が出る。

- **逆回転すれば既定のまま**
- ブレーキがかかるだけで回らないなら、端末1 のドライバに
  `-p reverse_mode:=direct` を足して再起動して試す
- どちらでもだめなら `disabled`。ただし `safety_node` のスタック脱出
  （`recovery_speed` を負にして出す）が効かなくなるので、その旨を
  `src/minicar_safety/config/safety_params.yaml` 側にも書く

決まった値は `motor_driver_params.yaml` の `reverse_mode` に書く。

### 1-4. 送信側が落ちたら止まるか（端末2・端末3）

上流が落ちたときにドライバ側で止まることを見る。**Ctrl-C ではなく `kill -9`** で殺す
（Ctrl-C だと `motor_bench` が停止指令を出して終わるので、ドライバ側の
ウォッチドッグを試したことにならない）。

```bash
# 端末2（コンテナ内）で走らせておく
ros2 run minicar_motor motor_bench cmd --vehicle-params-file $VP \
  --v 0.3 --duration 30 --yes

# 端末3 — Jetson ホストから殺す（コンテナ内のプロセスもホストから見える）
sudo pkill -9 -f motor_bench
```

`cmd_timeout`（既定 0.3 s）以内に車輪が止まり、端末1 のログに
`cmd stale ... 中立を出力` が出ること。

**ここまで通れば第1部は完了。** `src/minicar_motor/config/` の変更をコミットしておく
（`save` が作る `*.bak` は `.gitignore` 済み）。

---

## 第2部: マップの校正（後日でよい）

`motor_driver_params.yaml` の暫定マップを実測値に置き換える。
端末1 のドライバは第1部と同じ手順で起動しておく。

`motor_bench cmd` は 1 点ごとに記録用の行を出す。**マップに書くのは指令値ではなく
この u と、測った実際の量。**

```
  記録用: delta_cmd=+0.4200 u_steer=+0.8000 u_throttle=+0.5000  R_measured=____ m
```

### 2-1. 舵角マップ

`steer_map_delta_rad` に実舵角、`steer_map_u` に対応する `u_steer` を、
**delta の昇順**で並べる（狭義単調増加でないと起動時に落ちる）。
実舵角の測り方は 2 通りある。

**簡易法（浮かせたまま）** — 各点でタイヤの切れ角を分度器かスマホの角度計で直接測る。
アッカーマンで内輪と外輪の角度が違うので、左右の平均を取る。下の方法より精度は劣る。

```bash
ros2 run minicar_motor motor_bench cmd --vehicle-params-file $VP \
  --v 0.3 --duration 5 \
  --sweep 0.42,0.30,0.20,0.10,0,-0.10,-0.20,-0.30,-0.42
```

**円を描かせる方法（接地）** — 一定舵で円を描かせ、床にチョークで半径 R を測る。

```bash
ros2 run minicar_motor motor_bench cmd --vehicle-params-file $VP \
  --v 0.4 --duration 8.0 \
  --sweep 0.42,0.30,0.20,0.10,0,-0.10,-0.20,-0.30,-0.42
```

1 点ごとに止まって、何が起きるかを読み上げる。チョークを置いて Enter。

```
=== 1/9 ===
  v     = +0.40 m/s
  delta = +0.420 rad (+24.1 deg)
  omega = +0.695 rad/s
  時間  = 8.0 s
  -> 旋回半径 R = 0.58 m (直径 1.15 m / 円周 3.62 m)
  -> 走行距離 3.20 m = 0.88 周
  車輪を接地させる場合、3.2 m 四方の空間と、即時に電源を切れる体勢が要る
  測ったら: delta_actual = atan(0.257/R_measured)
  Enter で実行 (s でスキップ, q で中止) >
```

```
delta_actual = atan(L / R_measured)        L = 0.257 m
```

小舵角ほど円が大きい。**場所が足りない点は `s` で飛ばす。**
飛ばした点は外挿しない（テーブル範囲外は端点でクランプされる）。

| delta [rad] | R [m] | 必要な直径 [m] |
| --- | --- | --- |
| 0.42 | 0.576 | 1.2 |
| 0.30 | 0.831 | 1.7 |
| 0.20 | 1.268 | 2.6 |
| 0.10 | 2.561 | 5.2 |
| 0.05 | 5.136 | 10.3 |

**記録シート**

| delta_cmd [rad] | u_steer | 向き | R_measured [m] または実測角 | delta_actual [rad] |
| --- | --- | --- | --- | --- |
| +0.42 | | 左 | | |
| +0.30 | | 左 | | |
| +0.20 | | 左 | | |
| +0.10 | | 左 | | |
| 0.00 | | 直進 | ∞ | 0.000 |
| −0.10 | | 右 | | |
| −0.20 | | 右 | | |
| −0.30 | | 右 | | |
| −0.42 | | 右 | | |

「u を少しずつ増やしても舵が切れない（車がまっすぐ走る）」区間があれば、
それがデッドバンド。その端を折れ点に足す。左右非対称なら 0 の左右で傾きが変わる。

### 2-2. 速度マップ（接地が必須）

空転と接地では負荷が違い回転数が変わるので、床で測るしかない。
チョークでスタート線とゴール線を 3 m 空けて引き、通過時間を測る。

```bash
ros2 run minicar_motor motor_bench cmd --vehicle-params-file $VP \
  --delta 0.0 --duration 6.0 --v 0.5
```

- `--v` を小さい方から変えて繰り返す（0.3 / 0.5 / 0.8 / 1.0 / 1.5 / 2.0）
- **助走を取る。** 加速中の区間を含めると定常速度にならない。スタート線の手前から
  走らせ、線を跨いだ時刻で測る。**直線 5 m 以上**が要る
- このツールは `safety_node` を通さないので加速度制限が無い。**`--v` を急に上げない**

| v_cmd [m/s] | u_throttle | 距離 [m] | 時間 [s] | v_actual [m/s] |
| --- | --- | --- | --- | --- |
| 0.3 | | 3.0 | | |
| 0.5 | | 3.0 | | |
| 0.8 | | 3.0 | | |
| 1.0 | | 3.0 | | |
| 1.5 | | 3.0 | | |
| 2.0 | | 3.0 | | |

`throttle_map_v_mps` に `v_actual`、`throttle_map_u` に `u_throttle` を昇順で並べる。
現状の `throttle_map` は上端 ±0.5 で頭打ちなので、**v=2.0 を指令しても半分の
スロットルしか出ない**。速い側を埋めるときにここを上げる。
v=0 付近で「u を上げても動き出さない」区間が ESC のデッドバンドで、その端を折れ点に足す。

### 2-3. 受け入れ確認（接地）

マップを実測値へ差し替え、**端末1 のドライバを Ctrl-C → 再起動してから**行う。

```bash
# 1. 直進 2 m: v=0.5 を 4 秒 -> 2.0 m 進むこと
ros2 run minicar_motor motor_bench cmd --vehicle-params-file $VP \
  --v 0.5 --delta 0.0 --duration 4.0

# 2. 旋回 R=0.83 m: delta=0.30 の円が直径 1.66 m になること
ros2 run minicar_motor motor_bench cmd --vehicle-params-file $VP \
  --v 0.4 --delta 0.30 --duration 8.0
```

ここがずれるなら、ずれているのはマップであってドライバではない。測定点を増やす。

---

## 測った値の置き場所

パスはすべてホスト側で、リポジトリのルートからの相対パス。

| 測ったもの | 書く先 | 反映方法 |
| --- | --- | --- |
| PWM 6 点 | `src/minicar_motor/config/pwm_params.json`（`save` が書く） | ドライバ再起動 |
| 後退方式 | `src/minicar_motor/config/motor_driver_params.yaml` の `reverse_mode` | ドライバ再起動 |
| 実舵角 ↔ u_steer | 同 `steer_map_delta_rad` / `steer_map_u` | ドライバ再起動 |
| 実速度 ↔ u_throttle | 同 `throttle_map_v_mps` / `throttle_map_u` | ドライバ再起動 |
| ホイールベース・トレッド | `src/minicar_bringup/config/vehicle_params.yaml`（メジャーで実測したら更新） | `docker compose up -d --build` |

`vehicle_params.yaml` はマウントしていないので、変えたときだけ再ビルドが要る。

**車両諸元を `motor_driver_params.yaml` へ書き写さない。**
`vehicle_params.yaml` が唯一の出所で、`vehicle_params_file` で渡す。

---

## やってはいけないこと

- **マップを直したのにドライバを再起動しない。** `motor_driver_params.yaml` は
  起動時に一度読むだけで、走っているノードには反映されない
- **接地状態で `--yes` を使う。** 確認を飛ばすので、意図せず走り出す
- **測定中に `safety_node` や nav2・MPPI を上げる。** `/cmd_vel` の publisher が
  2 つになって指令が混ざる。`motor_bench` は起動時に publisher 数を数えて
  警告するが、止めるのは手作業
- **`--v` を急に上げる。** このツールは `safety_node` を通さないので
  加速度制限が無い

---

## オフライン確認（実機も I2C も不要）

通電前に、ツール自身が壊れていないことを確かめられる。コンテナ内で:

```bash
# 純関数の単体テスト
cd /ws && colcon test --packages-select minicar_motor \
  --event-handlers console_direct+ && colcon test-result --verbose

# PWM 校正ツールを dryrun で通す（PWM は出ない。操作を覚えるのに使える）
# 本物の pwm_params.json を上書きしないよう /tmp へ書く
ros2 run minicar_motor motor_bench pwm --out /tmp/pwm_params.json
```

> `test_motor_driver_core.py` の一部は `pytest` から見ると引数が fixture 扱いに
> なり collection error で落ちる。`python3` で直接叩けば 12 件すべて通る。
> `test_motor_bench.py` は `colcon test` でも直接実行でも通る。

---

## 別解: 1-1 を FaBo の Notebook でやる

Jetson に FaBo の JupyterLab 環境が残っていれば、`notebooks/01_find_pwm.ipynb` で
6 点を測ってもよい。出力される JSON はキー構成が同じなので、
`src/minicar_motor/config/pwm_params.json` にそのまま置けば使える。
ドライバを起動する前に、**Notebook のカーネルは必ず止める**（PCA9685 を取り合う）。
