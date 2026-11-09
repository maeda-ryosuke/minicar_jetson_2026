# Nav2 MPPI 改造計画:自転車モデル+スピード固定

## 目的
Nav2 の MPPI コントローラをフォークし、次の2点を実現する。
1. 制御入力を **スピード v とステア角 δ** にし、自転車モデルで軌道を予測する
2. スピードを **トピックから受け取った値で固定** し、δ だけを最適化できるようにする(区間ごとの目標速度は別ノードが決め、加減速の制限は MPPI がかける)

## 前提
- ROS 2 Humble
- ベース:`nav2_mppi_controller`。コピー元は、インストール済みの `ros-humble-nav2-mppi-controller` と同じバージョンのタグにする(`dpkg -l` で確認)
- 配列演算は **xtensor**(Humble 版の既存コードに合わせる。Eigen には移行しない)
- 新パッケージ名:`my_mppi_controller`、プラグイン名:`my_mppi_controller::MPPIController`
- 出力:`geometry_msgs/TwistStamped`(controller_server の仕様のまま)
- モータドライバは `minicar_motor` をそのまま使う(舵ゲインの補正は済み)
- 速度ゾーンは `minicar_nav2_zone` を使わず、新パッケージ `speed_zone_publisher` で置き換える

## 方針
- **実装はできるだけ簡潔にする。計画に書いていないもの(警告・入力チェック・汎用化・追加のパラメータやオプション)は入れない。実装後にユーザーが確認する**
- 既存のデータ構造は変えず、**`wz` 系の枠をステア角 δ として読み替える**
  - `cvx`:スピード指令 v / `cwz`:ステア角指令 δ
  - `state.vx`:スピード / `state.wz`:`v·tan(δ)/L` で計算したヨーレート
  - ステア角の上限は `wz_max` を流用せず、新パラメータ `steer_max` で持つ
- 位置の積分(`integrateStateVelocities`)と critic は変更しない
- 差分は最小限にする。関係ない箇所のリファクタはしない
- ステップごとにビルドとテストを通し、コミットを分ける
- 新しく作るパッケージは2つ。役割を分ける
  - `my_mppi_controller`:`/target_speed` を購読し、v と δ を作る。エリアのことは知らない
  - `speed_zone_publisher`:自己位置とエリア定義(YAML)から、今の区間の目標速度を `/target_speed` に出す

## ファイル配置
```
jetson_humble/
├── PLAN.md
└── src/
    ├── my_mppi_controller/      # nav2_mppi_controller(humble)をコピーして改造
    │   ├── config/              # MPPI のパラメータ YAML
    │   └── launch/my_mppi.launch.py
    ├── speed_zone_publisher/    # 新規作成。自己位置とエリア定義から /target_speed を出す
    │   ├── config/zones.yaml    # エリア定義(MPPI 側は読まない)
    │   └── launch/speed_zone_publisher.launch.py
    └── minicar_bringup/
        └── launch/race_bicycle.launch.py   # 新規作成
```
- Nav2 本体(apt でインストール済みの `/opt/ros/humble`)は変更しない。controller_server などはそのまま使い、MPPI だけ自作パッケージに差し替える
- 既存の `race.launch.py` と `minicar_nav2/launch/nav2_mppi.launch.py` は変更しない
- コピー直後の状態を最初にコミットし、改造前との差分を `git diff` で確認できるようにする

## コメントのルール
- コメントは日本語で、人が読んで分かる簡潔な説明にする(何をしているか、なぜそうしたか)
- 追加・変更した箇所は、元のコードと区別できるように目印をつける
  - 追加:`// [ADD] 説明`
  - 変更:`// [MOD] 説明(元:元の処理を一言で)`
  - 数行にわたる場合は `// [ADD BEGIN] 説明` 〜 `// [ADD END]`(`MOD` も同様)
- 新規ファイル(新パッケージ含む)は、ファイル先頭に役割を2〜3行で書く。中の各行への目印は不要
- 例:
  ```cpp
  // [MOD] δ をヨーレートに変換して出力(元:wz をそのまま出力)
  cmd.twist.angular.z = v * std::tan(delta) / wheelbase_;
  ```

## 構成
```
自己位置(tf) → [speed_zone_publisher] → /target_speed(区間の目標速度)
    → [my_mppi_controller(controller_server)](加減速の制限)→ /cmd_vel_raw
    → [stall_stop_node] → /cmd_vel → [motor_driver] → 車両
```
- safety_node は通らない(safety_node は v だけを加速制限し、ω は素通しするので、ドライバで復元される δ がずれる)

## ステップ

### 1. フォークとリネーム
- パッケージ名、名前空間、pluginlib の XML を変更する
  - critic の pluginlib 登録名(`mppi::critics::*`)も新しい名前空間にする。apt 版と同じ名前で登録されるのを避けるため
- **完了条件**:ビルドが通り、元の MPPI と同じ設定で同じ挙動になる(乱数シードを固定して比べる)

### 2. 自転車モデルの追加
- `motion_models.hpp` に `BicycleMotionModel` を追加し、`motion_model: "Bicycle"` で選べるようにする
- `predict()` の中身:
  - `state.vx[t] = cvx[t-1]`
  - δ はサーボの1次遅れで追従:`δ += (cwz[t-1] - δ)·α`、`α = 1 - exp(-dt/τ)`
    - 1ステップの間 cmd が一定という条件での厳密解。オイラー法の `dt/τ` だと、model_dt 0.1・τ 0.08 で目標を行き過ぎる
  - その後、1ステップの変化量を `steer_rate_max·dt` で制限する
  - `state.wz[t] = state.vx[t]·tan(δ)/L`
- δ の初期値:遅れモデルで推定した δ を、周期をまたいで持ち越したもの(前回の指令そのものではない)
- パラメータ:`wheelbase`, `steer_time_constant`, `steer_rate_max`
- **完了条件**:単体テストで、δ=0 なら直進、δ一定なら半径 `L/tan(δ)` の円になる

### 3. ステア角の制約
- Bicycle モデルのとき、`steer_max` [rad] で `cwz` をクリップする
- `steer_max` は vehicle_params の `delta_max`(0.3003)以下にする。超えた分は motor_driver が切り捨てるので、予測と合わなくなる(README に書くだけにする)
- **完了条件**:`cwz` が ±`steer_max` を超えない

### 4. 出力の変換
- 指令を作る部分で `angular.z = v·tan(δ)/L` に変換する
- モータドライバ(v, ω 入力)は変更しない
- **確認済み**:motor_driver は `δ = atan(L·ω / v)` で、L は vehicle_params の値、v は受け取った指令値を使う(`motor_driver_core.py` の `steer_angle`)。stall_stop は通常時は素通し
  - safety_node を挟む経路では v だけが加速制限されて δ がずれるので、この構成では使わない
- **完了条件**:出力のヨーレートが、予測モデルと同じ式になっている

### 5. スピード固定モードの追加(MPPI 側)
- パラメータ `speed_mode`:`sampled`(元の挙動)/ `fixed_topic`
- `fixed_topic` のとき:
  - `target_speed_topic`(`std_msgs/Float32`、[m/s])を購読する。値は区間の目標速度
  - スピード方向のノイズを0にする(全サンプルが同じ v 列を使う)
  - 毎周期、目標速度に向けて加減速を制限した v の列を作り、制御列の v を上書きする(ウォームスタートで引き継がない)
    - 加速:`min(前回の出力, 実速度 + speed_restart_margin)` を起点に `accel_max` で上げる。実速度は computeVelocityCommands に渡される robot_speed を使う。発進・スタック復帰後・タイムアウトからの復帰も、この処理で扱える
    - 減速:前回の出力から `decel_max` で下げる
    - ホライズン内の v 列もこのランプで埋める。predict() は `state.vx[t] = cvx[t-1]` なので、軌道の計算にもランプが入る(predict() の変更は不要)
  - 一度も受信していないとき、または最後の受信から `target_speed_timeout` を超えたら v=0 にする(`/target_speed` が無ければ走らない。切り分けしやすくするため)
  - 目標速度の値と受信時刻は mutex で守る(購読と computeVelocityCommands は別スレッド)
- パラメータ:`accel_max`, `decel_max`, `speed_restart_margin`(stall_stop の `cmd_min` 0.1 より大きくする)
- `vx_min: 0.0`、`vx_max` はゾーン速度の最大値以上にする(Humble は cvx をこの範囲にクリップするので、ランプやタイムアウト時の 0 が潰れないように。設定だけで対応し、コードは変えない)
- `/speed_limit` と `setSpeedLimit()` には手を入れない
  - 注意:`/speed_limit` が届くと vx 系の上限が縮むので、この構成では出さない
- **完了条件**:単体テストで、
  - 目標速度を段差で変えたとき、v がランプ状に変わる
  - 実速度が低いときは「実速度 + margin」から加速し直す
  - 一度も受信していないとき、およびタイムアウトで0になる

### 6. エリア別速度ノードの追加(新パッケージ `speed_zone_publisher`)
- `minicar_nav2_zone` を置き換える(同時に起動しない)
- 入力:
  - tf(`map` → `base_link`)
  - エリア定義 YAML:`map` 座標の多角形と速度のリスト(どのエリアにも入らない場合の `default_speed` も持つ)
- 処理(50Hz):現在位置が入っているエリアの速度を `/target_speed`(`std_msgs/Float32`)に出力する
  - tf が取れないときは出力しない(MPPI 側がタイムアウトで止まる)
  - エリアが重なる場合は遅いほうを使う
  - 減速の先読みはしない。遅いエリアは、手前に `(v1² - v2²)/(2·decel_max)` だけ広げて描く(例:3.0→1.5、decel 2.0 なら 1.69 m)
- パラメータ:`zones_yaml`(`default_speed` は zones.yaml の中だけで持つ)
- エリアを RViz に Marker で表示する
- **完了条件**:単体テストで、エリア内なら設定速度、エリア外なら zones.yaml の `default_speed` になる

### 7. launch・設定例・ドキュメント
- 既存の `race.launch.py` と `nav2_mppi.launch.py` は変更しない
- パッケージごとに単体起動用の launch を作る
  - `my_mppi_controller/launch/my_mppi.launch.py`:controller_server と lifecycle_manager を起動する。`nav2_mppi.launch.py` と同じ書き方にし、`wheelbase` を vehicle_params から注入する(`nav2_mppi.launch.py` の `min_turning_r` と同じやり方)
  - `speed_zone_publisher/launch/speed_zone_publisher.launch.py`:speed_zone_publisher ノードを1つ起動する
- `minicar_bringup/launch/race_bicycle.launch.py` を新しく作る。`race.launch.py` をもとに、`nav2_mppi.launch.py` の include を上の2つの include に置き換える。その他(amcl・stall_stop・raceline・motor_driver)は同じ
- YAML の設定例と、追加したパラメータ・起動方法の説明を README に書く

```yaml
# MPPI(my_mppi_controller/config/)。wheelbase は launch が vehicle_params から注入する
FollowPath:
  plugin: "my_mppi_controller::MPPIController"
  motion_model: "Bicycle"
  steer_time_constant: 0.08
  steer_rate_max: 4.0
  steer_max: 0.30        # ステア角上限 [rad]。vehicle_params の delta_max 以下
  wz_std: 0.15           # ステア角のノイズ [rad]
  speed_mode: "fixed_topic"
  target_speed_topic: "/target_speed"
  target_speed_timeout: 0.3
  accel_max: 2.0         # [m/s^2]
  decel_max: 2.0         # [m/s^2] 惰性で出せる減速度以下
  speed_restart_margin: 0.5   # [m/s]
  vx_min: 0.0
  vx_max: 3.0            # ゾーン速度の最大値以上
```

```yaml
# エリア定義(speed_zone_publisher/config/zones.yaml)
default_speed: 1.0
zones:
  - name: straight_1
    speed: 3.0
    polygon: [[0.0, -1.0], [8.0, -1.0], [8.0, 1.0], [0.0, 1.0]]
  - name: corner_1
    speed: 1.5
    polygon: [[8.0, -1.0], [11.0, -1.0], [11.0, 3.0], [8.0, 3.0]]
```

### 8. bag 再生による検証とレポート
実走 bag `bags/20261105_fast4`(41 s)を再生し、新しい2つのパッケージの出力を確かめる。結果は図つきのレポートにまとめる。
- **方法**(開ループ):bag の自己位置とセンサを再生し、新しいノードを動かして出力を記録する
  - 再生するトピック:`/tf` `/tf_static` `/map` `/scan` `/odometry/filtered` `/odom`(`--clock`、`use_sim_time: true`)
  - 再生しないトピック:`/cmd_vel*` `/trajectories` `/transformed_global_plan` costmap 系(新しいノードが出し直す)
  - 起動するもの:raceline_manager(FollowPath を送る)、`my_mppi.launch.py`、`speed_zone_publisher.launch.py`
  - 記録するトピック:`/target_speed`、ゾーンの Marker、`/trajectories`(MPPI の MarkerArray)、`/transformed_global_plan`、`/cmd_vel_raw`
  - 注意:車は bag のとおりに動き、MPPI の出力には従わない。そのため、閉ループの追従性能は評価しない。見るのは、各時刻の自己位置に対して出力がまともかどうか
  - zones.yaml は、bag の地図に合わせた検証用のものを作る(minicar_nav2_zone の slow_zones の矩形を出発点にする)
- **確認すること**
  1. target_speed:bag の自己位置(`map → base_link`)と zones.yaml から、出るはずの速度をオフラインで計算し、記録した `/target_speed` と一致するか。エリア境界での切り替わりの遅れも見る
  2. MPPI の v:`/target_speed` に向けて `accel_max` / `decel_max` のランプになっているか
  3. MPPI の δ:`atan(ω·L/v)` で戻した δ が ±`steer_max` に収まっているか
  4. 軌道:最適軌道が自己位置から始まり、経路(`/transformed_global_plan`)に沿っているか。曲率が `tan(steer_max)/L` を超えていないか。NaN や途切れがないか。サンプル軌道の広がり方
  5. 参考:bag に記録されている元の MPPI の出力(`/cmd_vel_raw`、`/trajectories`)と並べて比べる
- **可視化**(matplotlib の静止画)
  - 地図・ゾーンの多角形・走行軌跡。軌跡は `/target_speed` の値で色分けする
  - 時系列:期待される target_speed / 記録した `/target_speed` / MPPI の出力 v / 実速度(`/odometry/filtered`)
  - 時系列:MPPI の δ と ±`steer_max`
  - 地図上のスナップショット:エリア境界とコーナーの数か所で、サンプル軌道・最適軌道・経路・自己位置を重ねる
- **置き場所**:スクリプトは `tools/bag_analysis/bicycle_replay/`、出力は `bags/20261105_fast4/analysis/bicycle/`
- **レポート**:上の図と、確認項目ごとの結果(○/×と数値)を HTML にまとめる
- **完了条件**:確認項目 1〜4 がすべて満たされ、その結果がレポートにまとまっている

## 実車での確認(人が行う)
1. パラメータを実測する
   - `steer_time_constant`:サーボが切れるまでではなく、指令曲率から実ヨー曲率までの応答で測る(10/26 の実測では約 0.26 s 遅れていた)
   - `accel_max` / `decel_max`:速度を段差で変える指令を出し、加速の様子と惰性での減速度を測る
   - `wheelbase`、ステア角の上限
2. 改造前後で比較する(同じコース・同じエリア速度)
   - 高速コーナーでのふくらみ(ラインからの横ずれの最大値)
   - 直線での蛇行(横ずれの振れ幅)
   - 速度の違うエリアで、同じ `wz_std` のまま挙動が安定しているか

## 今回やらないこと
- 動的自転車モデル(横滑り)
- δ̇ を振る方式、色付きノイズ
- 舵応答のむだ時間モデル(1次遅れで表せないと分かった場合に検討する)
- ホライズン内でエリアの境界をまたぐ予測(次のエリアの速度は予測に入れない)
- critic やプランナーの変更
