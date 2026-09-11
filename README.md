# jetson_humble

Jetson実機上でTG30 LiDAR、ROS 2 Humble、SLAM Toolbox、FTG/MPPIを動かすDocker環境。
Gazebo、RViz2、`robot_localization`はコンテナに含めない。

このディレクトリは自己完結しており、`minicar_gazebo/`を参照しない。

## パッケージ構成

`src/` 以下は colcon でビルドできる5つの `ament_python` パッケージ。

| パッケージ | 内容 |
| --- | --- |
| `minicar_scan` | TG30起動・設定・静的TF、SLAM用scanフィルタ |
| `minicar_ftg` | FTG目標点生成と設定 |
| `minicar_mppi` | MPPIノード、ROS非依存コア、設定、オフラインテスト |
| `minicar_safety` | 指令監視・制限・停止・復帰と設定 |
| `minicar_bringup` | 統合launch、SLAM設定、共有車両諸元 |

Docker内では外部パッケージ `ydlidar_ros2_driver` も `/ws/src` に取得してビルドする。
YDLidar-SDKとドライバのコミットはDockerfileで固定している。
`tg30_publisher/` は実行時・ビルド時とも不要。

Docker build 時に `/ws/src` を `colcon build` し、シェル起動時に
`/ws/install` を読み込む。コード・設定を変更したら
`docker compose up -d --build` で再ビルド・コンテナ再作成する。
地図だけを `/maps` に書き出し、ホストに保持する。

ROS 2 Humble と依存関係がある環境では、このディレクトリで
`source /opt/ros/humble/setup.bash && colcon build`、
`source install/setup.bash` によりDockerなしでも使用できる。
センサ起動には別途、Dockerfileと同じSDK・ドライバの導入が必要。

個別ノードは `ros2 run minicar_scan scan_filter_node` などで起動する。
FTG・MPPI・safetyの単独起動時は、共有車両諸元を明示する:

```bash
ros2 run minicar_mppi mppi_node --ros-args \
  --params-file "$(ros2 pkg prefix --share minicar_mppi)/config/mppi_params.yaml" \
  -p vehicle_params_file:="$(ros2 pkg prefix --share minicar_bringup)/config/vehicle_params.yaml"
```

統合launchでは車両諸元を自動で渡す。`vehicle_params_file:=/absolute/path/file.yaml`
で上書きできる。`use_sim_time` の既定値は `false`。
MPPIはFTGをimportせず、`PointStamped` トピックで目標点を受け取る。
SLAM用の `/scan_filtered`、FTGの `/ftg/scan_filtered` と
MPPIが障害物判定に使う生の `/scan` は区別する。

## 構成と責務

ROS 2通信はDockerのhost networkを使う。TFのpublisherは次の1箇所ずつにする。

```text
map ──> odom ──> base_link ──> laser_frame
  SLAM      host側             コンテナ側
 Toolbox   robot_localization   static TF
```

- Jetsonホスト
  - 既存の`robot_localization`をネイティブ実行する。
  - `/odom`と動的TF `odom -> base_link`をpublishする。
  - `/imu`とIMUのセンサTFをpublishする。
- Dockerコンテナ
  - TG30から`/scan`をpublishし、`base_link -> laser_frame`の静的TFを配信する。
  - ホストの`/odom`、`/imu`、`/tf`、`/tf_static`をsubscribeする。
  - SLAM Toolboxを起動したときだけ`map -> odom`をpublishする。
  - `robot_localization`や`odom -> base_link`の変換を起動しない。

## 1. ホストのDDS設定を確認

コンテナは、Composeを実行したシェルの設定を引き継ぐ。ホスト側ROS 2と同じ値を
exportしてからbuild・起動する。未設定時の既定値はDomain ID `0`、Fast DDS、
`ROS_LOCALHOST_ONLY=0`。

```bash
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}"

# 以下は例。ホストで別の値を使用している場合は、その値へ置き換える。
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

ホストがCyclone DDSを明示使用している場合は、次のように合わせる。イメージには
Fast DDSとCyclone DDSの両方をインストールしてある。

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 2. Buildと起動

Jetson上でbuildすることで公式`ros:humble`のARM64イメージが選ばれる。
TG30をUSB接続し、旧`tg30_publisher`とホスト側の同じLiDAR静的TFを停止しておく。
既定のホストポートは`/dev/ttyUSB0`。異なる場合は、以下の起動前に
`export LIDAR_DEVICE=/dev/serial/by-id/実際のデバイス名`などで指定する。
コンテナ内では常に`/dev/ttyUSB0`へマップする。

```bash
cd ~/Docker/jetson_humble
docker compose config
docker compose build
docker compose up -d
docker compose exec jetson bash
```

Composeは同じ`jetson`コンテナ内でTG30と静的TFを自動起動する。
SLAM・MPPIは手動起動。`privileged`や別のLiDAR/GUIコンテナは使用しない。

```bash
docker compose logs -f jetson
docker compose exec jetson ros2 topic hz /scan
docker compose exec jetson ros2 topic echo /scan --once --qos-reliability best_effort
docker compose exec jetson ros2 run tf2_ros tf2_echo base_link laser_frame
```

目安は約10 Hz、`frame_id: laser_frame`、有効な距離データの継続受信。
TG30の設定は`src/minicar_scan/config/TG30.yaml`、取付TFは同ディレクトリの
`lidar_tf.yaml`。TFは旧構成のx=0.2 m・回転なしを引き継いだ初期値で、実測値ではない。
変更後は`docker compose up -d --build`で反映する。

USB未接続ならComposeのデバイス割り当てが失敗する。ポートを開けない場合は
ドライバのログを確認する。ドライバが終了すると静的TFを含むlaunchも終了し、
`restart: unless-stopped`により再起動する。USBの再列挙後は
`docker compose up -d --force-recreate`でデバイスを再割り当てする。
停止には以下の`down`を使う。

コンテナを止める場合:

```bash
docker compose down
```

## 3. ARM64と依存関係を確認

コンテナ内で実行する。

```bash
uname -m                         # aarch64
dpkg --print-architecture        # arm64
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_map_server
ros2 pkg prefix teleop_twist_keyboard
ros2 pkg prefix ydlidar_ros2_driver
python3 -c "import numpy, scipy, yaml"

# 何も表示せず終了コード1なら、Gazeboは入っていない。
command -v gz
```

## 4. ホストROS 2との疎通確認

Jetsonホストで`robot_localization`とセンサノードを起動してから、コンテナ内で
次を実行する。環境変数を変えて再起動した場合は、古いROS 2 daemonを停止する。

```bash
ros2 daemon stop
ros2 topic list
ros2 topic echo --once /odom
ros2 topic echo --once /tf
ros2 topic echo /tf_static --qos-durability transient_local --once
ros2 run tf2_ros tf2_echo odom base_link
```

`/odom`が見えても`tf2_echo odom base_link`が失敗する場合、ホスト側
`robot_localization`の`publish_tf`、`odom_frame`、`base_link_frame`を確認する。
コンテナ側で`odom -> base_link`を追加して回避するとTFが重複するため行わない。

SLAMにはさらに、`/scan`の`header.frame_id`から`base_link`までのTFが必要。

```bash
ros2 topic echo --once /scan | grep frame_id
ros2 topic info /tf -v
ros2 topic info /tf_static -v
```

トピックが見えない場合は、ホストとコンテナの設定を比較する。

```bash
env | grep -E '^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_LOCALHOST_ONLY)='
```

ホスト側でも同じコマンドを実行し、値を合わせてからComposeを再作成する。

## 5. SLAM

端末1でSLAMを起動する。

```bash
cd ~/Docker/jetson_humble
docker compose exec jetson bash -c \
  "ros2 launch minicar_bringup slam_mapping.launch.py"
```

別端末でTFを確認する。

```bash
docker compose exec jetson bash -c \
  "ros2 run tf2_ros tf2_echo map odom"
docker compose exec jetson bash -c \
  "ros2 topic info /tf -v"
```

期待するTFは、SLAM Toolboxによる`map -> odom`と、ホスト側
`robot_localization`による`odom -> base_link`。後者のpublisherが複数ある場合は
SLAMを続行せず、重複しているpublisherを停止する。

地図を保存する。

```bash
docker compose exec jetson bash -c \
  "ros2 run nav2_map_server map_saver_cli -f /maps/jetson_map"
docker compose exec jetson bash -c \
  "ros2 service call /slam_toolbox/serialize_map \
   slam_toolbox/srv/SerializePoseGraph \"{filename: '/maps/jetson_map'}\""
```

成果物はホストの`jetson_humble/maps/`へ保存される。

## 6. キーボード操作

`teleop_twist_keyboard`は`/cmd_vel`を直接publishする。実機走行系がこのトピックを
受けることと、非常停止手段を確認してから使用する。

```bash
docker compose exec jetson bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 7. FTG + MPPI

> **警告:** 現在の車両・操舵・速度パラメータはシミュレーション由来の初期値で、
> 実機校正済みではない。最初にオフラインテストを実行し、その後は車輪を浮かせるか、
> 即時に非常停止できる状態で低速確認する。無人走行から開始しない。

オフラインテスト:

```bash
docker compose exec jetson bash -c "cd /ws && colcon test --packages-select minicar_mppi --event-handlers console_direct+ && colcon test-result --verbose"
```

実機パラメータを`src/minicar_bringup/config/vehicle_params.yaml`と
`src/minicar_mppi/config/mppi_params.yaml`へ反映後、
FTG + MPPI + safety chainを起動する。

```bash
docker compose exec jetson bash -c \
  "ros2 launch minicar_bringup mppi.launch.py"
```

データフロー:

```text
/scan ──> FTG ──> /ftg/target_point ──> MPPI ──> /cmd_vel_raw
                                                        │
/scan, /imu ─────────────────────────────────────> safety_node ──> /cmd_vel
/odom ───────────────────────────────────────────> MPPI
```

MPPI launchは`robot_localization`やTF publisherを起動しない。
