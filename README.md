# jetson_humble

Jetson実機上でTG30 LiDAR、ROS 2 Humble、SLAM Toolbox、Nav2 MPPI、
RealSense D455 + Isaac ROS cuVSLAMを動かすDocker環境。
Gazebo、RViz2、`robot_localization`はコンテナに含めない。

このディレクトリは自己完結しており、`minicar_gazebo/`を参照しない。

## パッケージ構成

`src/` 以下は colcon でビルドできる9つの `ament_python` パッケージ。

| パッケージ | 内容 |
| --- | --- |
| `minicar_scan` | TG30起動・設定・静的TF、SLAM用scanフィルタ |
| `minicar_realsense` | D455、取付静的TF、Isaac ROS cuVSLAMの単体bringup |
| `minicar_ftg` | FTG目標点生成と設定 |
| `minicar_mppi` | MPPIノード、ROS非依存コア、設定、オフラインテスト |
| `minicar_safety` | 指令監視・制限・停止・復帰と設定 |
| `minicar_bringup` | 統合launch、SLAM設定、共有車両諸元 |
| `minicar_nav2` | Nav2 MPPI設定とレースライン走行系の統合launch |
| `minicar_raceline` | CSVレースライン、FollowPath送信、ラップ管理 |
| `minicar_motor` | Twistから舵角・速度・PCA9685 PWMへの変換 |

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

## D455 + Isaac ROS cuVSLAM単体構成

この構成はJetson Linux R36.4.3（JetPack 6.2）、Jetson Orin、ROS 2 Humble、
Isaac ROS 3.2専用。既存の`jetson`サービスとは別の`vslam` profileで動かす。
VSLAM確認中は`jetson`、ホスト側`robot_localization`、LiDAR、SLAM Toolbox、Nav2を
停止し、cuVSLAMだけが`map -> odom -> base_link`をpublishするようにする。

Humble版Isaac ROS 3.2では、現行版の`tracking_mode=1`に相当する設定は
`num_cameras=2`と`enable_imu_fusion=true`である。入力は次の5トピックへ固定する。

| D455データ | cuVSLAM入力 |
| --- | --- |
| 左rectified IR | `/visual_slam/image_0` |
| 左CameraInfo | `/visual_slam/camera_info_0` |
| 右rectified IR | `/visual_slam/image_1` |
| 右CameraInfo | `/visual_slam/camera_info_1` |
| 統合IMU | `/visual_slam/imu` |

### 1. D455取付TFを設定

`src/minicar_realsense/config/camera_mount.yaml`に、実測した
`base_link -> camera_link`の並進[m]とroll/pitch/yaw[rad]を設定する。
測定後に`configured: true`へ変更する。全要素ゼロのidentityや未設定状態では、
誤った`base_link`オドメトリを防ぐためlaunchがエラー終了する。

### 2. Isaac ROSイメージをビルド

D455を接続するJetson上で実行する。スクリプトはIsaac ROS Common `v3.2-15`を
`.isaac_ros_common/`へ取得し、公式`ros2_humble.realsense`レイヤーの上に
`minicar_realsense`とIsaac ROS Visual SLAM 3.2を構築する。

```bash
cd ~/Docker/jetson_humble
./scripts/build_vslam_image.bash
docker image inspect minicar_vslam:3.2 >/dev/null
```

### 3. VSLAMだけを起動

ホストと同じDDS設定をexportし、D455をUSB 3.xポートへ接続して起動する。
preflightはaarch64、R36.4.3、6 GB以上のRAM、RealSense ROS 4.51.1、
librealsense 2.55.1、Isaac ROS 3.2、D455とUSB 3.x接続を検査する。

```bash
docker compose --profile vslam config
docker compose --profile vslam up vslam
```

別端末から入力、出力、TFを確認する。

```bash
docker compose --profile vslam exec vslam ros2 topic hz /visual_slam/image_0
docker compose --profile vslam exec vslam ros2 topic hz /visual_slam/image_1
docker compose --profile vslam exec vslam ros2 topic hz /visual_slam/imu
docker compose --profile vslam exec vslam ros2 topic hz /visual_slam/tracking/odometry
docker compose --profile vslam exec vslam ros2 topic echo /visual_slam/status
docker compose --profile vslam exec vslam ros2 run tf2_ros tf2_echo map odom
docker compose --profile vslam exec vslam ros2 run tf2_ros tf2_echo odom base_link
docker compose --profile vslam exec vslam ros2 topic info /tf -v
```

左右IRは約90 Hz、IMUは約200 Hzが目安。Odometryは`frame_id: odom`、
`child_frame_id: base_link`でなければならない。`/tf`の詳細表示で、
`map -> odom`と`odom -> base_link`のpublisherがcuVSLAM以外にも存在する場合は停止する。

同じROS Domainへ接続したRViz2端末で、同梱設定を使用する。
Jetson上でRVizを常用すると計測へ影響するため、可能なら別PCで表示する。

```bash
rviz2 -d "$(ros2 pkg prefix --share minicar_realsense)/rviz/d455_cuvslam.rviz"
```

停止時はVSLAMサービスだけを指定する。

```bash
docker compose --profile vslam stop vslam
docker compose --profile vslam rm -f vslam
```

## 既存TG30構成の責務

以下は`jetson`サービスで従来のTG30 + SLAM Toolboxを使う場合の構成であり、
上記`vslam`サービスとは同時に起動しない。ROS 2通信はDockerのhost networkを使う。
TFのpublisherは次の1箇所ずつにする。

```text
map ──> odom ──> base_link ──> laser_frame
  SLAM      host側             コンテナ側
 Toolbox   robot_localization   static TF
```

- Jetsonホスト
  - 既存の`robot_localization`をネイティブ実行する。
  - `/odometry/filtered`と動的TF `odom -> base_link`をpublishする。
  - `/imu/data`とIMUのセンサTFをpublishする。
- Dockerコンテナ
  - TG30から`/scan`をpublishし、`base_link -> laser_frame`の静的TFを配信する。
  - ホストの`/odometry/filtered`、`/imu/data`、`/tf`、`/tf_static`をsubscribeする。
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

Composeは同じ`jetson`コンテナ内でTG30、静的TF、SLAM用scanフィルタを自動起動する。
SLAM・MPPIは手動起動。`privileged`や別のLiDAR/GUIコンテナは使用しない。

```bash
docker compose logs -f jetson
docker compose exec jetson ros2 topic hz /scan
docker compose exec jetson ros2 topic echo /scan --once --qos-reliability best_effort
docker compose exec jetson ros2 run tf2_ros tf2_echo base_link laser_frame
```

目安は約10 Hz、`frame_id: laser_frame`、有効な距離データの継続受信。
TG30の設定は`src/minicar_scan/config/TG30.yaml`、取付TFは同ディレクトリの
`lidar_tf.yaml`。現在の設定は`base_link`基準でx=0.332 m、yaw=+78度（反時計回り）。
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
ros2 topic echo --once /odometry/filtered
ros2 topic echo --once /tf
ros2 topic echo /tf_static --qos-durability transient_local --once
ros2 run tf2_ros tf2_echo odom base_link
```

`/odometry/filtered`が見えても`tf2_echo odom base_link`が失敗する場合、ホスト側
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

## 5. SLAMで地図を作る

SLAM Toolboxは`/odometry/filtered`トピックを自己位置として直接使わず、scanの時刻における
`odom -> base_link` TFを参照する。最初にホストのEKFとコンテナのセンサを起動し、
次の入力が揃っていることを確認する。

```bash
docker compose exec jetson ros2 topic hz /scan_filtered
docker compose exec jetson ros2 topic echo /scan_filtered --once \
  --qos-reliability best_effort
docker compose exec jetson ros2 topic echo /odometry/filtered --once
docker compose exec jetson ros2 run tf2_ros tf2_echo odom base_link
docker compose exec jetson ros2 run tf2_ros tf2_echo base_link laser_frame
```

`/scan_filtered`は約10 Hzで、scanとodomの`header.stamp`が同じ実時間系であることを
確認する。`/tf`のpublisherも調べ、`odom -> base_link`を出す
`robot_localization`が1つだけであることを確認する。

```bash
docker compose exec jetson ros2 topic info /tf -v
```

端末1でmappingを起動する。

```bash
cd ~/Docker/jetson_humble
docker compose exec jetson bash -c \
  "ros2 launch minicar_bringup slam_mapping.launch.py"
```

通常は`enable_interactive_mode=false`で動作する。RVizからpose graphを手動修正する
場合に限り、`interactive_mode:=true`を付けて起動する。独自パラメータを試す場合は
絶対パスを`slam_params_file:=...`へ渡せる。

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

非常停止できる状態で手動操作し、0.2～0.3 m/s程度でコースを複数周する。
急旋回、車輪の空転、停止中に車体を持ち上げて移動する操作は避ける。最後は開始地点
付近まで戻り、ループ閉じ込み後にRViz上で次を確認する。

- 同じ壁が二重に描かれていない。
- ループ閉じ込み時に地図や`map -> odom`が誤った位置へ飛んでいない。
- `/scan_filtered`が地図上の壁と重なる。

地図が安定したら、車両を停止したまま同じbasenameで占有地図とpose graphを保存する。
以下では正本名を`track_v1`とする。

```bash
docker compose exec jetson bash -c \
  "ros2 run nav2_map_server map_saver_cli -f /maps/track_v1"
docker compose exec jetson bash -c \
  "ros2 service call /slam_toolbox/serialize_map \
   slam_toolbox/srv/SerializePoseGraph \"{filename: '/maps/track_v1'}\""

ls -lh maps/track_v1.pgm maps/track_v1.yaml \
  maps/track_v1.posegraph maps/track_v1.data
```

成果物はComposeのvolumeを通してホストの`jetson_humble/maps/`へ残る。
`track_v1.pgm`と`track_v1.yaml`は表示・将来のmap server用、
`track_v1.posegraph`と`track_v1.data`はSLAM Toolbox Localization用である。
Localizationの正本は後者2つなので必ず対で保管する。

## 6. 保存した地図で再局在化する

mappingプロセスを停止してからLocalizationを起動する。同時に起動すると双方が
`map -> odom`をpublishするため禁止する。`posegraph_file`には拡張子を付けない。

```bash
cd ~/Docker/jetson_humble
docker compose exec jetson bash -c \
  "ros2 launch minicar_bringup slam_localization.launch.py \
   posegraph_file:=/maps/track_v1"
```

任意位置から開始するときは、同じROS Domainに接続したRVizでFixed Frameを`map`にし、
地図上のおおよその車両位置と前方向を「2D Pose Estimate」で指定する。この操作は
`geometry_msgs/msg/PoseWithCovarianceStamped`を`/initialpose`へpublishする。指定後は
車両を低速で短距離動かし、scan matchingが収束するまで自動制御を開始しない。

```bash
docker compose exec jetson ros2 topic echo /slam_toolbox/pose
docker compose exec jetson ros2 run tf2_ros tf2_echo map odom
docker compose exec jetson ros2 run tf2_ros tf2_echo map base_link
docker compose exec jetson ros2 topic info /tf -v
```

RVizでscanと保存地図が重なり、静止中の`map -> base_link`が発散せず、
`map -> odom`のpublisherがSLAM Toolboxだけであることを確認してから制御系を起動する。
位置が収束しない場合は走行を開始せず、より正確な`/initialpose`を与え直す。

レースラインCSVは、この確認を終えた固定版pose graphの`map`座標で作成する。地図を
作り直した場合、同名で安易に上書きせず版を上げ、レースラインも再整合させる。

## 7. キーボード操作

`teleop_twist_keyboard`は`/cmd_vel`を直接publishする。実機走行系がこのトピックを
受けることと、非常停止手段を確認してから使用する。

```bash
docker compose exec jetson bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 8. Nav2 MPPI + Race Line

外部のEKFとSLAM Toolboxを起動し、`/odometry/filtered`と
`map -> odom -> base_link`が得られることを確認してから起動する。
既定の`backend:=dryrun`ではPWMを出さず、変換結果だけを確認できる。

```bash
docker compose exec jetson bash -c \
  "ros2 launch minicar_nav2 raceline_mppi.launch.py \
   raceline_file:=/maps/raceline.csv"
```

車輪を浮かせ、PWMと速度・操舵マップを校正した後だけ実機出力を有効にする。

```bash
docker compose exec jetson bash -c \
  "ros2 launch minicar_nav2 raceline_mppi.launch.py \
   raceline_file:=/maps/raceline.csv backend:=fabo_pca9685"
```

データフロー:

```text
Race Line CSV -> raceline_manager --FollowPath--> Nav2 MPPI
                                                    | /cmd_vel_raw
/scan, /imu/data -----------------------------> safety_node
                                                    | /cmd_vel
                                              motor_driver -> PWM

/odometry/filtered ---------------------------> Nav2 MPPI
map -> odom -> base_link ----------------------> Nav2 MPPI / raceline_manager
```

Nav2のローカルCostmapは`odom`座標系の空グリッドで、障害物レイヤと
Costmap系Criticは無効。障害物回避を追加するときは両方を同時に設定する。
旧`minicar_bringup mppi.launch.py`とは`/cmd_vel_raw`が競合するため同時起動しない。

## 9. 旧 FTG + 自作MPPI

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
/scan, /imu/data ────────────────────────────────> safety_node ──> /cmd_vel
/odometry/filtered ──────────────────────────────> MPPI
```

MPPI launchは`robot_localization`やTF publisherを起動しない。
