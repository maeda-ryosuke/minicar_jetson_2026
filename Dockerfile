FROM ros:humble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Jetson 実機用。Gazebo / GUI / robot_localization はコンテナへ入れず、
# ホスト上のセンサ・オドメトリ推定ノードと ROS 2 DDS で通信する。
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-numpy \
        python3-scipy \
        python3-yaml \
        python3-colcon-common-extensions \
        python3-pytest \
        ros-humble-ament-index-python \
        ros-humble-geometry-msgs \
        ros-humble-launch-ros \
        ros-humble-nav-msgs \
        ros-humble-nav2-controller \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-nav2-map-server \
        ros-humble-nav2-mppi-controller \
        ros-humble-nav2-msgs \
        ros-humble-rclpy \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-sensor-msgs \
        ros-humble-slam-toolbox \
        ros-humble-teleop-twist-keyboard \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
        ros-humble-visualization-msgs && \
    rm -rf /var/lib/apt/lists/*

# SDK と humble ドライバをコミット固定で取得する。
# ros:humble に含まれるビルドツールを明示。既存ROS依存のキャッシュと分離する。
RUN apt-get install -y --no-install-recommends git cmake build-essential
ARG YDLIDAR_SDK_COMMIT=01cdda4f2b36dff2a706d0535c64228d863c7411
ARG YDLIDAR_DRIVER_COMMIT=4ef70d3f32a85704ade0be54b214f3763b1ab3e8
RUN git init /opt/YDLidar-SDK && \
    git -C /opt/YDLidar-SDK remote add origin https://github.com/YDLIDAR/YDLidar-SDK.git && \
    git -C /opt/YDLidar-SDK fetch --depth 1 origin ${YDLIDAR_SDK_COMMIT} && \
    git -C /opt/YDLidar-SDK checkout --detach FETCH_HEAD && \
    cmake -S /opt/YDLidar-SDK -B /opt/YDLidar-SDK/build && \
    cmake --build /opt/YDLidar-SDK/build -j2 && \
    cmake --install /opt/YDLidar-SDK/build && ldconfig

# minicar_motor の backend:=fabo_pca9685 用。FaBo の notebooks/98_setting.ipynb と
# 同じライブラリ。既存の ROS パッケージのレイヤーをキャッシュのまま残すため別の RUN にする。
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        i2c-tools \
        python3-pip \
        python3-smbus && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install --no-cache-dir \
        git+https://github.com/FaBoPlatform/FaBoPWM-PCA9685-Python@jupyterlab

# D455 ドライバ。別コンテナの Isaac ROS 3.2 cuVSLAM が要求する組み合わせ
# (librealsense 2.55.1 + realsense-ros 4.51.1-isaac、FW 5.13.0.50)に固定する。
# Isaac ROS の Dockerfile.realsense と同じく RSUSB backend でビルドし、
# カーネルパッチや udev なしで /dev/bus/usb 経由で D455 を扱う。
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libeigen3-dev \
        libssl-dev \
        libudev-dev \
        libusb-1.0-0-dev \
        pkg-config \
        ros-humble-cv-bridge \
        ros-humble-diagnostic-updater \
        ros-humble-image-transport \
        ros-humble-rosbag2-storage-mcap && \
    rm -rf /var/lib/apt/lists/*
ARG LIBREALSENSE_VERSION=v2.55.1
ARG REALSENSE_ROS_COMMIT=c04f43308c00a6f495f14a47cc84cc293047cdff
ARG REALSENSE_BUILD_JOBS=4
RUN git clone --depth 1 --branch ${LIBREALSENSE_VERSION} \
        https://github.com/IntelRealSense/librealsense.git /opt/librealsense && \
    cmake -S /opt/librealsense -B /opt/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_WITH_CUDA=OFF \
        -DBUILD_EXAMPLES=ON \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_GLSL_EXTENSIONS=OFF \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DBUILD_UNIT_TESTS=OFF \
        -DCHECK_FOR_UPDATES=OFF && \
    cmake --build /opt/librealsense/build -j${REALSENSE_BUILD_JOBS} && \
    cmake --install /opt/librealsense/build && ldconfig && \
    rm -rf /opt/librealsense/build

# docker compose exec の対話シェルでも ROS 2 コマンドをそのまま使えるようにする。
# 非対話 bash は compose の BASH_ENV で同じ setup.bash を読む。
WORKDIR /ws
RUN git init src/ydlidar_ros2_driver && \
    git -C src/ydlidar_ros2_driver remote add origin https://github.com/YDLIDAR/ydlidar_ros2_driver.git && \
    git -C src/ydlidar_ros2_driver fetch --depth 1 origin ${YDLIDAR_DRIVER_COMMIT} && \
    git -C src/ydlidar_ros2_driver checkout --detach FETCH_HEAD
# realsense2_description は xacro 依存で、ドライバ起動には不要なので除外する。
RUN git init src/realsense-ros && \
    git -C src/realsense-ros remote add origin https://github.com/NVIDIA-ISAAC-ROS/realsense-ros.git && \
    git -C src/realsense-ros fetch --depth 1 origin ${REALSENSE_ROS_COMMIT} && \
    git -C src/realsense-ros checkout --detach FETCH_HEAD && \
    rm -rf src/realsense-ros/realsense2_description
COPY src /ws/src
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY docker/ros_setup.bash /etc/minicar/ros_setup.bash
COPY docker/entrypoint.bash /etc/minicar/entrypoint.bash
# ホスト→コンテナのSHM切り分け用(README「ホストのDDS設定を確認」)。
COPY docker/fastdds_udp_only.xml /etc/minicar/fastdds_udp_only.xml
RUN echo 'source /etc/minicar/ros_setup.bash' >> /root/.bashrc
ENV BASH_ENV=/etc/minicar/ros_setup.bash

ENV DEBIAN_FRONTEND=
ENTRYPOINT ["/bin/bash", "/etc/minicar/entrypoint.bash"]
CMD ["bash"]
