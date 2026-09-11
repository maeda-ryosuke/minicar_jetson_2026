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
        ros-humble-nav2-map-server \
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

# docker compose exec の対話シェルでも ROS 2 コマンドをそのまま使えるようにする。
# 非対話 bash は compose の BASH_ENV で同じ setup.bash を読む。
WORKDIR /ws
RUN git init src/ydlidar_ros2_driver && \
    git -C src/ydlidar_ros2_driver remote add origin https://github.com/YDLIDAR/ydlidar_ros2_driver.git && \
    git -C src/ydlidar_ros2_driver fetch --depth 1 origin ${YDLIDAR_DRIVER_COMMIT} && \
    git -C src/ydlidar_ros2_driver checkout --detach FETCH_HEAD
COPY src /ws/src
RUN source /opt/ros/humble/setup.bash && colcon build
COPY docker/ros_setup.bash /etc/minicar/ros_setup.bash
COPY docker/entrypoint.bash /etc/minicar/entrypoint.bash
RUN echo 'source /etc/minicar/ros_setup.bash' >> /root/.bashrc
ENV BASH_ENV=/etc/minicar/ros_setup.bash

ENV DEBIAN_FRONTEND=
ENTRYPOINT ["/bin/bash", "/etc/minicar/entrypoint.bash"]
CMD ["bash"]
