#!/bin/bash
set -euo pipefail

fail() {
    echo "VSLAM preflight failed: $*" >&2
    exit 1
}

[[ "$(uname -m)" == "aarch64" ]] || fail "aarch64 Jetson is required"

if [[ -r /etc/nv_tegra_release ]]; then
    l4t_release="$(head -n 1 /etc/nv_tegra_release)"
    [[ "${l4t_release}" == *"R36 (release), REVISION: 4.3"* ]] || \
        fail "Jetson Linux R36.4.3 is required; found: ${l4t_release}"
else
    fail "/etc/nv_tegra_release is unavailable; NVIDIA Container Runtime is required"
fi

memory_kib="$(awk '/^MemTotal:/ {print $2}' /proc/meminfo)"
(( memory_kib >= 6000000 )) || fail "at least 6 GB RAM is required; Orin Nano 4 GB is unsupported"

vslam_version="$(dpkg-query -W -f='${Version}' ros-humble-isaac-ros-visual-slam 2>/dev/null)" || \
    fail "ros-humble-isaac-ros-visual-slam is not installed"
[[ "${vslam_version}" == 3.2.* ]] || fail "Isaac ROS Visual SLAM 3.2.x is required; found ${vslam_version}"

realsense_manifest="$(ros2 pkg xml realsense2_camera 2>/dev/null)" || \
    fail "realsense2_camera is not installed"
grep -q '<version>4\.51\.1</version>' <<<"${realsense_manifest}" || \
    fail "realsense2_camera 4.51.1 is required"

librealsense_version="$(pkg-config --modversion realsense2 2>/dev/null)" || \
    fail "librealsense2 development metadata is unavailable"
[[ "${librealsense_version}" == "2.55.1" ]] || \
    fail "librealsense 2.55.1 is required; found ${librealsense_version}"

device_summary="$(rs-enumerate-devices -s 2>&1)" || fail "cannot enumerate RealSense devices"
grep -q 'Intel RealSense D455' <<<"${device_summary}" || fail "Intel RealSense D455 was not found"
grep -Eiq 'USB[^:]*:[[:space:]]*3\.[0-9]' <<<"${device_summary}" || \
    fail "D455 is not connected through USB 3.x"

echo "VSLAM preflight passed: JetPack 6.2, D455 USB 3.x, Isaac ROS ${vslam_version}"
