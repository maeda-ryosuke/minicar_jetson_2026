#!/bin/bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
common_dir="${project_dir}/.isaac_ros_common"
common_ref="v3.2-15"
image_name="minicar_vslam:3.2"

if [[ "$(uname -m)" != "aarch64" ]]; then
    echo "Build this Jetson image on the target aarch64 Jetson, not $(uname -m)." >&2
    exit 1
fi
if [[ ! -r /etc/nv_tegra_release ]] || \
        ! head -n 1 /etc/nv_tegra_release | grep -q 'R36 (release), REVISION: 4.3'; then
    echo "Jetson Linux R36.4.3 (JetPack 6.2) is required." >&2
    exit 1
fi

if [[ ! -d "${common_dir}/.git" ]]; then
    git clone --branch "${common_ref}" --depth 1 \
        https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git "${common_dir}"
fi

actual_ref="$(git -C "${common_dir}" describe --tags --exact-match 2>/dev/null || true)"
if [[ "${actual_ref}" != "${common_ref}" ]]; then
    echo "${common_dir} must be checked out at ${common_ref}; found ${actual_ref:-unknown}" >&2
    exit 1
fi

"${common_dir}/scripts/build_image_layers.sh" \
    --image_key ros2_humble.realsense.minicar_vslam \
    --image_name "${image_name}" \
    --context_dir "${project_dir}"

echo "Built ${image_name}"
