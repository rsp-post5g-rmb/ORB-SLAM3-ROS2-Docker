#!/bin/bash
# Build ORB_SLAM3 and the colcon workspace.
#
# Run automatically by the Dev Container `postCreateCommand`, but safe to run by
# hand at any time: `colcon build --symlink-install` is incremental, so re-running
# only rebuilds what changed.
#
# Usage:
#   build_workspace.sh        # CPU build
#   build_workspace.sh cuda   # NVIDIA / FastTrack CUDA build
set -euo pipefail

ORB_SLAM3_DIR=/home/orb/ORB_SLAM3
COLCON_WS=/root/colcon_ws

if [ ! -f "${ORB_SLAM3_DIR}/build.sh" ]; then
    echo "ERROR: ${ORB_SLAM3_DIR}/build.sh not found." >&2
    echo "The ORB_SLAM3 (or FastTrack) submodule is empty. On the HOST run:" >&2
    echo "    git submodule update --init --recursive --remote" >&2
    echo "then rebuild the Dev Container (Dev Containers: Rebuild Container)." >&2
    exit 1
fi

CUDA_CMAKE_ARGS=()
if [ "${1:-}" = "cuda" ]; then
    echo ">>> CUDA build enabled"
    CUDA_CMAKE_ARGS=(--cmake-args -DORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA=ON)
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

echo ">>> Building ORB_SLAM3 (Thirdparty + library) ..."
cd "${ORB_SLAM3_DIR}"
chmod +x build.sh
./build.sh

echo ">>> Building colcon workspace ..."
cd "${COLCON_WS}"
colcon build --symlink-install "${CUDA_CMAKE_ARGS[@]}"

echo ">>> Build complete. Open a new terminal (or 'sws') to use the workspace."
