#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# entrypoint.bridge.sh — Container entrypoint for NUEVO Bridge (ROS2 mode)
#
# Runs colcon build for both ROS2 packages on every startup.
# Build artifacts are cached in named Docker volumes (build/ and install/),
# so only the first startup is slow (~60s for nuevo_msgs CMake codegen).
# Subsequent restarts reuse the cache and are fast (~5s).
# ─────────────────────────────────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash

echo "[entrypoint] Building ROS2 packages (cached after first run)..."
colcon build \
    --packages-select nuevo_msgs nuevo_bridge \
    --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF

source /ros2_ws/install/setup.bash

echo "[entrypoint] ROS_DISTRO=${ROS_DISTRO}"
echo "[entrypoint] Launching: $*"
exec "$@"
