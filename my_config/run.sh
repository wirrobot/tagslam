#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# TagSLAM 一键运行脚本
# 用法: bash run.sh [visualize]
#   visualize — 额外启动可视化调试窗口
# ═══════════════════════════════════════════════════════════════
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_DIR="$SCRIPT_DIR"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# 自动查找 ROS2 环境
# Always source base ROS2 first, then the workspace overlay on top
source /opt/ros/humble/setup.bash 2>/dev/null \
  || source /opt/ros/jazzy/setup.bash 2>/dev/null \
  || source /opt/ros/rolling/setup.bash 2>/dev/null

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

echo "═══════════════════════════════════════"
echo "  TagSLAM一键启动"
echo "  config dir: $CONFIG_DIR"
echo "═══════════════════════════════════════"

cleanup() {
    echo ""
    echo "Shutting down..."
    kill %1 %2 2>/dev/null
    wait 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── 启动 sync_and_detect（Tag 检测）──
echo "[1/2] Starting sync_and_detect..."
ros2 launch tagslam sync_and_detect.launch.py \
    cameras:="$CONFIG_DIR/cameras.yaml" \
    tagslam_config:="$CONFIG_DIR/tagslam.yaml" \
    use_approximate_sync:=True &

sleep 2

# ── 启动 tagslam（SLAM 优化）──
echo "[2/2] Starting tagslam..."
ros2 launch tagslam tagslam.launch.py \
    cameras:="$CONFIG_DIR/cameras.yaml" \
    camera_poses:="$CONFIG_DIR/camera_poses.yaml" \
    tagslam_config:="$CONFIG_DIR/tagslam.yaml" \
    use_approximate_sync:=True &

sleep 2

# ── 可选：启动可视化 ──
if [ "${1:-}" = "visualize" ]; then
    echo "[3/3] Starting visualizer..."
    python3 "$CONFIG_DIR/visualizer.py" &
fi

echo ""
echo "All nodes running. Press Ctrl+C to stop."
echo "  Topics:"
echo "    /detector/tags          — 检测到的 Tag"
echo "    /tagslam/odom/body_rig  — 摄像头位姿"
echo ""

wait
