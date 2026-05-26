#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# TagSLAM 一键运行脚本
# 用法:
#   bash run.sh              — 仅启动 SLAM
#   bash run.sh viz          — SLAM + 可视化窗口
#   uv run tagslam-tools menu — 交互式菜单
# ═══════════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOLS_DIR="$SCRIPT_DIR/tools"

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null \
  || source /opt/ros/jazzy/setup.bash 2>/dev/null \
  || source /opt/ros/rolling/setup.bash 2>/dev/null

# Source workspace overlay
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

echo "═══════════════════════════════════════"
echo "  TagSLAM 一键启动"
echo "═══════════════════════════════════════"

cleanup() {
    echo ""
    echo "Shutting down..."
    jobs -p | xargs -r kill 2>/dev/null
    wait 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

CONFIG="$SCRIPT_DIR/config"

# Start camera publisher using CLI
echo "[1/4] Starting camera publisher..."
cd "$TOOLS_DIR" && uv run python -c "from tagslam_tools.camera import publish_camera_loop; publish_camera_loop()" &
sleep 1

# Start sync_and_detect
echo "[2/4] Starting sync_and_detect..."
ros2 launch tagslam sync_and_detect.launch.py \
    cameras:="$CONFIG/cameras.yaml" \
    tagslam_config:="$CONFIG/tagslam.yaml" \
    use_approximate_sync:=True &
sleep 2

# Start tagslam
echo "[3/4] Starting tagslam..."
ros2 launch tagslam tagslam.launch.py \
    cameras:="$CONFIG/cameras.yaml" \
    camera_poses:="$CONFIG/camera_poses.yaml" \
    tagslam_config:="$CONFIG/tagslam.yaml" \
    use_approximate_sync:=True &
sleep 2

# Optional visualizer
if [ "${1:-}" = "viz" ] || [ "${1:-}" = "visualize" ]; then
    echo "[4/4] Starting visualizer..."
    cd "$TOOLS_DIR" && uv run python -c "from tagslam_tools.visualizer import run_visualizer; run_visualizer()" &
fi

echo ""
echo "All nodes running. Press Ctrl+C to stop."
echo "  Topics:"
echo "    camera/image_raw      — 摄像头画面"
echo "    /detector/tags        — 检测到的 Tag"
echo "    /odom/body_rig        — 摄像头位姿"
echo ""

# Show interactive menu option
echo "Tip: run 'cd tools && uv run tagslam-tools menu' for interactive mode"
echo ""

wait
