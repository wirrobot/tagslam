# TagSLAM: SLAM with Tags

TagSLAM is a ROS2-based package for Simultaneous Localization and
Mapping using Apriltag fiducial markers.

## Platforms supported

ROS2 Humble / Jazzy / Rolling.

> **Note for Humble**: the `ros2` branch has been patched to work with
> Humble (bag-level compatibility fixes). If building against a newer
> ROS2 distro, the original upstream code can be used without patches.

## Quick Start (3-AprilTag Example)

This repository includes a ready-to-use configuration for a **3-AprilTag**
setup under `config/`.

### Tag Layout

```
Tag 1 (ID=1, 2cm)   Tag 0 (ID=0, 15cm)   Tag 2 (ID=2, 2cm)
     ← left              center                right →
   y = -0.4 m           y = 0.0             y = +0.4 m
```

**Coordinate system** (origin at Tag 0 center):
- **x**: perpendicular to wall, pointing toward camera
- **y**: horizontal along wall, right is positive
- **z**: vertical, up

---

### 1. Install Dependencies

```bash
sudo apt install -y \
  ros-$(rosversion -d)-gtsam \
  ros-$(rosversion -d)-apriltag-msgs \
  ros-$(rosversion -d)-apriltag-detector \
  ros-$(rosversion -d)-apriltag-detector-umich \
  ros-$(rosversion -d)-cv-bridge \
  ros-$(rosversion -d)-image-transport \
  ros-$(rosversion -d)-tf2 ros-$(rosversion -d)-tf2-msgs \
  ros-$(rosversion -d)-nav-msgs \
  ros-$(rosversion -d)-rosbag2 \
  libopencv-dev libboost-graph-dev libyaml-cpp-dev
```

Also fix a NumPy version conflict (ROS2 Humble requires NumPy 1.x):

```bash
pip3 install "numpy<2" "opencv-python<4.10"
```

---

### 2. Build

```bash
git clone https://github.com/waliwuao/tagslam.git
cd tagslam

# Clone flex_sync dependency alongside tagslam
git clone https://github.com/berndpfrommer/flex_sync.git -b master flex_sync

# Build both packages (disable tests to avoid optional linter deps)
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --base-paths . flex_sync

# Source the overlay
source install/setup.bash
```

---

### 3. Configure Your Camera

Edit `config/cameras.yaml` — update these fields to match your hardware:

```yaml
cam0:
  intrinsics: [fx, fy, cx, cy]           # from camera calibration
  distortion_coeffs: [k1, k2, p1, p2, k3]  # or 4 coeffs for equidistant
  resolution: [width, height]
  image_topic: camera/image_raw            # your camera's ROS topic
```

If you don't have a ROS2 camera driver (e.g. using a phone camera via
Iriun/DroidCam), use the built-in CLI tool:

```bash
cd tools && uv run tagslam-tools publish
```
Or for interactive capture and calibration:
```bash
cd tools && uv run tagslam-tools capture   # SPACE to save frames
cd tools && uv run tagslam-tools calibrate # calibrate from saved images
```

---

### 4. One-Click Run

After building and configuring, source ROS2 and launch:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

# Run SLAM only
cd tools && uv run tagslam-tools launch

# Run SLAM + live visualizer overlay (shows camera XYZ on image)
cd tools && uv run tagslam-tools launch --viz
```

Or use the interactive menu:

```bash
cd tools && uv run tagslam-tools menu
```

The `launch` command automatically:
1. Sources ROS2 environment (ensure it's sourced before running)
2. Starts camera publisher
3. Starts `sync_and_detect` (tag detection)
4. Starts `tagslam` (SLAM optimization)
5. Optionally starts visualizer (with `--viz`)

Press `Ctrl+C` to stop all nodes.

---

### 5. Output Topics & Visualization

| Topic | Type | Description |
|-------|------|-------------|
| `camera/image_raw` | `sensor_msgs/Image` | Camera feed (published by camera_pub.py) |
| `/detector/tags` | `apriltag_msgs/AprilTagDetectionArray` | Detected tag corners |
| `/odom/body_rig` | `nav_msgs/Odometry` | Camera rig pose (appears after first tag is seen) |
| `/tf` | `tf2_msgs/TFMessage` | All transforms (world→rig→cam, rig→tag) |

The visualizer (`config/visualizer.py`) overlays the camera's current
X/Y/Z position on the live camera feed (top-left panel).

---

### 6. Dump Final Results

```bash
ros2 service call /tagslam/dump std_srvs/srv/Trigger
```

Writes `camera_poses.yaml`, `poses.yaml`, `error_map.txt`, `tag_corners.txt`
to the current directory.

---

### Files in `config/`

| File | Purpose |
|------|---------|
| `cameras.yaml` | Camera intrinsics, distortion, topic names |
| `camera_poses.yaml` | Camera-to-rig extrinsic prior (optional) |
| `tagslam.yaml` | Tag layout, body definitions, SLAM parameters |

### Python Tools (`tools/`)

| Command | Purpose |
|--------|---------|
| `uv run tagslam-tools menu` | Interactive menu for all operations |
| `uv run tagslam-tools publish` | Publish camera frames to ROS2 |
| `uv run tagslam-tools capture` | Interactive camera preview + capture |
| `uv run tagslam-tools calibrate` | Calibrate camera from chessboard images |
| `uv run tagslam-tools visualize` | Live SLAM pose overlay on camera feed |
| `uv run tagslam-tools launch` | One-click launch full pipeline |

---

## How to use (manual / advanced)

For users who need fine-grained control, the two ROS2 nodes can also
be launched independently.

### Sync and detect (tag detection only)

```bash
ros2 launch tagslam sync_and_detect.launch.py \
  cameras:=./cameras.yaml \
  tagslam_config:=./tagslam.yaml
```

### TagSLAM (optimization only)

```bash
ros2 launch tagslam tagslam.launch.py \
  cameras:=./cameras.yaml \
  camera_poses:=./camera_poses.yaml \
  tagslam_config:=./tagslam.yaml
```

### Running from a rosbag

```bash
# Detect tags from bag
ros2 run tagslam sync_and_detect_from_bag --ros-args \
  -p cameras:=./cameras.yaml \
  -p tagslam_config:=./tagslam.yaml \
  -p in_bag:=./input_bag \
  -p out_bag:=./tag_bag

# Run SLAM from tag bag
ros2 run tagslam tagslam_from_bag --ros-args \
  -p cameras:=./cameras.yaml \
  -p tagslam_config:=./tagslam.yaml \
  -p camera_poses:=./camera_poses.yaml \
  -p in_bag:=./tag_bag
```

Always pass `use_sim_time:=True` when playing bags:

```bash
ros2 bag play --clock-topics-all my_bag/
```

---

## Trouble Shooting

### Nothing happens / no tags detected

- Verify the camera is actually pointed at the AprilTags.
- Check topic flow: `ros2 topic hz /detector/tags`
- Ensure `use_sim_time` is consistent across all nodes (False for live, True for bags).

### No pose published on `/tagslam/odom/body_rig`

- Tagslam only starts publishing after the **first tag** is observed.
  Point the camera at a tag and wait 1–2 seconds.

### Jerky motions

- Check image quality. Use `apriltag_draw` from the
  [apriltag_detector](https://github.com/ros-misc-utilities/apriltag_detector) repo.

### Large reprojection errors / SUBGRAPH ERROR warnings

- Poor image quality
- Wrong camera calibration (intrinsics / distortion)
- Wrong tag size in `tagslam.yaml`
- Wrong tag pose specified


## License

This software and any future contributions to it are licensed under
the [Apache License 2.0](LICENSE).
