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
uv run run.py camera publish
```
Or for interactive capture and calibration:
```bash
uv run run.py camera capture   # SPACE to save frames
uv run run.py calibrate calibrate # calibrate from saved images
```

---

### 4. One-Click Run

After building and configuring, source ROS2 and launch:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

# Interactive menu
uv run run.py --menu

# Run SLAM only
uv run run.py launch

# Run SLAM + live visualizer overlay
uv run run.py launch --viz

# Direct subcommands
uv run run.py camera capture     # capture calibration images
uv run run.py calibrate calibrate # calibrate from images
```

Or use the interactive menu:

```bash
uv run run.py --menu
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

### 7. Tag Detection Test

A stand-alone test script validates AprilTag detection and 6-DOF
pose estimation using `pupil_apriltags` (no ROS2 required).
The script auto-checks dependencies on launch.

#### Run

```bash
# Place your test image (must be named test.jpg)
cp your_tag_image.jpg tools/tests/test.jpg

# One-click run — shows pixel coordinates AND pose
./run_test_tag.sh -sv
```

#### What is tested

The script runs **5 tests** using the camera intrinsics and tag
sizes from `config/cameras.yaml` and `config/tagslam.yaml`:

1. **Image load** — verifies `test.jpg` is a valid grayscale image.
2. **Tag detection** — prints each tag's ID, family, hamming distance, and **4 corner pixel coordinates**.
3. **Corners validation** — corners must lie within image bounds and form a convex quad.
4. **Pose estimation** — computes 6-DOF pose (3×3 rotation matrix, 3×1 translation vector) using the tag size configured for the detected ID.
5. **Pose consistency** — verifies that the distance ratio matches the tag-size ratio when a single tag is estimated with two different sizes.

#### Output example

```
────────────────────────────────────────────────────────
  Tag ID=0  (with pose)
────────────────────────────────────────────────────────
  Family:          tag36h11
  Hamming:         0
  Decision margin: 120.00
  Corners (px):
    [  539.79,   460.21]
    [  739.88,   459.87]
    [  739.87,   259.87]
    [  540.13,   259.87]
  Pose  (tag_size=0.1283 m):
    R = [[ 0.9999  0.0001 -0.0137]
         [ 0.0001  0.9999  0.0135]
         [ 0.0137 -0.0135  0.9998]]
    t = [-0.0055  0.0027  1.0606]  m
    Distance: 1.0606 m
    Error:    0.000000
```

#### Advanced options

```bash
./run_test_tag.sh -k pose -sv     # run only pose-related tests
./run_test_tag.sh -h              # show all pytest passthrough options
```

---

### Files in `config/`

| File | Purpose |
|------|---------|
| `cameras.yaml` | Camera intrinsics, distortion, topic names |
| `camera_poses.yaml` | Camera-to-rig extrinsic prior (optional) |
| `tagslam.yaml` | Tag layout, body definitions, SLAM parameters |

### Python Tools (`uv run run.py`)

| Command | Purpose |
|--------|---------|
| `uv run run.py --menu` | Interactive menu for all operations |
| `uv run run.py camera publish` | Publish camera frames to ROS2 |
| `uv run run.py camera capture` | Interactive camera preview + capture |
| `uv run run.py calibrate calibrate` | Calibrate camera from chessboard images |
| `uv run run.py visualize visualize` | Live SLAM pose overlay on camera feed |
| `uv run run.py launch --viz` | One-click launch full pipeline |

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
