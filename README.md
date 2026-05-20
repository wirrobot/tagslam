# TagSLAM: SLAM with Tags

TagSLAM is a ROS2-based package for Simultaneous Localization and
Mapping using Apriltag fiducial markers.

## Platforms supported

ROS2 Rolling / Jazzy or newer.

## Quick Start (3-AprilTag Example)

This repository includes a ready-to-use configuration for a 3-AprilTag
setup under `my_config/`.

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

### 1. Install Dependencies

```bash
sudo apt install -y \
  ros-$(rosversion -d)-gtsam \
  ros-$(rosversion -d)-apriltag-detector \
  ros-$(rosversion -d)-apriltag-msgs \
  ros-$(rosversion -d)-cv-bridge \
  ros-$(rosversion -d)-image-transport \
  ros-$(rosversion -d)-tf2 \
  ros-$(rosversion -d)-tf2-msgs \
  ros-$(rosversion -d)-rosbag2 \
  libopencv-dev libboost-graph-dev libyaml-cpp-dev python3-opencv
```

### 2. Build

```bash
mkdir -p ~/tagslam_ws/src && cd ~/tagslam_ws/src
git clone https://github.com/berndpfrommer/tagslam.git -b ros2
git clone https://github.com/berndpfrommer/flex_sync.git -b master
cd ~/tagslam_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 3. Edit Config

Before running, update `my_config/cameras.yaml` with your actual camera
parameters (intrinsics, distortion, resolution, image_topic).

### 4. Run

```bash
# SLAM only
bash my_config/run.sh

# SLAM + real-time visualizer overlay (shows xyz on camera feed)
bash my_config/run.sh visualize
```

The visualizer window shows the camera image with the current camera pose
(X/Y/Z coordinates) overlaid in the top-left corner. Press `Q` or `Esc` to close.

### Output Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/detector/tags` | `apriltag_msgs/...` | Detected tag corners |
| `/tagslam/odom/body_rig` | `nav_msgs/Odometry` | Camera rig pose |
| `/tf` | `tf2_msgs/TFMessage` | All transforms |

### 5. Dump Results

```bash
ros2 service call /tagslam/dump std_srvs/srv/Trigger
```

Writes `camera_poses.yaml`, `poses.yaml`, `error_map.txt` to the current directory.


## How to use


### Sync and detect

TagSLAM operates off of tags (and odometry messages, if provided). In a scenerio with multiple cameras or odometry it is important
that all sensors are synchronized, meaning that the sensor data has matching ROS header.stamp fields.
Sync_and_detect runs the apriltag detector across multiple cameras and emits synchronized messages with the decoded tags.
These messages in turn are used by TagSLAM. Note that ``sync_and_detect`` can also deal with odometry:
it drops all odometry messages except for the ones that coincide (approximately) with the camera images, and alters the header.stamp
field to match exactly the ones of the image messages.

You can run ``sync_and_detect`` either from a bag file, and write the
detected tags into another bag, or you can run it as a stand-alone (composable) node.
It will use the ``cameras.yaml`` file to determine what topics to read from the input bag, what image transport (raw vs compressed), what tag detector
(MIT vs UMich), and what output tag topics to use. The ``tagslam.yaml`` file is searched for bodies with odometry topics.


Here is how to run it from a bag:
```
ros2 run tagslam sync_and_detect_from_bag --ros-args -p "cameras:=./cameras.yaml" -p "tagslam_config:=./tagslam.yaml" -p "in_bag:=name_of_input_bag" -p "out_bag:=./tag_bag"
```

For online operation, launch a ``sync_and_detect`` node like this:
```
ros2 launch tagslam sync_and_detect.launch.py use_sim_time:=<True/False> cameras:=<path_to_cameras.yaml_file> tagslam_config:=<path_to_tagslam_config_file> use_approximate_sync:=<True/False>
```

### TagSLAM

TagSLAM can run off a rosbag, or as a node. When running off a bag, TagSLAM will automatically recognize when there are only
image topics, but no tag topics in the rosbag, and will start ``sync_and_detect`` to do tag detection.

Run TagSLAM from a rosbag like this:
```
ros2 run tagslam tagslam_from_bag --ros-args -p "cameras:=./cameras.yaml" -p "tagslam_config:=./tagslam.yaml" -p "camera_poses:=./camera_poses.yaml" -p "in_bag:=./bag_with_tags_and_odom" -p "out_bag:=./out_bag"
```

For online operation, launch a ``tagslam`` node like this:
```
ros2 launch tagslam tagslam.launch.py use_sim_time:=<True/False> cameras:=<path_to_cameras.yaml_file> camera_poses:=<path_to_camera_poses.yaml file> tagslam_config:=<path_to_tagslam_config_file> use_approximate_sync:=<True/False>
```

### Rosbag

When playing from a ros2 bag it's important to pass ``use_sim_time:=True`` to all launch scripts, and to let the ros2 bag player drive the clock:
```
ros2 bag play --clock-topics-all my_bag/
```

## Trouble Shooting

### Nothing happens

- Check that the topics match. ``ros2 node info`` is your friend.
- If you have multiple cameras running, check that they are synchronized, i.e. that the ``header.stamp`` time stamps match between cameras. If they don't match, use an pproximate synchronizer.
- Is ``use_sim_time`` set consistently across all nodes?

### Jerky motions
- Check for image quality and that the tag detector works as it should. Use the apriltag_detector and in particular ``apriltag_draw`` from [this repo](https://github.com/ros-misc-utilities/apriltag_detector), which is available as installable apt package under ROS2.

### Large reprojection errors and SUBGRAPH ERROR warnings

- poor image quality
- bad calibration file
- bad camera pose file
- wrong tag size
- wrong tag pose specified


## License

This software and any future contributions to it are licensed under
the [Apache License 2.0](LICENSE).
