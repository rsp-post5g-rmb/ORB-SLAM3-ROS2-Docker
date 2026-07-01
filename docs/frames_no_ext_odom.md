# Frame Conventions (no external odometry)

Scope: the default setup where SLAM is your **only** pose source —
`odometry_mode: false`. In this mode ORB-SLAM3 publishes `map → base_footprint`
directly; there is no `odom` frame to worry about.

Everything the node outputs is already in standard ROS (REP-103) axes — the raw
ORB camera pose is converted before publishing.

---

## TL;DR

- **Axes:** ORB-SLAM3 tracks in the **camera optical** frame (X-right, Y-down,
  Z-forward); the wrapper rotates it to **ROS/REP-103** (X-forward, Y-left,
  Z-up) before publishing.
- **Everything published is in the `map` frame**, REP-103 axes.
- **`robot_*` params** are NOT a spawn pose — they are the fixed
  **`base_footprint → camera_link`** camera mount offset.
- A fresh launch (or the `reset_mapping_pose` service) puts `base_footprint`
  exactly at the `map` origin.

---

## Frame tree

```
map ──(published by SLAM)──▶ base_footprint ──(your URDF / static TF)──▶ camera_link
      map → base_footprint          base_footprint → camera_link
      = SLAM estimate               = the robot_* mount extrinsic
```

The wrapper publishes **only** `map → base_footprint`. The
`base_footprint → camera_link` link comes from your robot description; the
`robot_*` params tell SLAM what that offset is.

| Frame | Param | Default | Meaning |
|-------|-------|---------|---------|
| global / world | `global_frame` | `map` | Fixed world frame; header of every published pose/TF. Origin is pinned where SLAM initialized. |
| robot base | `robot_base_frame` | `base_footprint` | Robot body frame (REP-103). |
| camera | — | `camera_link` | RealSense body frame, offset from the base by the mount extrinsic below. |

Params documented in `README.md:177-185`.

---

## Axis convention (ORB → ROS)

ORB-SLAM3 tracks the camera in the **optical** frame; ROS uses REP-103:

| Axis | ORB-SLAM3 (optical) | ROS (REP-103) |
|------|---------------------|---------------|
| X | right | forward |
| Y | down | left |
| Z | forward | up |

The fixed rotation applied to every pose (`type_conversion.cpp:57-59`,
in `se3ORBToROS` at `:51`):

```
              | 0  0  1 |      ros.x =  orb.z   (forward)
R_orb→ros  =  |-1  0  0 |  →   ros.y = -orb.x   (left)
              | 0 -1  0 |      ros.z = -orb.y   (up)
```

So consumers of `/robot_pose_slam` and the TF get standard ROS axes — do **not**
re-apply an optical→body rotation yourself.

---

## Camera mount extrinsic (`robot_*` params)

The `robot_x/y/z` + `robot_qx/qy/qz/qw` params define the static transform
**`base_footprint → camera_link`** — where the camera physically sits on the
robot. Getting the rotation wrong (e.g. a tilted camera) is the most common
cause of an offset/rotated pose.

```yaml
# base_footprint -> camera_link
robot_x: 0.30
robot_y: 0.0
robot_z: 0.50
robot_qx: 0.0
robot_qy: 0.0
robot_qz: 0.0
robot_qw: 1.0
```

How it's used: the mount is built into `robotBase_to_cameraLink_` from these
params (`orb_slam3_interface.cpp:35-37`), then applied by conjugation to turn
the tracked **camera** pose into the published **base_footprint** pose
(`orb_slam3_interface.cpp:354`):

```
T(map→base) = M · T(map→camera) · M⁻¹        where  M = base→camera
```

Because it's a conjugation, when the camera pose is identity the base pose is
also identity — that's why a fresh launch starts the robot exactly at the `map`
origin. (Comment in `params/ros_params/gazebo-rgbd-ros-params.yaml:15-17`.)

---

## What gets published

| Interface | Type | Frame | Source |
|-----------|------|-------|--------|
| `/robot_pose_slam` | `geometry_msgs/PoseStamped` | header = `map`; pose of `base_footprint` | `getRobotPose` (`orb_slam3_interface.cpp:438`) |
| TF `map → base_footprint` | TF | published directly | `getDirectMapToRobotTF` (`orb_slam3_interface.cpp:386`), selected by the `!odometry_mode_` branch at `slam_node_base.cpp:144-150` |
| `/map_data`, `/slam_info` | `slam_msgs/*` | header = `map` | keyframe / landmark data |

---

## Gotchas

- **Poses are already REP-103** — don't re-rotate downstream; the wrapper did it.
- **`map` can jump on loop closure** — expected; the global frame is corrected
  when drift is closed. Buffer accordingly if you consume TF.
- **`robot_*` is an extrinsic, not a start pose** — it moves the camera relative
  to the base, not where the robot spawns.
- **Origin resets** — a fresh launch, or `reset_mapping_pose`, pins a new `map`
  origin at the current `base_footprint`. Plain `reset_mapping` keeps the
  previous pose. See [quickstart.md](quickstart.md).

---

## Source map

| What | File |
|------|------|
| Optical→ROS matrix | `orb_slam3_ros2_wrapper/src/type_conversion.cpp:51-78` |
| Conversion API + contracts | `orb_slam3_ros2_wrapper/include/orb_slam3_ros2_wrapper/type_conversion.hpp:61-93` |
| Mount extrinsic built | `orb_slam3_ros2_wrapper/src/orb_slam3_interface.cpp:35-37` |
| Pose conversion + mount applied | `orb_slam3_ros2_wrapper/src/orb_slam3_interface.cpp:348-354` |
| Pose + `map→base_footprint` TF | `orb_slam3_ros2_wrapper/src/orb_slam3_interface.cpp:386-448` |
| Frame params | `README.md:177-185`, `orb_slam3_ros2_wrapper/params/ros_params/*.yaml` |
