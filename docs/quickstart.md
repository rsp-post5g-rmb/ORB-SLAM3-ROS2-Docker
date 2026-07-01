# ORB-SLAM3 RGBD Live Pose Logging with RealSense D435

Reference setup for running **live full SLAM** (map built on the fly, no prebuilt
atlas) on a mobile manipulator with a RealSense **D435** (no IMU), using the
`suchetanrs/ORB-SLAM3-ROS2-Docker` wrapper. Goal: spawn point becomes the world
origin, and the camera pose is logged on `/robot_pose_slam` as the robot moves.

- **Container:** ORB-SLAM3 ROS 2 Humble docker (suchetanrs)
- **Camera:** Intel RealSense D435, 640x480 @ 30 fps, depth aligned to color
- **Mode:** `rgbd` (plain RGB-D, no IMU)
- **ROS_DOMAIN_ID:** 55 (set by the container's `ros_env_vars.sh`)

---

## 0. Build and bring up the container

RealSense (librealsense2 SDK + `ros-humble-realsense2-camera`) and all build
dependencies are baked into the image, so there's nothing to `apt install` by
hand. But the image is built with `USE_CI=false`, so **ORB-SLAM3 and the colcon
workspace are not pre-compiled** — you build them once inside the container (see
"build ORB-SLAM3 + the workspace" below). That build persists on the host via
bind mounts, so it's a one-time cost.

Order: run the **host** steps first, then start the container, then build the
workspace **inside** it.

### One-time (host): fetch the ORB-SLAM3 submodules

`ORB_SLAM3` and `FastTrack` are git submodules that are empty on a fresh clone.
From the repo root:

```bash
git submodule update --init --recursive
```

### One-time (host): build the image

From the repo root (where the `Dockerfile` lives):

```bash
# CPU
sudo docker build --build-arg USE_CI=false -t orb-slam3-humble:22.04 .

# or NVIDIA
sudo docker build --build-arg USE_CI=false --build-arg TARGET=nvidia_gpu \
  -t orb-slam3-humble-nvidia:22.04 .
```

Enable X11 forwarding on the host (once) so the ORB-SLAM3 viewer can open:

```bash
echo "xhost +" >> ~/.bashrc && source ~/.bashrc
```

### Bring up the container

```bash
# CPU
sudo docker compose run orb_slam3_22_humble

# or NVIDIA
sudo docker compose run orb_slam3_22_humble_nvidia
```

The compose file already grants USB/camera access (`privileged: true`,
`/dev:/dev`, `network_mode: host`, `ipc: host`) — no extra flags needed. Every
new shell auto-sources ROS 2, the workspace, and `ros_env_vars.sh`
(`ROS_DOMAIN_ID=55` + CycloneDDS) through `~/.bashrc`.

### One-time (in container): build ORB-SLAM3 + the workspace (~15-25 min)

The image ships no compiled workspace, so build it once now:

```bash
# 1) ORB-SLAM3 Thirdparty (DBoW2, g2o, Sophus) + vocabulary + the library.
#    Run from this directory exactly — build.sh uses relative paths.
cd /home/orb/ORB_SLAM3
./build.sh

# 2) the ROS 2 wrapper packages
cd /root/colcon_ws
colcon build --symlink-install
source install/setup.bash
```

> Skipping this is the #1 cause of `package 'orb_slam3_ros2_wrapper' not found`
> at launch: the wrapper's CMake does `find_package(Sophus)` and
> `find_package(ORB_SLAM3)`, both of which step 1 provides. If step 2 already
> failed for this reason, clear the stale attempt first:
> `rm -rf /root/colcon_ws/{build,install}/orb_slam3_ros2_wrapper` then rebuild.

### Verify RealSense is present (first run only)

```bash
realsense-viewer                # Launch viewer
ros2 pkg list | grep realsense  # realsense2_camera present -> ROS wrapper OK
xeyes                           # a pair of eyes pops up -> X11 forwarding OK
```

> Opening a second shell into the same running container (e.g. one for the
> camera, one for SLAM):
> ```bash
> sudo docker exec -it <container_id> bash
> ```
> Find `<container_id>` with `sudo docker ps`. Each new shell auto-sources
> `ros_env_vars.sh` via `~/.bashrc`, so they already share `ROS_DOMAIN_ID=55` +
> CycloneDDS — no manual `source` needed.

---

## 1. Bring up the D435 with aligned depth

ORB-SLAM3 RGBD requires depth **registered to the color frame**.

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=front_camera
  align_depth.enable:=true \
  enable_sync:=true \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30
```

### Confirm topics and grab intrinsics

```bash
source ros_env_vars.sh            # matches ROS_DOMAIN_ID=55 + CycloneDDS
ros2 topic list | grep camera
ros2 topic echo --once /camera/camera/color/camera_info
```

> **Note the namespace.** Recent realsense-ros publishes under a *doubled*
> namespace: `/camera/camera/...`. Adjust topic names below to match what
> `ros2 topic list` actually shows on your machine.

From `camera_info`, the `k` array is `[fx, 0, cx, 0, fy, cy, 0, 0, 1]`.
**Use your own camera's values** — the numbers below are an example from one D435 unit:

| Param | Value (example) |
|-------|-----------------|
| fx    | 607.811767578125 |
| fy    | 607.1177368164062 |
| cx    | 321.6131591796875 |
| cy    | 246.10968017578125 |

`d` is all zeros (`plumb_bob`) because the color stream is factory-rectified →
distortion coefficients are 0.

### Confirm depth encoding

```bash
ros2 topic echo --once /camera/camera/aligned_depth_to_color/image_raw --field encoding
```

Must be `16UC1` (16-bit millimetres) → `DepthMapFactor: 1000.0`.
If it's `32FC1` (float metres), use `DepthMapFactor: 1.0` instead.

---

## 2. Edit the ORB-SLAM3 settings file

`rgbd.launch.py` hardcodes this path, so edit it in place (back it up first):

```bash
cd /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/orb_slam3_params
cp gazebo_rgbd.yaml gazebo_rgbd.yaml.bak
nano gazebo_rgbd.yaml
```

Replace the camera block (top of file through `DepthMapFactor`) with **your**
intrinsics:

```yaml
Camera.type: "PinHole"

Camera.fx: 607.811767578125
Camera.fy: 607.1177368164062
Camera.cx: 321.6131591796875
Camera.cy: 246.10968017578125

# Color stream is rectified -> zero distortion
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.k3: 0.0

Camera.width: 640
Camera.height: 480
Camera.fps: 30.0

# bf = baseline(m) * fx ; D435 IR baseline ~0.05 m -> 0.05 * fx
Camera.bf: 30.39

# rgb8 from realsense -> 1
Camera.RGB: 1

ThDepth: 40.0

# aligned depth is 16UC1 in millimetres -> divide by 1000 for metres
DepthMapFactor: 1000.0
```

Leave the `ORBextractor.*` and `Viewer.*` blocks below untouched.

> **No atlas line.** Do **not** add `System.LoadAtlasFromFile` — that keeps every
> run a fresh live-SLAM session with the origin pinned at startup.
> (Optionally add `System.SaveAtlasToFile: ./atlas` if you want to *save* a map
> for later; it never auto-loads, so it won't break the no-prebuilt-map flow.)

---

## 3. Edit the ROS params file

```bash
cd /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/ros_params
cp gazebo-rgbd-ros-params.yaml gazebo-rgbd-ros-params.yaml.bak
nano gazebo-rgbd-ros-params.yaml
```

```yaml
ORB_SLAM3_RGBD_ROS2:
  ros__parameters:
    robot_base_frame: base_footprint
    global_frame: map
    odom_frame: odom

    # Match your ACTUAL realsense topics (note doubled namespace)
    rgb_image_topic_name: /camera/camera/color/image_raw
    depth_image_topic_name: /camera/camera/aligned_depth_to_color/image_raw

    # base_footprint -> camera_link MOUNT transform (NOT a spawn pose).
    # Measure where the D435 physically sits on the robot.
    robot_x: 0.30
    robot_y: 0.0
    robot_z: 0.50
    robot_qx: 0.0
    robot_qy: 0.0
    robot_qz: 0.0
    robot_qw: 1.0

    visualization: true        # set false for headless once it works
    odometry_mode: false       # publishes map->base_footprint directly, no external odom needed
    publish_tf: true
    map_data_publish_frequency: 1000
    do_loop_closing: true
```

> The `robot_*` values are the **static camera mount extrinsic**, the single most
> common thing people get wrong. They are NOT where the robot starts. Get the
> rotation right too if the camera is tilted, or the logged pose will be offset.

---

## 4. Launch SLAM

Because the workspace was built with `--symlink-install` in Step 0, YAML edits are
live — no rebuild needed.

```bash
cd /root/colcon_ws
source install/setup.bash
ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py sensor_config:=rgbd
```

A viewer window appears. Move the camera slowly through the scene to initialize —
RGBD locks on fast since it has depth. You should see tracked feature points and
the camera trajectory.

---

## 5. Verify pose output

In a new shell **inside this container** (already on domain 55 + CycloneDDS via
`~/.bashrc`):

```bash
ros2 topic echo /robot_pose_slam
```

From a **different container** (e.g. another ROS 2 stack that should consume the
pose), it must match the DDS settings to see the topic:

```bash
export ROS_DOMAIN_ID=55
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /robot_pose_slam
```

`PoseStamped` in the `map` frame, updating as you move. The `map` origin is pinned
where SLAM initialized (your spawn point); `base_footprint` starts at identity
there. Every restart re-initializes a fresh origin.

Optional TF check:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

---

## 6. Record the pose

```bash
ros2 bag record -o slam_run1 /robot_pose_slam /tf /slam_info
```

---

## Key output interfaces

| Interface | Type | Purpose |
|-----------|------|---------|
| `/robot_pose_slam` | `geometry_msgs/PoseStamped` | Robot pose in `map` frame |
| `map -> base_footprint` | TF | Published directly (odometry_mode: false) |
| `/slam_info` | `slam_msgs/SlamInfo` | Map count, keyframes, tracking freq |
| `/map_data` | `slam_msgs/MapData` | Continuous pose-graph data |

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| `package 'orb_slam3_ros2_wrapper' not found` at launch | Workspace not built. Run Step 0's build: `/home/orb/ORB_SLAM3/build.sh`, then `colcon build --symlink-install` in `/root/colcon_ws`. |
| `find_package(Sophus)` / `find_package(ORB_SLAM3)` CMake error | Same cause — ORB-SLAM3 `build.sh` hasn't run, so Thirdparty libs aren't installed. Run it before `colcon build`. |
| Stuck on "waiting for images" | DDS mismatch — driver and SLAM must share `ROS_DOMAIN_ID=55` + CycloneDDS. Verify with `ros2 topic list`. |
| "Tracking LOST" frequently | Feature-poor scene (blank walls/floor). Raise `ORBextractor.nFeatures`, add texture, move slower. |
| Won't initialize | Depth scale mismatch — confirm encoding is `16UC1` and `DepthMapFactor: 1000.0`. |
| Pose offset / rotated vs reality | `robot_*` mount extrinsic wrong, especially rotation. |
| Pose jumps then settles | Loop closure correcting drift — expected. Note the `map` frame can jump on closure if feeding downstream. |
| No viewer window | X11 forwarding not set. `echo "xhost +" >> ~/.bashrc` on host; `source ~/.bashrc`. |
| `NetworkInterfaceAddress deprecated` warning | Harmless. To silence, update `/root/.ros/cyclonedds.xml` to the newer `<Interfaces><NetworkInterface .../></Interfaces>` syntax. |

---