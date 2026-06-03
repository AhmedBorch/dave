# Navigation — Map Matching + ESKF

Autonomous navigation stack for the BlueROV2: a global ESKF fusing IMU, DVL, depth, and heading, with optional absolute pose corrections from a LiDAR map-matcher, driven by a goal-directed planner and PD controller.

---

## Table of Contents

- [Docker Setup](#docker-setup)
- [Running the Stack](#running-the-stack)
  - [1. Simulation](#1-simulation)
  - [2. ESKF State Estimator](#2-eskf-state-estimator)
  - [3. Map Matcher](#3-map-matcher)
  - [4. Planner & Controller](#4-planner--controller)
  - [5. Joystick Teleoperation](#5-joystick-teleoperation)

---

## Docker Setup

Activate the virtual environment and launch your Docker container using `dockwater`:

```bash
source dave_ws/dave_venv/bin/activate
cd dave_ws/src/dockwater
./run.bash <your_docker_image>      # e.g. dave_custom_v2_latest
```

To open additional terminals into the running container:

```bash
./join.bash <your_docker_container> # e.g. dave_custom_v2_latest_runtime
```

### NVIDIA GPU

If Gazebo fails to render, you may need to symlink the NVIDIA libraries to match your host driver version. Check your version with `nvidia-smi`, then:

```bash
ln -sf libGLX_nvidia.so.<version> /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.0
ln -sf libEGL_nvidia.so.<version> /usr/lib/x86_64-linux-gnu/libEGL_nvidia.so.0
```

> Example for driver `580.159.04`:
> ```bash
> ln -sf libGLX_nvidia.so.580.159.04 /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.0
> ln -sf libEGL_nvidia.so.580.159.04 /usr/lib/x86_64-linux-gnu/libEGL_nvidia.so.0
> ```

---

## Running the Stack

Each component runs in a separate terminal. Launch them in the order shown below.

### 1. Simulation

```bash
ros2 launch dave_demos dave_robot.launch.py \
  z:=-0.5 \
  namespace:=bluerov2 \
  world_name:=dave_ocean_waves \
  paused:=false \
  open_virtual_joystick:=true \
  open_qgc:=false
```

### 2. ESKF State Estimator

**Without map-matcher corrections (IMU + DVL + depth + heading only):**

```bash
ros2 launch eskf eskf.launch.py
```

**With map-matcher pose updates:**

```bash
ros2 launch eskf eskf.launch.py map_pose_topic:=/map_matching/average_pose
```

**Side-by-side comparison (both instances simultaneously):**

```bash
ros2 launch eskf eskf_comparison.launch.py
```

Publishes the base ESKF on `/model/bluerov2/eskf/odom` and the map-aided ESKF on `/model/bluerov2/eskf_map/odom`.

### 3. Map Matcher

```bash
ros2 launch map_matching map_matching.launch.py \
  full_map_path:=src/dave/navigation/map_matching/maps/instruments_rig.pcd
```

Key output topics:

| Topic | Description |
|---|---|
| `/map_matching/vehicle_pose` | Raw GICP pose estimate (base_link in map frame) |
| `/map_matching/average_pose` | Voted/stable pose (modal cluster of last 30 estimates) |
| `/map_matching/average_cloud` | LiDAR scan transformed by the voted pose |
| `/map_matching/vehicle_euler` | Raw Euler angles — x=roll, y=pitch, z=yaw (rad) |
| `/map_matching/average_euler` | Voted Euler angles — x=roll, y=pitch, z=yaw (rad) |

### 4. Planner & Controller

```bash
ros2 launch goto_planner goto_planner.launch.py
```

Send a goal at runtime:

```bash
ros2 topic pub --once /goto_planner/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 8.0, y: 0.0, z: -2.5}}}"
```

### 5. (Optional) Joystick Teleoperation

> Connect the joystick **before** starting the Docker container.

```bash
ros2 launch goto_planner joy_teleop.launch.py
```
