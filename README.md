# DAVE

[![Publish a Docker image (AMD64; Common X86_64 Linux Machine)](https://github.com/IOES-Lab/dave/actions/workflows/docker-amd64.yml/badge.svg)](https://github.com/IOES-Lab/dave/actions/workflows/docker-amd64.yml)
[![Publish a Docker image (ARM64; Apple Silicon)](https://github.com/IOES-Lab/dave/actions/workflows/docker-arm64v8.yml/badge.svg?branch=ros2)](https://github.com/IOES-Lab/dave/actions/workflows/docker-arm64v8.yml)

Documentation is currently at [http://dave-ros2.notion.site](http://dave-ros2.notion.site)

For contribution, do `pip3 install pre-commit && pre-commit install && pre-commit run --all-files` before commit.

---

# Installation (Native)

## Requirements
This only works in Ubuntu 24.04

### ROS2 Jazzy
Follow this installation guide https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### Gazebo harmonic
```
sudo apt install -y gz-harmonic python3-rosdep python3-rosinstall-generator python3-vcstool ros-jazzy-gz-plugin-vendor ros-jazzy-gz-ros2-control ros-jazzy-effort-controllers ros-jazzy-geographic-info ros-jazzy-image-view ros-jazzy-joint-state-publisher ros-jazzy-joy ros-jazzy-joy-teleop ros-jazzy-key-teleop ros-jazzy-moveit-planners ros-jazzy-moveit-simple-controller-manager ros-jazzy-moveit-ros-visualization ros-jazzy-pcl-ros ros-jazzy-robot-localization ros-jazzy-robot-state-publisher ros-jazzy-ros-base ros-jazzy-ros2-controllers ros-jazzy-rqt ros-jazzy-rqt-common-plugins ros-jazzy-rviz2 ros-jazzy-teleop-tools ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-tools ros-jazzy-urdfdom-py ros-jazzy-gz-ros2-control ros-jazzy-xacro ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

### Verify if you have everything alright
Source your ROS env
```
source /opt/ros/jazzy/setup.bash
```
Try to run gazebo
```
gz sim
```

## Download DAVE
Clone this repository to your workspace. Build and source your ws and try running this command
```
ros2 launch dave_demos dave_robot.launch.py z:=-5 namespace:=rexrov world_name:=dave_ocean_waves paused:=false

```

