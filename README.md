# DAVE

[![Publish a Docker image (AMD64; Common X86_64 Linux Machine)](https://github.com/IOES-Lab/dave/actions/workflows/docker-amd64.yml/badge.svg)](https://github.com/IOES-Lab/dave/actions/workflows/docker-amd64.yml)
[![Publish a Docker image (ARM64; Apple Silicon)](https://github.com/IOES-Lab/dave/actions/workflows/docker-arm64v8.yml/badge.svg?branch=ros2)](https://github.com/IOES-Lab/dave/actions/workflows/docker-arm64v8.yml)

Documentation is currently at [http://dave-ros2.notion.site](http://dave-ros2.notion.site)

For contribution, do `pip3 install pre-commit && pre-commit install && pre-commit run --all-files` before commit.




# AUTONOMOUS NAVIGATION AND MAP MATCHING
To Run the simulation and the navigation stack please refer to the README file under /navigation

[![Watch the video](https://img.youtube.com/vi/AXh7XIMAuGg/hqdefault.jpg)](https://www.youtube.com/embed/AXh7XIMAuGg)

[<img src="https://img.youtube.com/vi/AXh7XIMAuGg/hqdefault.jpg" width="600" height="300"
/>](https://www.youtube.com/embed/AXh7XIMAuGg)



1. The AUV is given the coordinates of the structure, its initial coordinates (0,0,0) and a target position to navigate to (close to the rig, but without being too close)
2. The ESKF error grows 
3. The AUV settles and hovers when it reaches the target position.
4. Once the Rig is visible to the LiDAR and that the map matching stabilizes its estimate, and provides it, a fix in the ESKF estimate takes place and brings back close to the ground truth (0:48)
