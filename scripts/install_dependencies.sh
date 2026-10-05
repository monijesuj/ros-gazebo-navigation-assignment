#!/usr/bin/env bash
set -euo pipefail
# Ubuntu 22.04 with ROS 2 Humble and the OSRF repository already configured.
sudo apt-get update
sudo apt-get install -y ignition-fortress ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-rosbag2 python3-colcon-common-extensions python3-numpy python3-pytest ffmpeg
