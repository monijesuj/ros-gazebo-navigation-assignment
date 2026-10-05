#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="$PWD/.runtime/log"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/robot_demo/config/fastdds_udp.xml"
mkdir -p .runtime/log evidence
exec ros2 bag record -o "evidence/bag_$(date +%Y%m%d_%H%M%S)" /clock /tf /tf_static /odom /joint_states /scan /gps/fix /camera/depth_image /camera/camera_info /cmd_vel /map /planned_path /travelled_path /mission/status /mission/verification
