#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="$PWD/.runtime/log"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/robot_demo/config/fastdds_udp.xml"
mkdir -p .runtime/log evidence
exec ros2 bag record --use-sim-time -o "evidence/bag_$(date +%Y%m%d_%H%M%S)" /clock /tf /tf_static /robot_description /odom /joint_states /scan /gps/fix /imu/data /localization/pose /camera/image /camera/depth_image /camera/camera_info /cmd_vel /cmd_vel/nav /map /planned_path /travelled_path /mission/status /mission/verification /safety/status
