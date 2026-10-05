#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
if [[ -n "${ROBOT_DEMO_DEPS:-}" ]]; then
  export AMENT_PREFIX_PATH="$ROBOT_DEMO_DEPS:$AMENT_PREFIX_PATH"
  export LD_LIBRARY_PATH="$ROBOT_DEMO_DEPS/lib:$LD_LIBRARY_PATH"
  export PYTHONPATH="$ROBOT_DEMO_DEPS/local/lib/python3.10/dist-packages:$ROBOT_DEMO_DEPS/lib/python3.10/site-packages:$PYTHONPATH"
fi
if ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  echo 'Missing ros_gz_bridge. Run ./scripts/install_dependencies.sh first.' >&2; exit 1
fi
mkdir -p .runtime/log evidence
export ROS_LOG_DIR="$PWD/.runtime/log"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/robot_demo/config/fastdds_udp.xml"
export IGN_PARTITION="${IGN_PARTITION:-robot_demo}"
# Gazebo/ROS caches stay inside the repository for sandboxed execution.
export HOME="$PWD/.runtime"
if [[ "${ROBOT_DEMO_SOFTWARE_RENDERING:-0}" == 1 ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa
fi
colcon build --symlink-install --packages-select robot_demo
source install/setup.bash
exec ros2 launch robot_demo demo.launch.py report_file:="$PWD/evidence/mission_report.json" "$@"
