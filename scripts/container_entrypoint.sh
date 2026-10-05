#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
# Keep xvfb-run below PID 1 so Xvfb can send its startup-ready signal.
"$@" &
container_child=$!
trap 'kill -TERM "$container_child" 2>/dev/null || true' TERM INT
wait "$container_child"
