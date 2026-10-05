#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
echo 'ROS 2 Humble + Gazebo Fortress'
echo 'Choose a robot:'
select selected_robot in custom turtlebot3 jackal; do
  [[ -n "${selected_robot:-}" ]] && break
  echo 'Enter 1, 2 or 3.'
done
echo 'Choose a world:'
select selected_world in obstacle_course warehouse outdoor; do
  [[ -n "${selected_world:-}" ]] && break
  echo 'Enter 1, 2 or 3.'
done
exec ./run.sh robot:="$selected_robot" world:="$selected_world" "$@"
