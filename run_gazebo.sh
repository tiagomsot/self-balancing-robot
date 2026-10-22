#!/usr/bin/env bash
# Run from an Ubuntu/WSL terminal: bash run_gazebo.sh
set -e
set -o pipefail

task_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_launch_file="balance.launch.py"
task_launch_defaults=()
if [ "${1:-}" = "--passive" ]; then
  task_launch_file="passive.launch.py"
  task_launch_defaults=(paused:=false)
  shift
fi
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy was not found. Run this script inside the configured Ubuntu/WSL environment." >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
cd "$task_project_dir/ros2_ws"
colcon build --symlink-install --packages-select balancing_robot_sim
source install/setup.bash
exec ros2 launch balancing_robot_sim "$task_launch_file" "${task_launch_defaults[@]}" "$@"
