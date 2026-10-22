#!/usr/bin/env bash
# Balance, drive 0.50 m, stop, turn 180 degrees, and repeat.
set -e
set -o pipefail
task_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$task_project_dir/run_gazebo.sh" shuttle:=true "$@"
