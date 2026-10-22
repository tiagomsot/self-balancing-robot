#!/usr/bin/env bash
# Keep optional GPU dependencies on WSL's Linux filesystem, away from ROS.
set -e
set -o pipefail
task_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_env="${XDG_CACHE_HOME:-$HOME/.cache}/self-balancing-robot/gpu-venv"
mkdir -p "$(dirname -- "$task_env")"
if ! python3 -m venv "$task_env"; then
  printf '%s\n' 'Ubuntu venv support is required: sudo apt install python3-venv' >&2
  exit 1
fi
"$task_env/bin/python" -m pip install --no-compile -r "$task_project_dir/gpu/requirements.txt"
printf '\nGPU environment installed. Run: bash run_gpu.sh\n'
