#!/usr/bin/env bash
# Run in Ubuntu/WSL. Validate the winner, then update the normal simulation gains.
set -e
set -o pipefail
task_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_env="${XDG_CACHE_HOME:-$HOME/.cache}/self-balancing-robot/gpu-venv"
task_python="${ROBOT_GPU_PYTHON:-$task_env/bin/python}"
bash "$task_project_dir/build_cpp.sh" test
if [ ! -x "$task_python" ] || ! "$task_python" -c 'import mujoco_warp, warp, sklearn, scipy, yaml' >/dev/null 2>&1; then
  printf '%s\n' 'Install/update the GPU environment once: bash setup_gpu.sh'
  exit 1
fi
task_output="$task_project_dir/outputs/gpu-optimization-$(date +%Y%m%d-%H%M%S)-$$"
exec "$task_python" "$task_project_dir/gpu/bayes_search.py" --output "$task_output" "$@"
