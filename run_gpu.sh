#!/usr/bin/env bash
# Optional GPU batch experiment. Run inside the configured Ubuntu/WSL distro.
set -e
set -o pipefail
task_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_env="${XDG_CACHE_HOME:-$HOME/.cache}/self-balancing-robot/gpu-venv"
task_python="${ROBOT_GPU_PYTHON:-$task_env/bin/python}"
if [ ! -x "$task_python" ] || ! "$task_python" -c 'import mujoco_warp, warp, yaml' >/dev/null 2>&1; then
  printf '%s\n' 'Install the optional GPU environment once: bash setup_gpu.sh'
  exit 1
fi
bash "$task_project_dir/build_cpp.sh" test
if [ "${1:-}" = "--view" ]; then
  shift
  exec "$task_python" "$task_project_dir/gpu/live_view.py" "$@"
fi
task_output="$task_project_dir/outputs/gpu-batch-$(date +%Y%m%d-%H%M%S)-$$"
exec "$task_python" "$task_project_dir/gpu/parallel_sim.py" --output "$task_output" "$@"
