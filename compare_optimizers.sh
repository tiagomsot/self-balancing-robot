#!/usr/bin/env bash
# Matched BO/DE comparison, independent finalist checks, then active gain update.
set -euo pipefail
task_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bash "$task_project_dir/build_cpp.sh" test
task_env="${XDG_CACHE_HOME:-$HOME/.cache}/self-balancing-robot/gpu-venv"
task_python="${ROBOT_GPU_PYTHON:-$task_env/bin/python}"
if [ ! -x "$task_python" ]; then
  printf '%s\n' 'Install the GPU environment first: bash setup_gpu.sh'
  exit 1
fi
task_output="$task_project_dir/outputs/optimizer-comparison-$(date +%Y%m%d-%H%M%S)-$$"
exec "$task_python" "$task_project_dir/gpu/compare_optimizers.py" --output "$task_output" "$@"
