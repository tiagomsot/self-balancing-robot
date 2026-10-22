#!/usr/bin/env bash
set -e
set -o pipefail
task_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bash "$task_project_dir/build_cpp.sh" test
task_env="${XDG_CACHE_HOME:-$HOME/.cache}/self-balancing-robot/gpu-venv"
task_python="${ROBOT_GPU_PYTHON:-$task_env/bin/python}"
if [ ! -x "$task_python" ] || ! "$task_python" -c 'import mujoco,yaml,numpy' >/dev/null 2>&1; then
  printf '%s\n' 'Install the optional MuJoCo environment first: bash setup_gpu.sh'
  exit 1
fi
task_output="$task_project_dir/outputs/cpp-mujoco-$(date +%Y%m%d-%H%M%S)-$$"
exec "$task_python" "$task_project_dir/cpp/targets/mujoco/run.py" --output "$task_output" "$@"
