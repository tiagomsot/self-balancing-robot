#!/usr/bin/env bash
# Run from Ubuntu/WSL: bash build_cpp.sh [test|gazebo|nano|mujoco|components]
set -e
set -o pipefail
task_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_target="${1:-test}"
shift || true
case "$task_target" in
  test)
    cmake -S "$task_project_dir/cpp" -B "$task_project_dir/cpp/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build "$task_project_dir/cpp/build" --parallel 2
    ctest --test-dir "$task_project_dir/cpp/build" --output-on-failure ;;
  gazebo) exec bash "$task_project_dir/run_gazebo.sh" "$@" ;;
  nano) exec bash "$task_project_dir/cpp/targets/nano/build.sh" "$@" ;;
  component) exec bash "$task_project_dir/cpp/targets/nano/build.sh" --component "$@" ;;
  components) exec bash "$task_project_dir/cpp/targets/nano/build.sh" --components ;;
  mujoco) exec bash "$task_project_dir/run_cpp_mujoco.sh" "$@" ;;
  *) printf '%s\n' 'Usage: bash build_cpp.sh [test|gazebo|nano|mujoco|components|component NAME]'; exit 2 ;;
esac
