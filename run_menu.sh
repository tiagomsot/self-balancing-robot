#!/usr/bin/env bash
# Run in the VS Code Ubuntu/WSL terminal: bash run_menu.sh
set -o pipefail

task_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$task_project_dir" || exit 1
task_params="$task_project_dir/ros2_ws/src/balancing_robot_sim/config/balance.yaml"

# Keep the menu alive when Ctrl+C stops the foreground simulation.
trap 'printf "\nStopping simulation; returning to menu...\n"' INT

task_menu=main
while true; do
  if [ "$task_menu" = main ]; then
    cat <<'MENU'

========== Self-balancing robot ==========
  1) IMU-only balance - Gazebo / ROS 2
  2) IMU-only balance - MuJoCo
  3) IMU-only GPU live view - 16 robots
  4) Advanced tools and comparisons
  0) Exit

Actual robot: IMU, no encoders. Simulators use the shared Nano estimator and PID.
Position/speed are logged for evaluation, never used by the controller.
The old measured-distance routine is unavailable without motion feedback.
Passing optimizations update the gains used here automatically.
Close the simulation or press Ctrl+C here to return to the menu.
MENU
    task_prompt="Choose [0-4]: "
  else
    cat <<'MENU'

========== Advanced tools and comparisons ==========
  1) GPU live view - 16 robots balancing
  2) GPU balance batch - results and timing, no window
  3) Optimize balance gains - Bayesian search with GPU physics
  4) Balance with ideal motors - Gazebo comparison
  5) Unpowered fall - Gazebo comparison
  6) Original Python PD experiment - sliders and plots
  7) Build and test the shared C++ core
  8) Compile Nano balance-only firmware (no upload)
  0) Back to main menu

Optimization validates its winner, then updates the active gains automatically.
The Nano build compiles only; the real hardware is still unconfigured.
MENU
    task_prompt="Choose [0-8]: "
  fi
  if ! read -r -p "$task_prompt" task_choice; then
    printf '\n'
    exit 0
  fi

  task_command=()
  case "$task_menu:$task_choice" in
    main:0|main:q|main:Q) exit 0 ;;
    main:4) task_menu=advanced; continue ;;
    advanced:0|advanced:q|advanced:Q) task_menu=main; continue ;;
    main:1)
       task_command=(bash "$task_project_dir/run_gazebo.sh"
          gui:=true "params_file:=$task_params" actuator_mode:=motor_driver) ;;
    main:2) task_command=(bash "$task_project_dir/run_cpp_mujoco.sh") ;;
    main:3) task_command=(bash "$task_project_dir/run_gpu.sh" --view --worlds 16) ;;
    advanced:1) task_command=(bash "$task_project_dir/run_gpu.sh" --view --worlds 16) ;;
    advanced:2) task_command=(bash "$task_project_dir/run_gpu.sh") ;;
    advanced:3) task_command=(bash "$task_project_dir/optimize.sh") ;;
    advanced:4) task_command=(bash "$task_project_dir/run_gazebo.sh"
          gui:=true "params_file:=$task_params" actuator_mode:=ideal) ;;
    advanced:5) task_command=(bash "$task_project_dir/run_gazebo.sh" --passive
          gui:=true initial_tilt_deg:=5.0) ;;
    advanced:6)
       task_python=python3
       if [ -x "$task_project_dir/.venv/bin/python" ]; then
         task_python="$task_project_dir/.venv/bin/python"
       fi
       if ! "$task_python" -c 'import numpy, scipy, matplotlib' 2>/dev/null; then
         printf '%s\n' 'Python demo dependencies are missing in Ubuntu. Install them once:' \
           '  sudo apt install python3-venv python3-tk' \
           '  python3 -m venv .venv' \
           '  .venv/bin/python -m pip install -r requirements.txt'
         continue
       fi
       # This idealized pitch-only model has its own reference PD gains.
       task_output="$task_project_dir/outputs/menu-pd-$(date +%Y%m%d-%H%M%S)-$$-$RANDOM"
       task_command=("$task_python" "$task_project_dir/run_simulation.py"
          --controller reference --output "$task_output") ;;
    advanced:7) task_command=(bash "$task_project_dir/build_cpp.sh" test) ;;
    advanced:8) task_command=(bash "$task_project_dir/build_cpp.sh" nano) ;;
    *) printf 'Please choose one of the options shown above.\n'; continue ;;
  esac

  printf '\nStarting: '
  printf '%q ' "${task_command[@]}"
  printf '\nGazebo builds its package; the first GPU batch compiles kernels.\n\n'
  "${task_command[@]}"
  task_status=$?
  if [ "$task_status" -ne 0 ] && [ "$task_status" -ne 130 ]; then
    printf '\nCommand exited with code %s. Check the messages above.\n' "$task_status"
  fi
done
