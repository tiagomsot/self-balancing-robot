# Shared C++ controller and editable robot routine

**Current hardware mode is IMU-only.** All adapters and the Nano now share
`ImuEstimator` and pitch PID with limited integral and anti-windup. Wheel gains
are zero. Follow the [current guide](../docs/imu-only-control.md); the distance
routine instructions below are historical reference and unavailable in this
encoder-free configuration. The old `--legacy-controller` option is rejected.

Start with **[`src/estimator.cpp`](src/estimator.cpp)** and
**[`src/controller.cpp`](src/controller.cpp)** to follow the IMU measurements,
complementary filter, pitch PID, integral limits and motor requests.

From the repository root in the VS Code **Ubuntu/WSL terminal**:

```bash
bash build_cpp.sh test             # Compile C++ and run its checks
bash run_gazebo.sh                # IMU-only balance in Gazebo / ROS 2
bash run_cpp_mujoco.sh             # IMU-only balance in CPU MuJoCo
bash build_cpp.sh nano             # Compile Nano firmware; no upload
bash optimize.sh                  # Shared C++ balance core; physics on GPU
```

Launchers rebuild their target. Stop and relaunch after editing. The old
measured-distance routine in `src/my_routine.cpp` remains reference code and is
disabled in normal simulation, because the robot has no wheel feedback.

## What is shared

`src/` contains the sequence executor, control equations, acceleration references,
yaw PI, torque allocation, estimation helpers, faults and torque-to-PWM mapping.
It has no ROS, Arduino or simulator dependencies and no heap allocation in its
control tick. Desktop bindings allocate objects during construction/destruction.
AVR uses 32-bit `double`; desktop normally uses 64-bit. Identical source does not
guarantee bit-identical arithmetic or physical trajectories.

ROS Python files handle messages, parameters, startup and recording around the
compiled core. `motion_core.py` records history; its state machine is now C++.
Simulator motor physics stays separate; its torque-to-PWM mapping calls C++.

Standard MuJoCo supplies sensor state to the same core. GPU batches, viewer and
optimization call the core on the **CPU**, transfer state/commands at 200 Hz,
and run **physics on CUDA**. Transfer costs are included in timing. The original
Warp controller is retained as historical source; `--legacy-controller` is
rejected by the normal launcher. Isaac Sim remains an optional future adapter;
no Isaac installation/integration is claimed.

## Nano build and hardware status

For learning and checking the electronics, use the
[complete sketch walkthrough](targets/nano/sketch-walkthrough.md),
[six component sketches](targets/nano/components/README.md), and
[wiring reference](targets/nano/wiring-reference.md).

The owner confirmed **no encoders**. Firmware explicitly disables position/speed
feedback and runs **balance only**. It cannot execute measured-distance moves
using the wheel state supplied by a simulator.

`targets/nano/RobotFirmware/BoardConfig.h` is deliberately unconfigured. Actual
driver/IMU identification, pins, motor polarities and sensor mounting are needed.
The driver template supports MPU-6050 over I2C and direction/enable PWM outputs;
compatibility with the real hardware must be checked. No motor pins are touched
while configuration is disabled. No upload was performed.

```bash
sudo apt install gcc-avr avr-libc arduino-core-avr python3-yaml
bash build_cpp.sh nano
bash build_cpp.sh components       # Compile all six diagnostic sketches
bash build_cpp.sh component 04_IMUPitch
```

This compiles for ATmega328P at 16 MHz. Configuration is generated from existing
YAML/geometry files. The current **unconfigured** image uses 16,042 bytes of
program storage and 639 bytes of static RAM, excluding worst-case stack use.
Configured driver code must be measured again. Board/bootloader need confirmation.

The template has a 200 Hz scheduler, shared complementary pitch estimator,
initial gyro calibration, and serial `a`/`d` arming/disarming. Simulation gains
remain unvalidated on hardware. Compilation and uploading are separate actions.

## Configuration and evidence

The ROS package's `config/balance.yaml` remains the gain/limit source and
`robot_geometry.json` the geometry source. The Nano generator checks that
wheel-feedback gains are zero, matching the simulator settings. Generated files
and binaries are ignored by Git. Default optimization runs validate their winner
in Gazebo, then update Kp, Kd and Ki in `balance.yaml`. A backup and
update record are saved in that search's output folder. Relaunch the simulation
to use the new values.

C++ checks exercise saturation, balance-priority steering, faults, stale/missing
data, time reversal, queue overflow, waiting and a complete four-leg sequence.
Python tests exercise native bindings, IMU-only state isolation, PID anti-windup
and promotion checks. Older `outputs/shared-cpp-*` round trips used wheel feedback
and are historical; current validation is linked from the IMU-only guide.
