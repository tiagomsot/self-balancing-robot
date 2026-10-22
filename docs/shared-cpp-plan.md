# Shared C++ robot software plan

Status, 2026-10-22: portable C++ core, editable routine, Gazebo/ROS bindings,
standard MuJoCo adapter and shared-C++/CUDA-physics optimization are implemented.
Nano firmware compiles; hardware wiring remains unconfigured and its mode is
balance-only because the owner confirmed no encoders. Isaac Sim is optional
future work. The sections below retain the design/implementation sequence;
see [current build instructions](../cpp/README.md) for what runs today.

## Goal

Write the robot's behaviour and control algorithms once, with a small, commented
routine that the owner can edit. Compile the portable source for either a classic
Arduino Nano or a desktop simulator. Keep each platform's sensor, clock, motor
and communication code in an adapter.

One source tree does not mean one executable: AVR and desktop targets need
different builds. Each simulator also needs its own model and adapter. The core
can be reused by Nano, Gazebo/ROS, standard MuJoCo and Isaac Sim; adding a target
is deliberate integration work, not automatic compatibility with every simulator.

## Planned directory

All project-owned C++ sources, headers, sketches and C++ tests go under `cpp/`:

```text
cpp/
  CMakeLists.txt                  Desktop library and test builds
  library.properties             Arduino library metadata
  src/
    RobotCore.h                  Public API and plain input/output types
    my_routine.cpp               MAIN FILE THE OWNER EDITS
    robot.cpp                    Coordinates each update
    behaviour.cpp                Nonblocking sequence executor
    estimator.cpp                Shared estimation from available measurements
    controller.cpp               Balance, speed, position and yaw control
    safety.cpp                   Arming, stale data, limits and latched faults
    motor_command.cpp            Shared request-to-duty mapping/calibration
    internal/                    Private headers
  targets/
    nano/RobotFirmware/
      RobotFirmware.ino          Thin setup()/loop() entry point
      nano_io.cpp                IMU, encoders, PWM, serial and hardware clock
    gazebo_ros/
      ros_adapter.cpp            ROS inputs, outputs, parameters and sim clock
    mujoco/
      mujoco_adapter.cpp         MuJoCo state, actuator and stepping interface
    isaac/
      isaac_adapter.cpp          Optional binding/plugin, if this target is added
  tests/                         Portable algorithm and adapter-contract tests
```

Only portable code goes in `src/`: Arduino compiles a library's `src/` tree
recursively, so ROS/CUDA/desktop headers must stay outside it. CMake selects the
requested desktop adapter explicitly. Existing ROS models, launch files and
canonical parameter files stay in their current package. Python remains useful
for optimization, plotting, experiment recording and optional bindings.

## Owner-facing routine

The proposed API is a queue of actions, built once during initialization:

```cpp
void defineRoutine(Routine& robot) {
    robot.waitUntilBalanced(2.0f); // Require two seconds of settled balance.
    robot.move(0.50f, 0.25f);      // Metres; maximum forward speed in m/s.
    robot.stop(2.0f);             // Brake, settle, then dwell for two seconds.
    robot.rotate(180.0f, 50.0f);   // Relative degrees; maximum degrees/second.
    robot.wait(1.0f);             // Hold still while balancing for one second.
    robot.repeat();              // Repeat movement after the startup step.
}
```

This API is now implemented in `cpp/src/my_routine.cpp`. Positive/negative motion
and rotation must be documented. Distance/angle steps finish using feedback, not
by assuming that speed multiplied by time equals the requested movement.

Every platform's runtime repeatedly reads measurements, calls `robot.update()`
and applies the returned command. Motion, waiting and settling never block the
balance update. `stop()` and `wait()` keep balancing active; `disarm()` explicitly
disables the motors. A missing sensor or fall fault has its own latched handling.
Terminal completion holds balance. Queue capacity is fixed and overflow is
reported rather than silently dropping actions.

## Shared contracts and configuration

- Define plain `Measurements`, `EstimatedState`, `MotionTarget`, `MotorCommand`,
  `RobotConfig` and `Status` types. State includes measurement validity, freshness
  and feedback capabilities. Distinguish requested, bounded and delivered effort.
- Use metres, seconds, radians and documented motor-command units internally.
  Convert degrees in the friendly routine API. Keep x-forward/y-left/z-up,
  motor polarity and IMU mounting conventions consistent across targets.
- Pass elapsed time explicitly. The Nano adapter handles hardware-counter wrap;
  simulator adapters use simulation time and handle reset/pause. Bound or reject
  invalid time steps; do not mix wall time with simulated control time.
- Target a 200 Hz balance update initially. Behaviour can update more slowly,
  but never delays the fast loop. Verify actual execution time on the Nano.
- Keep algorithms free of `Arduino.h`, ROS, simulator APIs, file access and
  operating-system dependencies. Use fixed-size storage and avoid heap allocation
  in the control loop. Check AVR flash, SRAM and stack headroom with the real build.
- Keep controller parameters in the canonical configuration and measured geometry
  in `robot_geometry.json`. Generate an AVR-compatible configuration header from
  those inputs instead of manually copying gains. Routine action values belong in
  `my_routine.cpp`; robot limits still bound those values. Any runtime override
  has an explicit, documented precedence.
- Compile-time platform differences are limited to build plumbing and device
  interfaces. Do not branch the control algorithm on a simulator name.
- The simulator contains the physical motor/driver model. Shared command-to-PWM
  calibration belongs in the software path; simulated motor physics does not run
  on the Nano to replace actual motor feedback.

## Build and run experience

Provide one small build helper/menu with targets for desktop tests, Gazebo, Nano
and later MuJoCo/Isaac. CMake builds desktop code; Arduino CLI builds the firmware
using the same portable library. The helper must explain missing tools and leave
uploading separate from compilation. VS Code opens the source and runs these
commands; it does not supply a different controller implementation.

Keep build products in target-specific generated directories. Builds record the
source/configuration version used. The Nano build compiles only its adapter and
portable core; a desktop build never attempts to compile Arduino pin code.

## Implementation order and completion checks

1. **Portable core and editable routine.** Establish the data types, bounded
   nonblocking action queue and annotated `my_routine.cpp`. Port the current
   balance equations, reference ramps, yaw controller, torque allocation, safety
   and movement sequencing into C++. Keep the current active gains as the
   migration reference. Check behaviour on recorded/synthetic feedback before
   changing a running simulator.
2. **Gazebo/ROS integration first.** Replace the active Python algorithm with a
   thin adapter calling the C++ core. Keep existing telemetry and recording
   contracts, or migrate their consumers together. Verify unchanged input/output
   conventions, balance recovery, stop/turn behaviour and a complete round trip.
   Preserve failed runs and compare against the pre-refactor records. Do not
   silently promote the rejected GPU candidate.
3. **Nano build and hardware adapter.** Compile the same core for the actual
   board and report memory use. Implement IMU estimation/calibration, fixed-rate
   scheduling, motor polarity/PWM and serial telemetry using confirmed hardware.
   Confirm wheel-feedback availability. Without encoders or another suitable
   measurement, position/speed feedback and accurate 0.50 m moves cannot simply
   reuse perfect simulated state. Balance-only mode must explicitly disable
   unsupported position/distance features. Firmware compilation does not establish
   hardware readiness or authorize upload automatically.
4. **Standard MuJoCo adapter.** Run the exact desktop C++ core with MuJoCo through
   its API. Match measurement availability, sensor sampling, delays, actuator
   commands and conventions. Compare matched trials with Gazebo and recorded
   hardware data; simulator physics can still produce different trajectories.
5. **Optimization integration.** Evaluate this shared implementation first. Use
   Gazebo for ROS integration evidence and MuJoCo for additional matched tests.
   For the existing Warp GPU batch, either call the shared CPU core with profiled
   transfers or deliberately integrate a device-compatible subset. Ordinary C++
   does not automatically run inside a GPU kernel. Avoid introducing another
   independently maintained controller; require numerical parity tests for any
   generated/device path before using its scores to choose gains.
6. **Isaac Sim, if needed.** Add a ROS adapter connection or native binding to
   the same core. This is an optional target, not a dependency for completing the
   Nano/Gazebo portfolio project. Benchmark CPU/GPU physics for its workload.

The first usable milestone is **edit one C++ routine, build it, and see that
routine execute in Gazebo**, with the portable core also compiling for AVR.
Hardware I/O and other simulator targets follow without rewriting the algorithm.

## Verification that matters

Use meaningful checks for saturation, steering headroom, stale sensors, fall
latching, irregular/invalid time steps, heading wrap, sequence completion and
nonblocking waits. Compare the shared code on the same recorded inputs across
targets within defined floating-point tolerances. Run actual Gazebo balance and
shuttle recordings after switching implementations. Include an AVR compile and
memory report; measure real loop timing when hardware is available. One passing
desktop build is not proof of Nano fit or real-robot stability.

## Details to confirm before hardware integration

Nano variant/processor, exact motor-driver and IMU markings, pin wiring and PWM
pins, motor polarity, IMU axes, encoder availability/resolution, and the current
Arduino sketch if one exists. Record measured motor response, battery behaviour
and sensor calibration when available. These details do not block the portable
core or initial Gazebo adapter.

## Current evidence to preserve

The active nominal gains remain Kp 1.0, Kd 0.10, position 0.15, velocity 0.35,
with a total torque request cap of 0.040 N m. The first GPU Bayesian candidate
improved its nominal objective but failed the subsequent Gazebo -10-degree test;
it was not promoted. Keep that result visible during the migration.

References: [Arduino build process](https://docs.arduino.cc/arduino-cli/sketch-build-process),
[Arduino library layout](https://docs.arduino.cc/arduino-cli/library-specification/),
[MuJoCo control loop](https://mujoco.readthedocs.io/en/stable/programming/simulation.html),
[Warp C++/CUDA integration](https://nvidia.github.io/warp/latest/user_guide/programming_model/cpp_cuda_workflows.html).
