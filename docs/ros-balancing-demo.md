# Preset ROS / Gazebo balancing demo

The current default is [IMU-only PID](imu-only-control.md), matching the robot's
lack of encoders. Historical gains/results below used different feedback and
are not the active hardware configuration.

The current measured geometry is shared through `config/robot_geometry.json`.
A fresh +5-degree nominal check with the 420 g body passed in
`outputs/measured-geometry-balance-check`. The existing searched gains were
retained; historical tables below describe the earlier geometry.

## Run

From the repository root in the VS Code Ubuntu/WSL terminal:

```bash
bash run_gazebo.sh
```

The script sources ROS 2 Jazzy, builds this workspace, and opens the balanced
Gazebo demo. The robot starts at a 5-degree lean. Physics starts automatically
after creation and connection of the controller and bridges. No controller
exercise needs to be completed to run it. Stop with Ctrl+C, or close Gazebo.
The default actuator is now an averaged TT motor / L293D surrogate; see
[motor-driver-model.md](motor-driver-model.md) for its assumptions and rating checks.

For moving between two waypoints, stopping and turning 180 degrees, run
`bash run_shuttle.sh`. See the [shuttle guide](shuttle-demo.md). Motion is
opt-in; the ordinary balance launch continues holding its starting position.

Try another initial lean with:

```bash
bash run_gazebo.sh initial_tilt_deg:=-5
```

## Controller and preset parameters

The ROS node reads the simulated IMU and wheel joint states. Its inner pitch
PD commands **total torque**, split equally between the two wheels. A slow
outer loop adjusts the desired lean using wheel position and speed, keeping
the demo near its starting point.

```text
target_pitch = clip(-position_gain*x - velocity_gain*v, +/-target_tilt_limit)
requested_total_torque = kp*(pitch - target_pitch) + kd*pitch_rate
commanded_total_torque = clip(requested_total_torque, +/-total_torque_limit)
each_wheel_request = commanded_total_torque / 2
delivered_wheel_torque = motor_driver_model(each_wheel_request, shaft_speed, supply)
```

All angles in those equations are radians. Positive pitch is a forward lean;
positive wheel torque drives forward. The wheel joint angle is relative to
the body, so wheel odometry uses joint angle plus body pitch, relative to its
initial value. Its speed uses joint angular velocity plus body pitch rate.
This assumes rolling without slip.

Presets are in `ros2_ws/src/balancing_robot_sim/config/balance.yaml`:

| Parameter | Value | Units |
| --- | --- | --- |
| Inner Kp | 1.0 | N m / rad |
| Inner Kd | 0.1 | N m s / rad |
| Total torque request cap | 0.040 | N m, shared across both wheels |
| Position gain | 0.15 | rad / m |
| Velocity gain | 0.35 | rad / (m/s) |
| Desired lean cap | 5 | degrees |
| Fall cutoff | 35 | degrees from upright |
| Sensor timeout | 0.25 | wall-clock seconds |

Edit the YAML and restart to apply changes. Controller parameters are read-only
during a run. Setting both outer gains to zero gives the original pitch-only
PD, which can balance while drifting. The automatic-start option is also in
the YAML; set it false to inspect the paused model before pressing Play.

These settings come from the [automated gain/cap search](controller-tuning.md).
They improve nominal recovery and reduce deadband drift, but do not pass every
stress test: long deadband runs retain oscillation, and a reduced-battery
10-degree start fell. The original full preset is `config/balance-baseline.yaml`;
`config/balance-tuned.yaml` preserves this selected preset independently of
future default edits. See the tuning report for exact recorded outcomes.

The motor-driver node converts requests to quantized PWM, estimates current
and delivered shaft torque, then commands Gazebo's `ApplyJointForce` system.
Use `actuator_mode:=ideal` to bypass those actuator effects. The balance node runs when IMU messages arrive;
the configured sensor rate is 200 Hz, with a 1 ms Gazebo physics step.
ROS transport and scheduling add delay. On a large fall or invalid sensor
input, the node latches zero torque and asks for a restart. It also sends zero
torque when sensor messages become stale. Stopping the controller shuts down
its launched simulation.

## Inspect or record

Topics include `/imu/data`, `/joint_states`, `/left_wheel/torque_request`,
`/right_wheel/torque_request`, `/left_wheel/torque`, `/right_wheel/torque`,
`/motors/telemetry`, and `/balance/telemetry`. The controller telemetry array is:

```text
[time_s, pitch_rad, rate_rad_s, x_m, v_m_s, target_rad, requested_nm, commanded_nm]
```

Motor telemetry carries the delivered torque, duty, current estimates, supply,
driver-loss proxy, and rating flags; the array's dimension label names each
field. The balance node logs pitch, estimated position, and commanded torque every two wall-clock
seconds. To reproduce the headless response check, in a sourced terminal from
the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 analysis/check_gazebo_balance.py --output outputs/gazebo-balance-5deg
```

The check starts an isolated headless simulation, records ten simulated
seconds, checks recovery and torque limits, and shuts down its own processes.
It saves `summary.json`, controller/motor/IMU CSV, effective parameters,
robot/world snapshots, source hashes, and `launch.log`. Use distinct output
folders and `--tilt -5` or `--tilt 10` for other initial conditions.

Earlier ideal-actuator checks on 2026-10-21 passed +5, -5, and +10 degree starts
over ten seconds, with less than 0.002 degrees of tilt in the final second and
less than 0.1 mm final wheel-odometry displacement. Those results used the old
0.10 velocity gain and ideal delivered torque. They are a historical baseline,
not the current motor-model response. See the motor model guide and saved
reports for the constrained actuator results.

## Scope

This demo uses ideal simulator orientation and wheel feedback. The physical
MPU-6050 needs an orientation estimator; physical encoders are not confirmed.
Motor speed/voltage effects and driver rating diagnostics are now modelled
approximately. Current is not artificially clamped to the chip rating.
PWM pulse compliance, detailed switching and thermal behaviour, gearbox
backlash, and sensor noise/delay remain unverified or unmodelled. The request
cap is a scenario setting. Simulation recovery is not proof of physical
hardware performance.

The original unpowered launch remains available as `passive.launch.py`, and
the standalone Python experiment and student controller exercise remain
available separately. See [the first model guide](ros-first-model.md) for
geometry and assumptions.

Reference: [Gazebo ApplyJointForce](https://gazebosim.org/api/sim/8/jointforcecmdcomponent.html).
