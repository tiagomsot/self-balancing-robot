# IMU-only control: the actual encoder-free robot

**Updated result:** the paused-start watchdog race has been corrected and a new
512-candidate search completed. The active profile and repeated Gazebo results
are in [the current optimization report](optimization-results.md). The earlier
correction results at the end of this page are retained as history.

The owner confirmed two-wire TT motors with **no encoders**. The earlier
four-gain experiments used wheel position/speed feedback and ideal orientation.
That was a modeling error for this hardware. Their data is retained as history;
those results are not hardware-ready gains.

The corrected signal path is:

```text
MPU-6050 accelerometer + gyro
  -> shared C++ complementary pitch/roll estimator
  -> shared C++ pitch PID with integral limiting and anti-windup
  -> shared torque-request-to-PWM mapping
  -> TT motor / driver model, or Nano output pins
```

`wheel_feedback_enabled` is false; `position_gain` and `velocity_gain` must both
remain zero. The adapters reject unavailable wheel feedback. Wheel angles,
speed, world position and true orientation are evaluation-only. Physical motor
speed still affects back EMF inside motor physics; it is not a controller input.

## Sensor model and common code

The virtual accelerometer and gyro are at the measured IMU position, 6 cm above
the axle. Their readings include that site's motion. MuJoCo supplies these
site-frame readings as described in its
[sensor documentation](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-accelerometer).
The controller no longer receives exact simulator pitch.

Simulated readings are quantized to the configured MPU-6050 scales: +/-2 g and
+/-250 degrees/s, at 16384 counts/g and 131 counts/(degree/s). Actual MPU readings
are already quantized. All adapters and the Nano use `ImuEstimator` in
[`estimator.cpp`](../cpp/src/estimator.cpp), with the existing 0.25-second
complementary filter. Once initialized, it uses gyro propagation during
near-free-fall or impact readings instead of mistaking unreliable acceleration
for gravity.

CPU/GPU MuJoCo start from a synthetic stationary IMU reading before release,
assuming completed calibration. The Nano performs its 500-sample stationary
gyro calibration before arming. Gazebo initializes from its first usable
accelerometer sample. Hardware mounting axes remain to be checked.

The hardware driver sets DLPF mode 3. The simulators add one sample (5 ms at
200 Hz) to approximate its documented 4.8-4.9 ms group delay. This does not
reproduce the full chip frequency response, measured noise, residual bias or
transport jitter. See the [sourced IMU specifications](components.md).

At 200 Hz, the balance law is
`request = Kp * pitch + Kd * pitch_rate + limited_integral`.
The integral accumulates `Ki * pitch`, is limited to half the request cap, and
does not wind up further into saturation. `Ki=0` gives PD. The combined request
cap remains 0.040 N m; it does not regulate physical current.

The GPU runs contact physics on CUDA and the same C++ controller on CPU.

## Run and optimize

In the project's Ubuntu/WSL terminal:

```bash
bash run_menu.sh
# 1: Gazebo / ROS 2 IMU-only balance
# 2: CPU MuJoCo IMU-only balance
# 3: GPU view, identical active gains with different starting tilts

bash optimize.sh --trials 256
bash build_cpp.sh nano
```

The optimizer searches **Kp, Kd and Ki** only. Bounds are 0.30-4.0, 0.020-0.80
and 0-2.0 respectively; wheel gains remain zero. Every candidate receives eight
eight-second starting-state tests. A selected candidate gets six additional
twelve-second tests and independent Gazebo recovery checks before any active
update. The score uses true pitch recovery, tipping rate and electrical
predictions. Drift is logged without demanding position/speed hold.

`bash compare_optimizers.sh --trials 256` compares the corrected three-gain
search. `--check-shuttle` is rejected: the old measured-distance routine needs
feedback absent from this robot. Feedback mode, gain shape and source hashes
prevent old search results from being promoted into this controller.

## Motion and hardware limits

IMU-only balance can remain upright while drifting. This implementation cannot
verify stationary wheel velocity or a measured 0.50 m leg. `my_routine.cpp` is
retained as reference, outside the normal hardware simulation. A later routine
could use timed lean commands and gyro-based turns, with honest distance and
heading drift limits. It must not borrow perfect simulator position.

Successful simulation remains separate from hardware validation. Nano outputs
stay disabled until the actual pins, motor directions, driver and IMU mounting
are checked using the component sketches.

## Earlier correction snapshot, before the paused-start watchdog fix

The earlier **provisional** gains were Kp=1.5384553561057395, Kd=0.020 and
Ki=2.0, with both wheel gains zero. The correction is implemented; robust
recovery over the full tested range is not yet achieved.

| Check with IMU quantization and the 5 ms delay approximation | Result |
| --- | --- |
| GPU initial-state cases, active controller | 8/8 pass |
| Six additional GPU conditions, active controller | 6/6 pass |
| GPU screening batch | 16/16 worlds pass; CPU/GPU parity checks pass |
| CPU MuJoCo, +5 degrees for 12 seconds | Pass |
| Gazebo, +5 and -5 degrees for 12 seconds | 2/2 pass |
| Gazebo, +10 and -10 degrees | 0/2 pass: falls |
| Python regression checks | 88 pass |
| Portable C++ checks | Pass |
| Nano compile and generated controller constants | Pass; match the simulator |

The 256-trial delayed-sensor search found a lower-score candidate that passed
GPU tests, but it failed the independent Gazebo +/-10-degree checks and was
not applied. An explicit design revision requiring more derivative damping
also failed those checks. Their failed traces remain preserved. These results
expose a transfer gap between the nominal backends; successful GPU runs do not
establish Gazebo or hardware stability.

In Gazebo's successful +5-degree run, the robot rolled about 1.94 m over the
test while staying near upright. That drift is visible in evaluation and is
not suppressed by unavailable wheel feedback. The default menu starts at
5 degrees; the gains are not described as globally optimal or hardware-ready.

The Nano build uses 16,042 program bytes and 639 bytes of static RAM, excluding
stack use. No upload was performed, and motor outputs remain unconfigured.

[Recorded results and source hashes](../experiments/imu-only-correction-20261022.json)
and [the provisional active profile](../experiments/imu-only-current-20261022.yaml).
