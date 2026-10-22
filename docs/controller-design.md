# Recommended controller approach

Recommendation recorded 2026-10-21. This is a proposed implementation path, not a hardware-validated controller.

Current ROS implementation: pitch PD and position feedback use ideal simulated
orientation and wheel states, with an [averaged motor/driver surrogate](motor-driver-model.md).
The complementary filter below is still planned, not implemented in that demo.

## First version: model-informed PD with a complementary filter

Use the mathematical model to design and check a PD balancing controller. The illustrative law in [stability-analysis.md](stability-analysis.md) is already PD: proportional feedback on tilt plus derivative feedback on angular rate. It is equivalent to PID with integral gain zero, under the documented signs and fixed reference.

The proposed signal path is:

```text
MPU-6050 accelerometer + gyro
    -> complementary filter estimates body pitch
    -> pitch error + gyro pitch rate feed the PD controller
    -> actuator model / command conversion and saturation
    -> driver and two motors
```

Use the gyro's calibrated pitch-rate measurement for the D contribution. Avoid obtaining angular rate by numerically differentiating noisy tilt estimates. Target a fixed 200 Hz update initially and verify actual execution time before deployment.

A complementary filter blends gyro integration with the slower accelerometer orientation reference. [MathWorks' IMU complementary-filter example](https://www.mathworks.com/help/nav/ug/estimate-orientation-with-comp-filter-and-imu-data.html) describes this division. The accelerometer senses translation as well as gravity, including acceleration from this robot's sensor offset; account for that in the simulated measurements and estimator evaluation. Treat filter tuning and resulting delay as part of the closed-loop design.

## How the alternatives fit

| Method | Role | Recommendation for this project |
| --- | --- | --- |
| PD / PID | Feedback controller calculating actuation from error | Start with PD; add a modest I contribution only if a demonstrated bias requires it |
| Kalman filter | State estimator using a model, measurements, and uncertainty assumptions | Compare with the complementary filter later, for example to estimate gyro bias |
| LQR | Model-based state-feedback controller | Useful next comparison; a full position-and-balance version needs reliable wheel position/velocity measurements or estimates |
| MPC | Predictive controller with explicit constraints | Useful simulation extension for voltage/torque constraints; assess computing and memory budgets before hardware implementation |

A Kalman filter can be used with PD, PID, LQR, or MPC. It does not itself choose balancing motor commands. See [state estimation in a control architecture](https://www.mathworks.com/help/mpc/ug/controller-state-estimation.html).

LQR gains can be computed offline; execution is then a weighted sum of the estimated states. The missing wheel feedback is a practical issue for a full-state design, rather than LQR computation alone. See the [inverted-pendulum LQR tutorial](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlStateSpace).

Conventional linear MPC solves an optimization at each update. Explicit MPC moves that work offline but trades against stored regions and memory. Neither is categorically impossible on a small processor; establishing deterministic timing and memory use would be a separate engineering task. For this Nano project, begin MPC comparisons on the simulation computer. See [implicit versus explicit MPC](https://www.mathworks.com/help/mpc/ug/explicit-mpc.html).

## Translate the model into implementable commands

The calculated gains command the SUM of both motor torques in N·m, using radians and radians/s. They are not PWM gains. Equal straight-line drive initially allocates half of that total to each motor, with electrical polarity adjusted for mirrored mounting.

Before using these gains on hardware, extend the plant with driver voltage drop, motor speed-dependent torque and response, deadband, and command saturation. Design or retune in that actuation domain. A constant torque-to-PWM multiplier is only an approximation; it does not establish real torque control with this hardware.

Keep I disabled initially. If integral action is added, implement anti-windup for actuator saturation; [MathWorks' anti-windup example](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html) explains why. Integral action in the pitch loop does not provide an absolute position measurement or guarantee zero drift.

## Comparison plan

1. Reproduce the ideal model's PD baseline.
2. Add realistic actuation, simulated IMU measurements, and the complementary filter; check stability again.
3. Compare PD and PID only when a specific disturbance or bias motivates integral action.
4. Compare complementary and Kalman estimators using identical recorded/simulated sensor data.
5. Evaluate LQR when the required state information is available; keep encoder-free and encoder-equipped cases explicit.
6. Explore constrained MPC in simulation if actuator limits dominate performance.

Use the same initial lean, disturbance, noise, motor assumptions, and delay for comparisons. Report tilt recovery, drift, saturation, control effort, estimator error, and execution time.
