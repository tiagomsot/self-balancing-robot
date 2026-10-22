# Preliminary mathematical stability check

Historical record for the original 300 g / 25 cm body, with the axle at the
body bottom. Its numerical output is preserved in
[stability-results-initial.json](../analysis/stability-results-initial.json).
Current measurements, results and reproduction are in
[stability-analysis.md](stability-analysis.md); running the current script uses
the updated geometry rather than reproducing this historical parameter set.

Calculated 2026-10-21 for the initial geometry. Numerical output is preserved in [stability-results-initial.json](../analysis/stability-results-initial.json).

## Result and scope

**The assumed ideal mechanical model is locally stabilizable near upright.** This is a mathematical result for the stated model. It is useful evidence that the chosen geometry is feasible, but it does not certify the complete physical robot with its driver losses, deadband, backlash, and sensing errors.

Two different objectives must be kept separate:

- Tilt and angular rate can be stabilized in this ideal model using pitch feedback.
- Holding position also requires controlling the translational states. The calculation demonstrates a full-state controller mathematically, but actual wheel position/speed sensing has not been established.

## Model assumptions

| Quantity | Value |
| --- | --- |
| Body mass m | 0.300 kg |
| Height / front-to-back depth | 0.250 m / 0.050 m |
| Body centre of mass above axle l | 0.125 m; axle at bottom assumed |
| Body pitch inertia about its centre Ib | m(height² + depth²)/12 = 0.001625 kg·m² |
| Wheel radius r | 0.050 m |
| Mass per wheel mw | 0.020 kg; provisional assumption |
| Inertia per wheel Iw | mw r²/2; solid cylinder approximation |
| Gravity g | 9.81 m/s² |

Motion is planar, both wheels remain on the ground, and rolling is without slip. Torque is delivered instantaneously. Motor rotor/reflected inertia, losses, backlash, battery sag, and sensor errors are omitted from the base model. These are material limitations, not claims that those effects are negligible in hardware.

## Coupled wheel/body equations

Let x be forward axle displacement, theta forward body lean from upright, and tau the **sum** of the two wheel motor torques. Equal motor reaction torque on the body is included. The equations are:

```text
M x_ddot + C cos(theta) theta_ddot - C sin(theta) theta_dot² = tau/r
C cos(theta) x_ddot + J theta_ddot - m g l sin(theta) = -tau

M = m + 2 mw + 2 Iw/r² = 0.360 kg
C = m l                 = 0.0375 kg·m
J = Ib + m l²           = 0.0063125 kg·m²
```

These equations were derived for this robot's assumptions using rigid-body mechanics. The modelling method, body reaction torque, linearization, controllability, and eigenvalue analysis are also discussed in [Howard and Bushnell, 2015, sections III–V](https://labs.ece.uw.edu/nsl/papers/brian_acc_2015.pdf). Their robot's numerical parameters are not used here.

A raw comparison of gravitational torque with motor stall torque would miss the acceleration of the wheel axle and the motor reaction torque. It would not establish stability.

## Linear stability calculation

Linearizing at upright gives:

```text
x_ddot     = -15.9253 theta + 189.0332 tau
theta_ddot = 152.8831 theta - 1281.3853 tau
```

Use radians, metres, seconds, and N·m. The four states are [x, x_dot, theta, theta_dot].

Without feedback, the eigenvalues are 0, 0, +12.3646, and -12.3646 per second. The positive eigenvalue confirms an unstable upright equilibrium. Its growing-mode e-folding time is about 0.081 s; this is not the time for a stationary lean to fall over.

The controllability matrix [B, AB, A²B, A³B] has **rank 4 out of 4**. Thus a full-state feedback controller exists for this ideal linear model. This result does not establish that every required state is available from the existing hardware.

For pitch alone, an illustrative torque feedback law is:

```text
tau = 0.219203 theta + 0.0124865 theta_dot
```

It produces:

```text
theta_ddot + 16 theta_dot + 128 theta = 0
```

The pitch eigenvalues are **-8 ± 8i per second**. Their negative real parts prove local asymptotic pitch stability in the smooth ideal model. These gains command total delivered torque, not Arduino PWM; copying them into analogWrite would be incorrect.

The full model with this pitch-only controller retains two zero eigenvalues. It does not asymptotically stabilize position or velocity. With hypothetical full-state feedback, the script independently places all four poles at -2, -3, and -8 ± 8i; that example presumes wheel state information that is currently unconfirmed.

## Motor demand check

Nonlinear mechanical responses were integrated for 3 s from rest using the ideal pitch controller, without actuator saturation. The resulting demands were then compared with a reference motor envelope.

| Starting lean | Peak torque per motor | Peak relative gearbox-output speed | Ideal torque-producing current per motor |
| --- | --- | --- | --- |
| 5 degrees | 0.00956 N·m | 27.5 RPM | 0.183 A |
| 10 degrees | 0.01913 N·m | 55.5 RPM | 0.366 A |

The current estimate uses effective torque constant = reference stall torque / reference stall current. It omits friction, no-load current, electrical transients, and motor variability; it is not a prediction of measured driver current.

For context, the reference motor is approximately 0.0785 N·m stall torque and 200 RPM no-load at 6 V, with a sample stall current of 1.5 A. Source: [Adafruit TT motor 3777](https://www.adafruit.com/product/3777). These are reference values, not identification of the owner's motor.

The script also checks torque and speed together using an assumed linear envelope:

```text
budget = abs(torque_each)/stall_torque + abs(relative_output_speed)/no_load_speed
```

The maximum budget is about 0.178 for the 5-degree case and 0.358 for the 10-degree case, against 1.0 at the reference 6 V motor terminals. This magnitude envelope is conservative in braking quadrants but still excludes losses, transients, and backlash. It does not mean the full 6 V reaches the real motors; the L293D's effect remains to be modelled. See [component notes](components.md).

Both ideal responses bring pitch back toward zero, but they drift: after 3 s, the 5-degree case is moving at about 0.073 m/s and the 10-degree case at 0.148 m/s. Pitch recovery is not position recovery.

## Sampling and uncertainty checks

At a 200 Hz update rate, exact zero-order-hold discretization of the pitch model gives:

| Additional pure measurement delay | Largest discrete pole magnitude | Ideal pitch result |
| --- | --- | --- |
| 0 ms | 0.9610 | Stable |
| 5 ms | 0.9615 | Stable |
| 10 ms | 0.9625 | Stable |
| 20 ms | 0.9669 | Stable |
| 40 ms | 0.9929 | Stable, little margin |
| 60 ms | 1.0154 | Unstable |

Discrete stability requires every pole magnitude below 1. This is a sensitivity experiment with perfect but delayed pitch/rate readings, not an exact MPU filter or estimator model, and not a proven hardware delay allowance.

Keeping the selected pitch gains fixed and changing the assumed wheel mass from 5 to 50 g per wheel also leaves the ideal continuous pitch poles stable in all four evaluated cases. This checks one uncertain parameter; it is not a complete robustness analysis.

The script verifies the analytic state matrices against central finite differences of the nonlinear equations. Pole placement, sampled eigenvalues, and nonlinear integration all completed successfully with NumPy 2.0.2 and SciPy 1.15.3.

## What the calculation establishes

The specified geometry admits local mathematical stabilization, and the example's ideal motor demands do not obviously rule out the reference TT motors. The next refinement is a voltage-input model including driver loss, motor dynamics, friction/deadband, backlash, and sensor estimation. That refinement will determine whether the useful stability region survives more realistic assumptions. Small-angle analysis alone cannot establish recovery from arbitrary pushes or falls.

## Reproduce

From the repository root, with Python and the packages in analysis/requirements.txt available:

```powershell
python analysis/stability_check.py
```

The calculation regenerates analysis/stability-results.json. It does not connect to or change the robot.
