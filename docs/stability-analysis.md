# Stability check for the measured geometry

Updated 2026-10-21. The calculation reads the shared measured geometry and saves
[stability-results.json](../analysis/stability-results.json). The original
300 g / 25 cm calculation is retained in [stability-analysis-initial.md](stability-analysis-initial.md).

## Model and result

The ideal planar model is locally controllable (rank 4 of 4). This establishes
mathematical stabilizability near upright for the stated assumptions; it is not
a hardware stability guarantee. Ground contact, driver losses, backlash and
estimation errors require separate validation.

| Quantity | Current value |
| --- | --- |
| Body mass / height / depth | 0.420 kg / 0.150 m / 0.035 m |
| Bottom below axle | 0.027 m |
| Uniform COM above axle | 0.048 m |
| Wheel radius / width | 0.040 m / 0.008 m |
| Wheel mass | 0.020 kg each, assumed |
| Wheel center separation | 0.168 m, measured |
| Body pitch inertia at COM | 0.000830375 kg m² |
| Body pitch inertia at axle | 0.001798055 kg m² |

The model retains equal/opposite motor reaction torque, no-slip rolling and
instantaneous total delivered torque tau. State order is [x, velocity, pitch,
pitch rate]; positive pitch and torque point forward.

```text
M x_ddot + C cos(theta) theta_ddot - C sin(theta) theta_dot² = tau/r
C cos(theta) x_ddot + J theta_ddot - m g l sin(theta) = -tau
M = 0.480 kg; C = 0.02016 kg m; J = 0.001798055 kg m²
```

The rigid-body modeling approach and reaction torque are discussed in
[Howard and Bushnell, sections III–V](https://labs.ece.uw.edu/nsl/papers/brian_acc_2015.pdf).
The numerical values here are this project's, rather than that paper's robot.

## Linearization and ideal PD

```text
x_ddot     = -8.731228 theta + 142.587730 tau
theta_ddot = 207.886391 theta - 2154.866582 tau
```

Open-loop poles are 0, 0 and +/-14.4183 per second. Analytic linearization is
checked independently against finite differences of the nonlinear dynamics.

For the standalone ideal-torque PD demo, placing the pitch poles at -8 +/- 8i gives:

```text
tau = 0.1558734049 theta + 0.0074250537 theta_dot
```

These defaults are derived from the current geometry. Pitch-only feedback leaves
position/velocity modes uncontrolled. A separate mathematical full-state example
places all poles at -2, -3 and -8 +/- 8i; wheel sensing is still unconfirmed on
hardware. The ROS/GPU motor-model controller retains its separately searched
gains and uses outer position/velocity feedback.

## Nonlinear ideal response

| Initial lean | Peak torque per motor | Peak shaft speed | Ideal torque current per motor |
| --- | --- | --- | --- |
| 5 deg | 0.006801 N m | 21.67 RPM | 0.130 A |
| 10 deg | 0.013603 N m | 43.61 RPM | 0.260 A |

This ideal calculation omits electrical losses and does not establish actual
current, thermal compliance or a physical stability region. It still drifts:
after three seconds, the +5-degree pitch-only case travels about 0.167 m and
continues at 0.055 m/s.

## Perfect-sensor delay experiment at 200 Hz

| Added pure delay | Largest discrete pole magnitude | Ideal pitch result |
| --- | --- | --- |
| 0 ms | 0.9613 | Stable |
| 5 ms | 0.9627 | Stable |
| 10 ms | 0.9648 | Stable |
| 20 ms | 0.9722 | Stable |
| 40 ms | 1.0006 | Unstable |
| 60 ms | 1.0216 | Unstable |

These are pure delays applied to perfect pitch/rate readings, without saturation.
They are not a hardware delay budget or a complete MPU filter model. Wheel-mass
sensitivity and further numerical outputs are in the JSON.

## Reproduce

```bash
python3 analysis/stability_check.py
```
