# Motor and driver approximation

Implemented 2026-10-21. The default ROS demo now uses an averaged electrical
model between the controller's requests and Gazebo's applied joint torques.
It is a reference-component surrogate, not identification of the owner's robot.
The exact TT motor variant, L293D marking, and PWM wiring remain unverified.

See [the original validation results](motor-driver-validation.md) for nominal
recovery, the ideal comparison, and the failing deadband/lag stress case. The
[later tuning report](controller-tuning.md) records the current preset and limits.

## Run and compare

```bash
bash run_gazebo.sh
bash run_gazebo.sh actuator_mode:=ideal
```

Stop the current launch before starting another. The first command uses the
motor/driver model; the second bypasses it for comparison. Presets and
assumptions are editable in `ros2_ws/src/balancing_robot_sim/config/balance.yaml`.
The original motor-model preset used a 0.18 outer velocity gain instead of 0.10
to restore damping after adding shaft drag. That full preset is now preserved
in `config/balance-baseline.yaml`; the earlier 0.10 version is preserved in
`config/sensitivity-original-gains.yaml`. The current default uses the later
automatically searched gains and a 0.040 N m request cap; hardware model values
are unchanged. See [the tuning report](controller-tuning.md).

The signal path is:

```text
IMU + ideal wheel feedback -> balance_controller -> bounded torque requests
    -> nominal static torque-to-PWM calibration -> 8-bit PWM magnitude
    -> shared battery + L293D loss + motor back EMF/current + shaft drag
    -> delivered joint torque -> Gazebo rigid-body physics
```

The controller's +/-0.040 N m cap is a software command cap. It does not
guarantees that the motors deliver the request. Physical torque can differ due
to speed, voltage loss, braking, quantization, and saturation. The model uses
gearbox-output shaft speed **relative to the chassis**, as required for motor
back EMF. The 48:1 ratio is not applied again to gearbox-output specifications.

## Reference facts and derived parameters

| Quantity | Value / basis |
| --- | --- |
| Supply | Owner's 6 V battery; chemistry and internal resistance unknown |
| Motor endpoints | Reference 200 RPM, 0.8 kgf cm stall torque, 1.5 A stall current, 0.15 A no-load current at 6 V |
| Armature resistance proxy | R = 6 / 1.5 = 4 ohm |
| Effective shaft torque constant | Kt = 0.0784532 / 1.5 = 0.05230 N m/A |
| Back EMF constant | Ke = (6 - R*0.15) / (200*2*pi/60) = 0.25783 V s/rad |
| Shaft drag | b = Kt*0.15 / (200*2*pi/60) = 0.0003746 N m s/rad |
| Driver typical full-bridge drop | 2.6 V at 0.6 A, combining source and sink |
| Driver current ratings | 0.6 A continuous; 1.2 A nonrepetitive for at most 100 microseconds |
| PWM | Signed direction and 0..255 magnitude; assumed 490 Hz |

Motor reference: [Adafruit TT 3777](https://www.adafruit.com/product/3777).
Speed/current/torque values mix listed endpoints and a vendor sample; they
describe a plausible proxy, not a fitted individual motor.
Driver reference: [TI L293/L293D datasheet](https://www.ti.com/lit/ds/symlink/l293.pdf).
Its electrical-loss anchor is specified at 0.6 A. There is no complete sourced
current-versus-drop curve for this particular chip.

The motor-alone surrogate matches the stated stall and no-load endpoints:

```text
I = (V - Ke*shaft_speed) / R
shaft_torque = Kt*I - b*shaft_speed
```

Kt is an effective geared-shaft constant and differs from Ke. This empirical
surrogate assumes symmetric reverse operation. It is not a complete gearbox
efficiency or regenerative-power model. Braking energy is not credited to the battery.

## Driver loss and PWM assumptions

Full-drive driver drop is approximated as `V0 + Rd*abs(I)`, with **assumed**
V0 = 1.8 V and Rd = (2.6 - 1.8)/0.6 = 1.333 ohm. Only the 2.6 V point at
0.6 A is sourced; this interpolation/extrapolation is an explicit assumption.
At 6 V and zero speed, it predicts about 0.788 A and 0.0412 N m per motor at
full drive, compared with the motor's raw 6 V stall torque of 0.0785 N m.
This also exceeds the driver's continuous-current rating; the model flags it.

The open-loop torque-to-duty calibration uses that nominal zero-speed torque.
It does not compensate for instantaneous shaft speed or battery sag and is
not a measured torque servo. `command_full_pwm_torque_nm` stays fixed at
0.04118793 N m when actual driver, motor, or battery assumptions change; a
sensitivity run cannot silently recalibrate the controller's feedforward.
PWM magnitude is quantized to 255 levels.

For assumed enable-PWM with coast during OFF time, the model uses a
quasi-static ON current and assumes OFF current decays to zero:

```text
I_on = [sign(duty)*(bus_voltage - V0) - Ke*shaft_speed] / (R + Rd)
I_average = abs(duty)*I_on
I_rms_estimate = sqrt(abs(duty))*abs(I_on)
shaft_torque = Kt*I_average - b*shaft_speed
```

The estimated ON time is `abs(duty)/pwm_frequency`. This approximation does
not resolve winding inductance, switching ripple, diode recirculation, or
current growth inside an individual pulse. Therefore I_on is a quasi-static
estimate, not an established actual pulse peak. Frequency changes that ON-time
diagnostic; it does not change the averaged current formula.

`pwm_off_mode: brake` selects an alternative averaged continuous-current model:

```text
I_average = [duty*(bus_voltage - V0) - Ke*shaft_speed] / (R + Rd)
```

This uses resistive braking during OFF time; it does not establish the exact
L293D switching behaviour. The actual input/enable PWM wiring must be checked
to choose and refine either model.

## Ratings are checked, not invented protections

The model reports average current, an RMS estimate, the ON-current estimate,
PWM saturation, supply voltage, and driver conduction-loss power. It never
silently clamps current to 0.6 A. The L293D is not a current regulator, and its
1.2 A nonrepetitive rating is not a repetitive PWM allowance.

An average below 0.6 A does not establish compliance when estimated ON current
is higher. Winding/pulse information is needed. Such runs are labelled
`pulse_rating_uncertain`; successful balance is reported separately from
electrical-rating diagnostics. All reports keep
`hardware_rating_compliance_established: false`.

The temperature diagnostic is **steady-state only**:
`ambient + reference_RthetaJA*driver_conduction_loss`, using 36.4 K/W from
TI's JEDEC reference. It omits transient thermal capacitance, quiescent and
switching losses, and the actual protoboard's cooling. It is not a predicted
junction temperature trace or a simulated thermal shutdown. The model flags
a supply below 5 V under the assumed regulated 5 V logic rail; behaviour outside
the driver's operating range is not validated.

## Unknown values and sensitivity cases

- Battery resistance defaults to zero. A configured nonzero value couples
  both motors through source sag; the bus voltage is solved consistently.
- Extra command deadband defaults to zero because this motor's threshold is
  unknown. The no-load reference current is represented by shaft drag.
- Optional mean-current lag defaults to zero. A configured time constant is
  a deliberate response-sensitivity assumption, not measured winding inductance.
- Backlash, gearbox compliance/reflected rotor inertia, left/right mismatch,
  battery discharge, and a detailed PWM circuit are not identified or modelled.
- IMU orientation and wheel sensing remain ideal. Physical wheel encoders have
  not been confirmed. This actuator change does not add an orientation estimator.

The hardware `config/sensitivity-*.yaml` files are **test assumptions**, not new
hardware facts: lower battery voltage/source resistance, driver maximum-drop
anchor, and an 8% deadband with a 2 ms mean-current lag. The deadband/lag case
is an exploratory stress setting, not a claimed specification.

With optional lag enabled, electrical diagnostics remain quasi-static proxies;
they are bounded below by the persistent mean current, not resolved waveforms.
The loss and temperature proxies are particularly uncertain during braking,
because the surrogate does not identify gearbox backdrive efficiency.

## Reproduce checks

After building and sourcing the ROS workspace, from the repository root:

```bash
python3 analysis/check_gazebo_balance.py --output outputs/motor-5deg
python3 analysis/check_gazebo_balance.py --mode ideal --output outputs/ideal-5deg
python3 analysis/check_gazebo_balance.py --tilt -5 --output outputs/motor-minus5deg
python3 analysis/check_gazebo_balance.py --tilt 10 --output outputs/motor-10deg
python3 analysis/check_gazebo_balance.py --params-file ros2_ws/src/balancing_robot_sim/config/sensitivity-lower-battery.yaml --output outputs/lower-battery
```

The check merges a sensitivity file over the defaults, saves effective
`parameters.yaml`, robot/world snapshots, source hashes, controller and motor
CSV, IMU CSV, and a JSON report. A failed balancing response is preserved too.
The report separates bounded controller commands from actual delivered torque.
The analysis window is limited to the requested simulated duration, even if
the recorder catches up later.

Endpoint, reversal, braking/coasting, battery sag, PWM resolution, and
no-invented-current-clamp checks run without ROS:

```bash
python -m unittest discover -s tests -v
```
