# Hardware and measured geometry

## Information provided by the owner

- Rectangular chassis: 15 cm high, 10 cm wide along the axle, 3.5 cm thick.
- The width runs along the wheel axle; thickness is front to back.
- Body mass: 420 g. Continue the requested uniform mass distribution with the centre of mass at the geometric centre.
- Two wheels: diameter 8 cm, axial width 0.8 cm; centers 16.8 cm apart (confirmed).
- The body bottom is 2.7 cm below the wheel axle.
- Two yellow TT geared DC motors.
- Owner confirmed no wheel encoders (2026-10-22): two motor power wires per motor.
  Simulated wheel position/speed feedback is not available on the physical robot.
- Arduino Nano controlling the motors through a black motor-driver chip.
- An IMU mounted on the internal protoboard 6 cm above the wheel axis.
- A protoboard runs along the interior of the body and carries the electronics.
- Battery mounted inside the rectangular body; battery supply is 6 V.

## Unverified component identification

The conversation identified the likely motor driver as L293D and the likely IMU as MPU-6050, commonly sold on a GY-521 board. These identifications have not been checked against chip markings, wiring, or Arduino source code.

The TT motor gearbox ratio and exact part number are unknown. The colour and shape alone do not establish a specification.

## Shared geometry in SI units

Updated 2026-10-21. The shared source is
[`robot_geometry.json`](../ros2_ws/src/balancing_robot_sim/config/robot_geometry.json).
Use an axle-centred frame with x forward, y left along the axle, and z upward.
Gazebo xacro, Python dynamics, stability calculation and the MuJoCo GPU loader
read this file. Wheel radius in the ROS controller is set from the same geometry.

| Parameter | Value | Basis |
| --- | --- | --- |
| Body x dimension | 0.035 m | Provided thickness |
| Body y dimension | 0.10 m | Provided width |
| Body z dimension | 0.15 m | Provided height |
| Body mass | 0.420 kg | Updated owner measurement |
| Body bottom relative to axle | -0.027 m | Provided mounting offset |
| Wheel radius | 0.04 m | Half the provided diameter |
| Wheel axial width | 0.008 m | Provided measurement |
| Wheel center separation | 0.168 m | Owner confirmed center to center |
| IMU height above axle | 0.06 m | Provided mounting offset |

The uniform-body centre of mass is 0.15/2 - 0.027 = **0.048 m above the axle**.
The top is 0.123 m above the axle. Upright ground clearance under the box is
0.040 - 0.027 = **0.013 m**. The IMU is 0.012 m above the body centre;
its x/y offset remains assumed zero. A marker on the front face is visual only.

For the uniform body box, inertia about its centre is:

| Component | Value (kg·m²) |
| --- | --- |
| Ixx | 0.0011375 |
| Iyy (pitch axis) | 0.000830375 |
| Izz | 0.000392875 |

Calculated using Ixx = m(y² + z²)/12, Iyy = m(x² + z²)/12, and Izz = m(x² + y²)/12. Products of inertia are zero in the box's principal frame.

Treat the 0.420 kg as the body including onboard components. Do not add separate
battery, board and motor masses on top. Wheel mass remains assumed at 0.020 kg
each, giving a provisional model total of 0.460 kg. Earlier 300 g / 25 cm-body
results and their screenshots describe the initial geometry, not these measurements.

The retained ROS gains passed a new nominal +5-degree balance check and one
drive/stop/turn/return cycle. A separate 256-world MuJoCo Warp batch passed all
response cases and CPU/GPU numerical checks. All 53 unit checks pass, including
the axle/COM geometry and legacy-record handling. Selected measurements and
source hashes are preserved in [geometry-validation.json](geometry-validation.json).
These are simulation checks; the gains have not been reoptimized for this geometry.

## Reference motor specification (assumed proxy)

Source: [Adafruit TT DC Gearbox Motor, product 3777](https://www.adafruit.com/product/3777), consulted 2026-10-21.

| Property | Reference value |
| --- | --- |
| Operating voltage | 3–6 V |
| Gear ratio | 48:1 |
| Listed no-load output speed at 6 V | Approximately 200 RPM (20.94 rad/s) |
| Listed stall torque at 6 V | 0.8 kgf·cm, approximately 0.0785 N·m |
| Vendor's measured stall current at 6 V | 1.5 A on one sample |

These are gearbox-output values, per motor. Do not apply the gear ratio a second time. The vendor also reports a sample reaching 250 RPM at 6 V, illustrating variation; the initial nominal model uses 200 RPM.

Stall torque is a zero-speed endpoint, not a continuous operating torque. A first approximation can use a linear torque–speed relationship between stall torque and no-load speed. Driver voltage drop, current limits, friction, backlash, and unequal motors will need refinement for hardware comparison.

The 6 V battery voltage is not the effective voltage across the motors. Reference L293D losses and their limitations are now documented in [components.md](components.md); driver identification and actual load-dependent losses remain unverified.

## Published low-speed behaviour and builder reports

Online sources are sufficient to begin an approximate simulation. Physical measurements and the existing Arduino sketch are refinements, not prerequisites for building the simulation.

- [Adafruit's measured PWM performance guide](https://learn.adafruit.com/improve-brushed-dc-motor-performance?view=all) shows that the Yellow-TT starting threshold changes with PWM frequency and driver current-decay mode. In one configuration the guide reports a 2.0 V PWM-equivalent starting threshold, versus 0.5 V or less at 25 Hz depending on decay mode. These are test-setup results, not universal thresholds for the owner's motor and driver. The guide's low-frequency advice concerns low-speed operation; it is not a validated balancing-controller setting.
- [Jim Demello's firsthand balancing build notes](https://demej00.wordpress.com/2024/05/26/self-balancing-robot-doesnt-balance-very-well/) report a motor command deadband around 20 on a 255 maximum scale (about 8%) and gearbox slack. The notes describe several motor variants and changing configurations, so that number is not a measured parameter for this robot.
- [The same builder's RobotShop project](https://community.robotshop.com/robots/show/self-balancing-robot-with-yellow-motors-on-smooth-surface) documents balancing with 48:1 yellow motors without encoders, while reporting backlash and drift. Its battery and driver differ from the proposed hardware. These two pages describe one builder, not two independent confirmations.

Consulted 2026-10-21. Model deadband and backlash as adjustable assumptions and test sensitivity rather than presenting a single borrowed value as exact. Quantified backlash for the owner's gearbox has not been established. Preserve the distinction between an ideal model that balances and evidence that the physical robot will balance.

## Measurements to refine later

- Wheel masses, tread material, and ground friction.
- Confirm what was included in the 420 g box measurement before splitting out component masses.
- Exact motor driver, IMU, and motor variant.
- IMU orientation and full mounting offset.
- Battery chemistry and voltage under load.
- Existing Arduino code, pin assignments, control-loop rate, and current balancing performance.
- No encoders, confirmed by the owner. Simulator wheel state is evaluation-only.

The gyro measures body rotation; an accelerometer displaced from the axle also sees acceleration due to rotational motion. Preserve the IMU offset rather than placing it at the centre of mass for convenience.
