# Component specifications for the first simulation

Researched 2026-10-21. Values below describe reference components, not measurements of this individual robot. The working electronics are a classic Arduino Nano, L293D, and MPU-6050 on a GY-521-style board; exact variants remain unverified. We can begin simulation without the existing Arduino sketch or new hardware measurements.

The controlled ROS simulation now implements an [averaged TT motor / L293D surrogate](motor-driver-model.md). That guide distinguishes the implemented effects and diagnostic proxies from these reference specifications and remaining unknowns. The standalone Python experiment remains ideal.

## Arduino Nano (classic)

| Property | Reference specification |
| --- | --- |
| Processor | ATmega328P, 16 MHz |
| Logic | 5 V |
| Memory | 32 KB flash, 2 KB SRAM, 1 KB EEPROM |
| PWM output | 8-bit duty command, 0–255 |
| PWM pins | D3, D5, D6, D9, D10, D11 |
| Standard AVR PWM rates | About 490 Hz; D5/D6 about 980 Hz |
| IMU connection | I2C: A4 = SDA, A5 = SCL |
| Recommended VIN supply | 7–12 V; regulated 5 V is a separate power option |

Sources: [Arduino Nano specifications](https://store.arduino.cc/products/arduino-nano), [processor identification](https://support.arduino.cc/hc/en-us/articles/4401874304274-Select-the-right-processor-for-Arduino-Nano), [PWM pins](https://support.arduino.cc/hc/en-us/articles/9350537961500-Use-PWM-output-with-Arduino), and [AVR PWM timing](https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm/). Rates assume the standard AVR core and no timer reconfiguration.

**Simulation choice:** represent controller timing and PWM resolution rather than emulate the processor. Start with a 200 Hz controller (5 ms period), implemented on the simulation computer. This is a proposed setting, not measured Nano performance. PWM switching frequency and controller update frequency are different parameters. Keep any debug output from blocking the physical control loop when firmware is developed.

## L293D motor driver

| Property | Reference specification |
| --- | --- |
| Capacity | Two bidirectional motors |
| Supplies | 5 V logic; 4.5–36 V motor supply |
| Continuous current | 0.6 A per path; thermal limits apply |
| Absolute peak | 1.2 A, nonrepetitive, maximum 100 microseconds |
| Typical drop at 0.6 A | 1.4 V sourcing + 1.2 V sinking |
| Stop modes | Equal enabled inputs: brake; enable low: coast |

Source: [TI L293/L293D datasheet](https://www.ti.com/lit/ds/symlink/l293.pdf), sections 5, 6, and 8. At 0.6 A, the combined typical drop is 2.6 V, leaving about 3.4 V from this robot's 6 V supply. Loss varies with current. Ratings do not imply current regulation or repetitive peak capability.

**Model choice:** include losses, flag overcurrent, and distinguish braking/coasting. A fixed drop or automatic current clamp would be an approximation, not a full device model. Actual PWM wiring remains unknown.

## MPU-6050 IMU

| Property | Reference specification |
| --- | --- |
| Measurements | Three-axis accelerometer and three-axis gyroscope; 16-bit outputs |
| Accelerometer ranges | ±2, ±4, ±8, ±16 g |
| Gyroscope ranges | ±250, ±500, ±1000, ±2000 degrees/s |
| Communication | I2C, up to 400 kHz |
| Bare-chip VDD | 2.375–3.46 V |

Source: [TDK/InvenSense product specification, revision 3.4](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/mpu-6000-datasheet1.pdf), sections 5 and 6. A breakout board's regulator and pull-ups are separate from the chip; do not infer 5 V logic tolerance from a GY-521 label.

**Proposed settings:** ±4 g and ±500 degrees/s, leaving margin for disturbances. From the 16-bit ranges, the nominal scales are 8192 counts/g and approximately 65.5 counts/(degree/s). These are selected simulation settings, not known settings in the existing firmware.

With digital low-pass filter setting 3, the datasheet register map specifies accelerometer bandwidth/delay of 44 Hz/4.9 ms and gyro bandwidth/delay of 42 Hz/4.8 ms. The filtered internal gyro rate is 1 kHz; SMPLRT_DIV = 4 yields 200 Hz using rate = 1000/(1 + divider). Source: [register map, sections 4.2–4.3](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf). These filter delays are in addition to controller timing; do not represent the sensor as instantaneous. The document is also indexed in [TDK's resource search](https://www.invensense.tdk.com/en-us/search-result?query=6050) if the legacy PDF link redirects.

**Modelling implication:** place the sensor 6 cm above the axle, initially aligned with body x-forward, y-left, z-up. Pitch is rotation about body y. Estimate tilt from accelerometer and gyro readings; reserve simulator ground-truth orientation for evaluation. Account for gravity, acceleration at the sensor offset, filtering, quantization, and later bias/noise. No numerical noise level for the actual module has been established; begin ideal, then document deliberate noise and bias experiments.

## Battery, protoboard, and mechanics

Only the battery's 6 V supply is known. Chemistry, capacity, internal resistance, and discharge behaviour cannot be identified from voltage alone. Start with an ideal 6 V source and a separate ideal regulated 5 V logic rail. Battery sag and regulator behaviour are later sensitivity tests. Because 6 V is below the Nano's recommended VIN range, the existing power wiring should not be inferred from the pack voltage.

The battery, Nano, driver, IMU and protoboard are included in the uniform 420 g
body approximation. No separate masses should be added for these parts. The
protoboard needs no electrical or geometric detail for balancing physics.

The custom wheel material, mass, and mounting clearance cannot be established online. Use the following explicitly chosen placeholders until refined:

| Parameter | Initial assumption | Reason |
| --- | --- | --- |
| Mass per wheel | 0.020 kg | Nonzero provisional value; not a product specification |
| Wheel inertia | Solid cylinder with 0.04 m radius, 0.008 m width | Updated measured geometry; mass still assumed |
| Wheel centre separation | 0.168 m | Owner confirmed center-to-center measurement |
| Axle location | 0.027 m above body bottom | Updated owner measurement; uniform COM is 0.048 m above axle |
| Ground friction coefficient | 0.8 | Adjustable scenario assumption; surface/tread unknown |
| Physics step | 0.001 s | Initial numerical choice; verify stability/convergence |

Successful simulated balancing does not verify the physical robot. Provisional
wheel masses are additional to the 420 g body, giving a model total of 460 g.
Revisit that split if the box weight included the wheels.

## What is ready

Geometry and component references are sufficient to build the passive model, simulated IMU, and initial actuation/controller interfaces. Remaining uncertainties should be editable parameters and clearly labelled in experiment results; they do not block starting the simulation. Motor specifications and low-speed reports are in [hardware.md](hardware.md).
