# Individual component sketches

Use one sketch at a time; uploading a new sketch replaces the previous program
on the Nano. Keep this directory structure: the tests include the shared device
headers and `BoardConfig.h` using relative paths.

Open the `.ino` files in VS Code to read/edit them. Run the build commands below
in the repository's **Ubuntu/WSL terminal**. The repository's helper compiles the
sketch together with Arduino AVR/Wire and the project C++ library. It does not
upload. Once a sketch is uploaded with a tool configured for the actual Nano,
use a serial monitor at **115200 baud** to see its output. Arduino IDE builds
also need the project's `cpp/` Arduino library made available; opening an `.ino`
alone is not the documented build route here.

Read the [wiring reference](../wiring-reference.md) before wiring the IMU or
motor supply. Tests 01–04 do not drive the motors; disconnect motor power for
these tests. Test 05 deliberately drives the wheels, with the chassis supported
and the wheels clear of the ground. Change wiring only with power disconnected.

| Order | Sketch | What you learn | Expected result |
| --- | --- | --- | --- |
| 01 | [Nano heartbeat](01_NanoHeartbeat/01_NanoHeartbeat.ino) | `setup`, `loop`, serial, GPIO and time | Built-in LED toggles every 500 ms; text appears |
| 02 | [I2C scanner](02_I2CScanner/02_I2CScanner.ino) | Addresses and communication | A responding address, often `0x68` or `0x69` for this IMU |
| 03 | [Raw MPU-6050](03_MPU6050Raw/03_MPU6050Raw.ino) | Acceleration versus angular velocity | Six channels change when you rotate the board |
| 04 | [Filtered pitch](04_IMUPitch/04_IMUPitch.ino) | Gyro bias, units, sensor fusion, timing | Raw acceleration angle and filtered pitch follow hand tilt |
| 05 | [Motor driver](05_MotorDriver/05_MotorDriver.ino) | PWM, H-bridge direction and polarity | Only the commanded wheel(s) pulse, then stop |
| 06 | [Battery voltage, optional](06_BatteryVoltage/06_BatteryVoltage.ino) | ADC scaling and a resistor divider | Voltage estimate close to a multimeter measurement |

## 01 — Nano and USB

```bash
bash build_cpp.sh component 01_NanoHeartbeat
```

Only the Nano is needed. `pinMode` makes the built-in LED pin an output.
`millis()` returns elapsed milliseconds. The program compares timestamps and
returns immediately until 500 ms have passed. Then it toggles the LED and
prints one line. The LED is on for half a second and off for half a second.

**Try it yourself:** change `500UL` to `1000UL`. Predict the LED's full on/off
period before running it. Then change it back. This is the scheduling pattern
used by the balance loop, with microseconds instead of milliseconds.

If there is no text, check the baud rate, port and uploaded sketch. A successful
compile alone cannot make the LED change.

## 02 — Find the IMU on the I2C bus

```bash
bash build_cpp.sh component 02_I2CScanner
```

Connect SDA/SCL and verified module power/ground as described in the wiring
reference. This program probes addresses 1–126 and prints those that acknowledge.
It repeats approximately every two seconds. The MPU helper disables the Nano's
internal 5 V pull-ups; the bus needs appropriate external/module pull-ups or
a level shifter. This scanner does not identify the chip from its address.

**Record:** the responding address. If it is `0x69`, update `imuAddress` in
[`BoardConfig.h`](../RobotFirmware/BoardConfig.h) for tests 03/04 and firmware.
Do not set `hardwareConfigured=true` just to run the IMU tests; they do not need
the motor pin mapping.

No response usually means wiring, power, pull-ups or an address/part mismatch.
Do not try to fix it by changing controller gains.

## 03 — Understand the six raw channels

```bash
bash build_cpp.sh component 03_MPU6050Raw
```

The shared device helper checks the identity register, wakes the sensor and
converts its register values to **g** and **degrees/second**. It has no pitch
filter or balancing controller. It prints a CSV-style row at 10 Hz.

With a stationary board, acceleration magnitude should be roughly 1 g and gyro
rates near zero, with noise and bias. Which acceleration channel reads roughly
+1 or -1 depends on orientation. Rotate around one marked axis at a time and
identify which gyro channel changes. A gyro measures turning **rate**, so its
value returns near zero when you stop at a new angle. It does not report angle.
The scale settings and conversion factors follow the
[TDK MPU-6050 specification](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/mpu-6000-datasheet1.pdf).

**Record:** the sensor axes relative to the robot. The adapter currently assumes
sensor x forward, y left and z up. If the board is mounted differently, both
acceleration and gyro must be remapped consistently in `nano_io.cpp` and this
pitch test. Position 6 cm above the axle does not tell us its orientation.

## 04 — Estimate pitch before adding motor motion

```bash
bash build_cpp.sh component 04_IMUPitch
```

Hold the board still for the first 500 successful samples, about 2.5 seconds
at 200 Hz. This averages the y-gyro bias. Calibration does not define your tilted
board as upright; the acceleration angle still sets the initial pitch.

The program then updates the shared `robot::PitchEstimator` at 200 Hz and prints
at 20 Hz. Columns are acceleration-only pitch, bias-corrected pitch rate and
filtered pitch. Tilt slowly, hold, then return upright. Both angle estimates
should settle near the new angle. Observe how translation/shaking affects the
acceleration estimate; it is not a perfect gravity measurement during motion.

**Try it yourself:** change the construction to
`robot::PitchEstimator pitchFilter(0.10);`, then `pitchFilter(0.50);`. A smaller
time constant corrects towards the acceleration angle faster; a larger one
trusts gyro integration for longer. Record what you see, then restore the default
`0.25` seconds. You are changing filter tuning, not PD tuning.

The test does not integrate wheel position or absolute heading. Read failures
or rejected filter inputs stop the test's updates; reset after investigating.

## 05 — Driver and both TT motors

```bash
bash build_cpp.sh component 05_MotorDriver
```

First confirm the driver marking and trace its wires into
[`BoardConfig.h`](../RobotFirmware/BoardConfig.h). Fill all six Arduino pin
numbers, verify two enable pins support PWM, and set `hardwareConfigured=true`
only after checking that mapping. This is a motor-only test; it does not require
the IMU to respond. The defaults compile but leave all motor pins untouched.

Support the robot with both wheels raised. The program waits for a serial
command; it never begins a pulse automatically:

| Character | Command |
| --- | --- |
| `l` / `L` | Left wheel positive / negative |
| `r` / `R` | Right wheel positive / negative |
| `f` / `b` | Both wheels positive / negative |
| `s` | Disable both bridges immediately when the command is processed |

Each pulse uses PWM magnitude **60 out of 255** and stops after at most 300 ms
unless another pulse command restarts the timer. Uppercase letters are deliberate.
Positive means the wheel rolls the robot forward; opposite-facing motor mounts
may require different polarity signs. Change `leftMotorPolarity` or
`rightMotorPolarity` between `1` and `-1` as needed, rebuild and retest.

**Record:** which motor responds, its direction, whether it starts at PWM 60,
and any left/right difference. A wheel that does not start at this small command
is not automatically wired incorrectly: low-speed friction, gearbox slack and
driver voltage loss matter. Do not hold a stalled motor on. This test measures
neither torque nor motor current and does not establish a safe continuous duty.
For an actual L293D, enable-low disables the output pair; see the
[TI driver truth table](https://www.ti.com/lit/ds/symlink/l293.pdf).

## 06 — Optional battery measurement

```bash
bash build_cpp.sh component 06_BatteryVoltage
```

This is an extension: the current robot has no reported battery sensing circuit.
By default, the sketch prints that it is disabled. Build the resistor divider in
the wiring reference before connecting a battery measurement to an analog pin.
Enter measured resistor values and ADC reference voltage, then enable
`dividerConnected`. Do not wire a 6 V battery directly to A0.

The code averages 16 ADC readings and reconstructs battery voltage from the
divider ratio. Compare it with a multimeter. The reading is not used by the full
balance firmware yet; no automatic battery cutoff is implemented.

## From component tests to the complete robot

The protoboard, wheels and chassis are passive parts; they do not each have an
Arduino sketch. The battery needs code only if you add a measurement circuit.
The driver and motors are tested together because the Nano cannot power a motor
through a GPIO pin.

After tests 01–05, read the [complete walkthrough](../sketch-walkthrough.md),
verify the pitch and motor signs together, and compile `bash build_cpp.sh nano`.
The main firmware still starts disarmed. Simulation gains are a starting point
for investigation, not measured hardware gains. No component sketch performs
the measured-distance shuttle; your physical motors have no encoders.

Keep a small record for your portfolio: sketch/version, wiring, expected result,
actual reading, what you changed and what happened. Explaining a bias correction
or a sign error you diagnosed is useful evidence of your own engineering work.
