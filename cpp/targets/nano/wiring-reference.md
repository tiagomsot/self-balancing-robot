# Wiring reference for the component sketches

This is a connection reference, **not a reconstruction of your existing wires**.
The Nano is reported, but L293D and MPU-6050 identification is still provisional.
Check markings before following their pin-specific sections. See the measured
[hardware list](../../../docs/hardware.md).

## Nano connections and power

These tests target the classic 5 V, 16 MHz, ATmega328P Nano. The official
[Nano pinout](https://docs.arduino.cc/resources/pinouts/A000005-full-pinout.pdf)
identifies A4 as SDA, A5 as SCL, and D3/D5/D6/D9/D10/D11 as PWM-capable.
This adapter reserves D0/D1 for serial and accepts six distinct motor pins
from D2–D13, with enable pins chosen from those PWM pins.

```text
Nano A4 (SDA) ---- suitable I2C voltage interface ---- IMU SDA
Nano A5 (SCL) ---- suitable I2C voltage interface ---- IMU SCL
Nano GND ------------------------------------------ IMU GND
Verified module supply ---------------------------- IMU VCC

Nano direction/PWM signals ------------------------ Motor driver inputs
Nano GND -------- common ground ------------------- Driver GND / battery -
Regulated logic supply ---------------------------- Driver logic supply
6 V motor battery + ------------------------------- Driver motor supply
Driver output pair -------------------------------- Left TT motor, two wires
Driver output pair -------------------------------- Right TT motor, two wires
```

For initial Nano/IMU tests, USB can power the Nano and motor power stays
disconnected. A 6 V motor battery is not a substitute for a regulated 5 V logic
supply; the Nano's listed VIN recommendation is 7–12 V. The portable robot's
logic power arrangement still needs confirming. Do not power motors from GPIO.
All signal grounds must share a reference. Check how your protoboard rails are
actually connected; some are split.

## IMU: verify the breakout, not just the chip name

The bare MPU-6050 uses a 2.375–3.46 V supply; a breakout may add a regulator and
pull-ups. A module accepting 5 V on VCC does not establish 5 V tolerance on SDA
and SCL. Verify its schematic/components; use suitable level shifting where
required. The code disables the Nano's internal 5 V I2C pull-ups, so appropriate
external/module pull-ups must exist. These details come from the
[TDK sensor specification](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/mpu-6000-datasheet1.pdf),
not from a confirmed schematic of your board.

| Module signal | Connection / status |
| --- | --- |
| VCC, GND | Verified module supply and common ground |
| SDA, SCL | Nano A4/A5 through the verified voltage interface |
| AD0 | Defined low/high sets address `0x68`/`0x69`; some modules already provide a pull resistor |
| INT | Not used by these polling sketches |
| XDA, XCL, if present | Auxiliary bus, not used here |

The IMU's position is 6 cm above the axle. Its orientation is still unknown.
The software expects x forward, y left and z up. Determine the board axes using
test 03, then map both gyro and acceleration consistently in `nano_io.cpp`.

## Driver: only if it is an L293D in a 16-pin DIP package

This is a **top view**, with the package notch at the top. Match the orientation
and exact marking before counting. These are IC pin numbers, not Nano numbers.

```text
                         notch
                     +----U----+
 left enable/PWM   1 |         | 16  logic supply VCC1
 left IN1          2 |         | 15  right IN2
 left motor wire   3 |         | 14  right motor wire
 ground            4 | L293D   | 13  ground
 ground            5 |         | 12  ground
 left motor wire   6 |         | 11  right motor wire
 left IN2          7 |         | 10  right IN1
 motor supply VCC2 8 |         |  9  right enable/PWM
                     +---------+
```

| `BoardConfig.h` constant | Conditional L293D DIP connection | Your Nano pin |
| --- | --- | --- |
| `leftEnablePin` | IC pin 1, 1,2EN | TODO: actual PWM-capable D pin |
| `leftIn1Pin` | IC pin 2, 1A | TODO |
| `leftIn2Pin` | IC pin 7, 2A | TODO |
| `rightEnablePin` | IC pin 9, 3,4EN | TODO: actual PWM-capable D pin |
| `rightIn1Pin` | IC pin 10, 3A | TODO |
| `rightIn2Pin` | IC pin 15, 4A | TODO |

Connect left motor across IC pins 3/6 and right motor across 11/14. Use 5 V
logic at VCC1, the motor supply at VCC2, and connect all ground pins. The left/right
assignment above is our convention. EN low disables its output pair. Provide
enable pull-downs so the bridges stay disabled while the Nano resets. TI shows
local 0.1 microfarad supply bypass capacitors; follow the actual part's circuit
and motor-supply decoupling requirements. Pinout and application reference:
[TI L293/L293D datasheet](https://www.ti.com/lit/ds/symlink/l293.pdf).

If your chip/module is L298, TB6612 or another driver, this physical pin drawing
does not apply. Identify it before adapting the interface. Existing current/drop
assumptions are documented in [the motor model](../../../docs/motor-driver-model.md).

## Optional battery divider — an added circuit

The robot has no reported battery voltage sensing. Test 06 is disabled until
you deliberately add and configure this circuit:

```text
battery + ---- Rtop ----+---- Rbottom ---- common GND
                       |
                     Nano A0
```

```text
V_A0      = V_battery * Rbottom / (Rtop + Rbottom)
V_battery = ADC_count / 1023 * V_reference * (Rtop + Rbottom) / Rbottom
```

For illustration, **22 kohm top / 10 kohm bottom** gives 1.875 V at A0 for a
6 V battery. These are an example design, not components known to be installed.
Check the battery's maximum charged voltage, component tolerances and input
limits before choosing parts. Both circuits need common ground, and A0 must
stay within the Nano's permitted input range, including when power sequencing.
Never connect the 6 V pack directly to A0. Enter measured resistance and actual
ADC reference values in test 06 and compare with a multimeter. The ADC behaviour
is described in Arduino's [analogRead reference](https://github.com/arduino/reference-en/blob/master/Language/Functions/Analog%20IO/analogRead.adoc).

The full balancing firmware does not currently consume this voltage measurement.
Adding this test does not add a battery cutoff or change controller gains.
