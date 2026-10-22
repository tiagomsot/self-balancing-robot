# Learn the Nano firmware

The hardware simulation now uses the same IMU-only estimator and pitch PID as
this firmware. See [the corrected signal path](../../../docs/imu-only-control.md).
Wheel feedback remains disabled in every adapter, and successful simulation
still does not confirm real wiring or gain suitability on hardware.

Start with the **[complete sketch walkthrough](sketch-walkthrough.md)**. It
explains `setup()`, every stage of `loop()`, the IMU adapter, filter, PID equation,
PWM output, faults, and the C++ syntax used in this project.

Then work through the **[individual component sketches](components/README.md)**
in order. Each is a small real Arduino `.ino` program with a specific observation
to make. The **[wiring reference](wiring-reference.md)** includes connection
tables, a conditional L293D pin drawing and the optional battery divider.

```text
nano/
  RobotFirmware/         Complete balance-only sketch and board configuration
  common/                Arduino IMU and motor I/O reused by the test sketches
  components/            Six small programs to try one at a time
  sketch-walkthrough.md  Explanation of the complete program
  wiring-reference.md    Known connections and wiring still to identify
  generate_config.py     Generate Nano gains from the existing project config
  build.sh               Compile firmware/tests for ATmega328P; never uploads
```

In the VS Code Ubuntu/WSL terminal, from the repository root:

```bash
bash build_cpp.sh component 01_NanoHeartbeat
bash build_cpp.sh components    # Compile all six component sketches
bash build_cpp.sh nano          # Compile the complete balance firmware
```

The toolchain is `gcc-avr`, `avr-libc`, `arduino-core-avr` and `python3-yaml`.
Compiled `.elf`, `.hex` and `memory.txt` files go under `nano/build/`, which Git
ignores. These commands compile; they do not run a simulation or upload to USB.
An uploader still needs the actual Nano variant, bootloader and serial port.

**Current hardware status:** the owner confirmed two-wire motors with no
encoders. The driver and IMU are provisionally L293D and MPU-6050; markings,
motor pins and sensor orientation still need checking. `hardwareConfigured`
stays `false`, with pin numbers `255` as invalid placeholders. The battery
test also stays disabled because no voltage divider has been reported.

Before the IMU-only correction on 2026-10-22, all six sketches and the complete firmware compiled using the
installed Arduino AVR core 1.8.6 / avr-g++ 7.3.0. The complete **unconfigured**
image used 15,338 program bytes and 627 bytes of static RAM. This excludes stack
use and is not a hardware test. Recheck memory after configuring the real pins.
Build sizes and isolated enabled-branch compile checks are recorded in the
[compile results](../../../experiments/nano-component-sketches-20261022.json).

The corrected IMU estimator and PID firmware also compiles: 16,042 program
bytes and 639 bytes of static RAM in its unconfigured build. Its controller
parameters now match the simulation; actual pins and sensor orientation remain
to be verified before a motor test.

The firmware and current simulators both run IMU-only balancing. The routine in
[`cpp/src/my_routine.cpp`](../../src/my_routine.cpp) is retained as reference;
measured-distance motion is disabled because this hardware lacks wheel feedback.
