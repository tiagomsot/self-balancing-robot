#!/usr/bin/env bash
# Compile only. This helper never uploads or chooses a serial port.
set -euo pipefail
shopt -s nullglob
task_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
task_avr_root="${ARDUINO_AVR_ROOT:-/usr/share/arduino/hardware/arduino/avr}"
task_build="$task_project_dir/cpp/targets/nano/build"
task_sketch="$task_project_dir/cpp/targets/nano/RobotFirmware"
task_name="RobotFirmware"
task_components=(01_NanoHeartbeat 02_I2CScanner 03_MPU6050Raw 04_IMUPitch 05_MotorDriver 06_BatteryVoltage)
case "${1:-}" in
  --components)
    for task_component in "${task_components[@]}"; do
      bash "${BASH_SOURCE[0]}" --component "$task_component"
    done
    exit 0 ;;
  --component)
    task_name="${2:-}"
    task_known=false
    for task_component in "${task_components[@]}"; do
      if [ "$task_name" = "$task_component" ]; then task_known=true; fi
    done
    if [ "$task_known" != true ] || [ "$#" -ne 2 ]; then
      printf 'Choose a component: %s\n' "${task_components[*]}"
      exit 2
    fi
    task_sketch="$task_project_dir/cpp/targets/nano/components/$task_name"
    task_build="$task_build/$task_name" ;;
  "") ;;
  *) printf '%s\n' 'Usage: bash build_cpp.sh nano [--component NAME|--components]'; exit 2 ;;
esac
if ! command -v avr-g++ >/dev/null || [ ! -d "$task_avr_root" ]; then
  printf '%s\n' 'Install build tools in Ubuntu: sudo apt install gcc-avr avr-libc arduino-core-avr'
  exit 1
fi
if [ "$task_name" = RobotFirmware ]; then
  python3 "$task_project_dir/cpp/targets/nano/generate_config.py"
fi
mkdir -p "$task_build"
printf '\nCompiling %s for ATmega328P / classic Nano...\n' "$task_name"
task_flags=(-mmcu=atmega328p -DF_CPU=16000000UL -DARDUINO=10819 -DARDUINO_AVR_NANO
  -DARDUINO_ARCH_AVR -Os -ffunction-sections -fdata-sections
  "-I$task_avr_root/cores/arduino" "-I$task_avr_root/variants/eightanaloginputs"
  "-I$task_avr_root/libraries/Wire/src" "-I$task_project_dir/cpp/src" "-I$task_sketch")
task_objects=()
for task_source in "$task_project_dir"/cpp/src/*.cpp "$task_sketch"/*.cpp \
    "$task_avr_root"/cores/arduino/*.cpp "$task_avr_root"/cores/arduino/*.c \
    "$task_avr_root"/cores/arduino/*.S "$task_avr_root"/libraries/Wire/src/*.cpp \
    "$task_avr_root"/libraries/Wire/src/utility/*.c; do
  task_object="$task_build/$(basename "$task_source").o"
  case "$task_source" in
    *.cpp) avr-g++ "${task_flags[@]}" -std=gnu++11 -fno-exceptions -fno-threadsafe-statics -c "$task_source" -o "$task_object" ;;
    *) avr-gcc "${task_flags[@]}" -c "$task_source" -o "$task_object" ;;
  esac
  task_objects+=("$task_object")
done
avr-g++ "${task_flags[@]}" -std=gnu++11 -fno-exceptions -x c++ -include Arduino.h \
  -c "$task_sketch/$task_name.ino" -o "$task_build/sketch.o"
avr-g++ -mmcu=atmega328p -Wl,--gc-sections "${task_objects[@]}" "$task_build/sketch.o" \
  -lm -o "$task_build/$task_name.elf"
avr-objcopy -O ihex -R .eeprom "$task_build/$task_name.elf" "$task_build/$task_name.hex"
avr-size --format=avr --mcu=atmega328p "$task_build/$task_name.elf" | tee "$task_build/memory.txt"
printf 'Compile complete: %s/%s.hex; no upload performed.\n' "$task_build" "$task_name"
