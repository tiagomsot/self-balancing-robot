#pragma once
#include <stdint.h>
// Fill these values from the ACTUAL wiring. No motor pins are touched while false.
const bool hardwareConfigured = false;
const uint8_t leftEnablePin = 255, leftIn1Pin = 255, leftIn2Pin = 255;
const uint8_t rightEnablePin = 255, rightIn1Pin = 255, rightIn2Pin = 255;
const int leftMotorPolarity = 1, rightMotorPolarity = 1;
// MPU-6050 normally uses 0x68 (AD0 low) or 0x69 (AD0 high). Run the scanner.
const uint8_t imuAddress = 0x68;
// Required axes: x forward, y left, z up. The adapter currently assumes this
// mounting; edit the sample-to-robot mapping in nano_io.cpp if it differs.
