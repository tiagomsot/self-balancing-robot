#include <Arduino.h>
#include "../../common/NanoMotorDevice.h"

// This test intentionally bypasses balance control: manual, short PWM pulses.
// Support the chassis with both wheels off the ground. No automatic start.
nanoio::NanoMotorDevice motors;
bool ready = false, pulseActive = false;
const int testPwm = 60;             // 60/255 duty, NOT 60 percent.
const uint32_t pulseLengthMs = 300;
uint32_t pulseStarted = 0;

void setup() {
    Serial.begin(115200);
    ready = motors.begin();
    if (!ready) {
        Serial.println(F("Disabled: fill/verify BoardConfig.h motor pins, then hardwareConfigured."));
        return;
    }
    Serial.println(F("Wheels raised. Commands: l/L left +/-; r/R right +/-; f/b both +/-; s stop."));
    Serial.println(F("Each command lasts at most 300 ms. Reset/disconnect before rewiring."));
}

void loop() {
    if (!ready) return;
    if (pulseActive && millis() - pulseStarted >= pulseLengthMs) {
        motors.stop();
        pulseActive = false;
        Serial.println(F("Stopped."));
    }
    if (!Serial.available()) return;
    char key = Serial.read();
    if (key == '\r' || key == '\n') return;
    int left = 0, right = 0;
    switch (key) {
        case 'l': left = testPwm; break;
        case 'L': left = -testPwm; break;
        case 'r': right = testPwm; break;
        case 'R': right = -testPwm; break;
        case 'f': left = right = testPwm; break;
        case 'b': left = right = -testPwm; break;
        case 's': motors.stop(); pulseActive = false; return;
        default: Serial.println(F("Unknown command; motors stopped."));
                 motors.stop(); pulseActive = false; return;
    }
    motors.writePwm(left, right);
    pulseStarted = millis();
    pulseActive = true;
}
