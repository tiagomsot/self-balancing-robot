#pragma once
#include <Arduino.h>
#include "../RobotFirmware/BoardConfig.h"

namespace nanoio {
inline bool pwmPin(uint8_t pin) {
    return pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11;
}

// Direction + enable/PWM interface, appropriate to an L293D-style H-bridge.
// All six Arduino pin numbers must come from the actual wiring.
class NanoMotorDevice {
public:
    NanoMotorDevice() : ready_(false) {}

    bool begin() {
        ready_ = false;
        if (!hardwareConfigured) return false;
        const uint8_t pins[] = {leftEnablePin, leftIn1Pin, leftIn2Pin,
                               rightEnablePin, rightIn1Pin, rightIn2Pin};
        if (!pwmPin(leftEnablePin) || !pwmPin(rightEnablePin)) return false;
        if ((leftMotorPolarity != 1 && leftMotorPolarity != -1) ||
            (rightMotorPolarity != 1 && rightMotorPolarity != -1)) return false;
        for (uint8_t i = 0; i < 6; ++i) {
            if (pins[i] < 2 || pins[i] > 13) return false;
            for (uint8_t j = 0; j < i; ++j)
                if (pins[i] == pins[j]) return false;
        }
        for (uint8_t i = 0; i < 6; ++i) {
            digitalWrite(pins[i], LOW);
            pinMode(pins[i], OUTPUT);
        }
        ready_ = true;
        stop();
        return true;
    }

    void stop() {
        if (!ready_) return;
        analogWrite(leftEnablePin, 0);
        analogWrite(rightEnablePin, 0);
    }

    void writePwm(int left, int right) {
        if (!ready_) return;
        writeWheel(left, leftEnablePin, leftIn1Pin, leftIn2Pin, leftMotorPolarity);
        writeWheel(right, rightEnablePin, rightIn1Pin, rightIn2Pin, rightMotorPolarity);
    }

private:
    static void writeWheel(int value, uint8_t en, uint8_t in1, uint8_t in2, int polarity) {
        value = constrain(value, -255, 255) * polarity;
        // Disable the bridge before changing direction, then apply 8-bit PWM.
        analogWrite(en, 0);
        digitalWrite(in1, value > 0 ? HIGH : LOW);
        digitalWrite(in2, value < 0 ? HIGH : LOW);
        analogWrite(en, abs(value));
    }
    bool ready_;
};
} // namespace nanoio
