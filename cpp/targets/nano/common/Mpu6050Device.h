#pragma once
#include <Arduino.h>
#include <Wire.h>

// Arduino-specific I/O, shared by the diagnostic sketches and RobotFirmware.
// The portable estimator/controller stays in cpp/src/.
namespace nanoio {
inline void beginI2CBus() {
    Wire.begin();
    // Classic Nano internal pull-ups go to 5 V. Use verified external/module
    // pull-ups at the IMU's logic voltage, or a suitable I2C level shifter.
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    Wire.setClock(400000UL);
    Wire.setWireTimeout(2000, true);
}

struct MpuSample {
    double ax, ay, az; // Acceleration in g, in the SENSOR's axes.
    double gx, gy, gz; // Angular velocity in degrees/second.
};

class Mpu6050Device {
public:
    Mpu6050Device() : address_(0x68), identity_(0), ready_(false) {}

    bool begin(uint8_t address) {
        ready_ = false;
        address_ = address;
        if (address != 0x68 && address != 0x69) return false;
        beginI2CBus();
        if (!selectRegister(0x75) || Wire.requestFrom(address_, uint8_t(1)) != 1)
            return false;
        identity_ = static_cast<uint8_t>(Wire.read());
        if ((identity_ & 0x7e) != 0x68) return false;
        // Wake; +/-250 deg/s gyro; +/-2 g accelerometer; DLPF setting 3.
        // The main sketch polls at 200 Hz; no new sample-rate divider is set.
        ready_ = writeRegister(0x6b, 0) && writeRegister(0x1b, 0) &&
                 writeRegister(0x1c, 0) && writeRegister(0x1a, 3);
        return ready_;
    }

    uint8_t identity() const { return identity_; }

    bool read(MpuSample& sample) {
        if (!ready_ || !selectRegister(0x3b) ||
            Wire.requestFrom(address_, uint8_t(14)) != 14) return false;
        int16_t raw[7];
        for (uint8_t i = 0; i < 7; ++i) {
            uint16_t high = static_cast<uint8_t>(Wire.read());
            raw[i] = static_cast<int16_t>((high << 8) |
                         static_cast<uint8_t>(Wire.read()));
        }
        sample.ax = raw[0] / 16384.0;
        sample.ay = raw[1] / 16384.0;
        sample.az = raw[2] / 16384.0;
        // raw[3] is temperature; the following three words are gyro readings.
        sample.gx = raw[4] / 131.0;
        sample.gy = raw[5] / 131.0;
        sample.gz = raw[6] / 131.0;
        return true;
    }

private:
    bool selectRegister(uint8_t reg) {
        Wire.beginTransmission(address_);
        Wire.write(reg);
        return Wire.endTransmission(false) == 0;
    }
    bool writeRegister(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(address_);
        Wire.write(reg);
        Wire.write(value);
        return Wire.endTransmission() == 0;
    }
    uint8_t address_, identity_;
    bool ready_;
};
} // namespace nanoio
