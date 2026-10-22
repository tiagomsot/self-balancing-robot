#include <Arduino.h>
#include <math.h>
#include <RobotCore.h>
#include "../../common/Mpu6050Device.h"
#include "../../RobotFirmware/BoardConfig.h"

// Uses the SAME complementary estimator as the complete firmware.
nanoio::Mpu6050Device imu;
robot::PitchEstimator pitchFilter;
bool ready = false;
uint16_t calibrationSamples = 0;
double gyroBias = 0;
uint32_t previousTick = 0, previousPrint = 0;

void setup() {
    Serial.begin(115200);
    ready = imu.begin(imuAddress);
    Serial.println(ready ? F("Hold IMU stationary: collecting 500 gyro samples.")
                         : F("IMU unavailable: run tests 02 and 03; reset to retry."));
    previousTick = micros();
}

void loop() {
    if (!ready) return;
    uint32_t now = micros();
    uint32_t elapsed = now - previousTick;
    if (elapsed < 5000UL) return;
    previousTick = now;
    nanoio::MpuSample sample;
    if (!imu.read(sample)) {
        ready = false;
        Serial.println(F("IMU read failed; reset to retry."));
        return;
    }
    // Sensor axes currently assumed x forward, y left, z up. No auto-remap.
    if (calibrationSamples < 500) {
        gyroBias += sample.gy;
        ++calibrationSamples;
        if (calibrationSamples == 500) {
            gyroBias /= 500;
            Serial.println(F("accel_pitch_deg,gyro_y_deg_s,filtered_pitch_deg"));
        }
        return;
    }
    double rate = (sample.gy - gyroBias) * robot::pi / 180;
    double pitch;
    if (!pitchFilter.update(sample.ax, sample.az, rate, elapsed * 1e-6, &pitch)) {
        ready = false;
        Serial.println(F("Estimator rejected input/timing; reset to retry."));
        return;
    }
    if (now - previousPrint < 50000UL) return; // Filter 200 Hz, print 20 Hz.
    previousPrint = now;
    Serial.print(atan2(-sample.ax, sample.az) * 180 / robot::pi, 2);
    Serial.print(','); Serial.print(sample.gy - gyroBias, 2);
    Serial.print(','); Serial.println(pitch * 180 / robot::pi, 2);
}
