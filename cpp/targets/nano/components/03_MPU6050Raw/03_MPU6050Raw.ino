#include <Arduino.h>
#include "../../common/Mpu6050Device.h"
#include "../../RobotFirmware/BoardConfig.h"

nanoio::Mpu6050Device imu;
bool ready = false;
uint32_t previousRead = 0;

void setup() {
    Serial.begin(115200);
    ready = imu.begin(imuAddress);
    if (!ready) {
        Serial.println(F("MPU-6050 unavailable. Check test 02/address/part/wiring; reset to retry."));
        return;
    }
    Serial.print(F("WHO_AM_I = 0x"));
    Serial.println(imu.identity(), HEX);
    Serial.println(F("ax_g,ay_g,az_g,gx_deg_s,gy_deg_s,gz_deg_s"));
}

void loop() {
    if (!ready) return;
    uint32_t now = millis();
    if (now - previousRead < 100UL) return; // 10 Hz, readable serial output.
    previousRead = now;
    nanoio::MpuSample sample;
    if (!imu.read(sample)) {
        Serial.println(F("IMU read failed."));
        return;
    }
    Serial.print(sample.ax, 3); Serial.print(',');
    Serial.print(sample.ay, 3); Serial.print(',');
    Serial.print(sample.az, 3); Serial.print(',');
    Serial.print(sample.gx, 3); Serial.print(',');
    Serial.print(sample.gy, 3); Serial.print(',');
    Serial.println(sample.gz, 3);
}
