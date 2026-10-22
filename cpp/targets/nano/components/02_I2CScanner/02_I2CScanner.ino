#include <Arduino.h>
#include "../../common/Mpu6050Device.h"

uint32_t previousScan = 0;
bool firstScan = true;

void setup() {
    Serial.begin(115200);
    nanoio::beginI2CBus();
    Serial.println(F("I2C scanner: classic Nano A4=SDA, A5=SCL."));
    Serial.println(F("Verify IMU supply/logic voltage and pull-ups first."));
}

void loop() {
    uint32_t now = millis();
    if (!firstScan && now - previousScan < 2000UL) return;
    firstScan = false;
    previousScan = now;
    uint8_t found = 0;
    for (uint8_t address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("Device acknowledged at 0x"));
            if (address < 16) Serial.print('0');
            Serial.println(address, HEX);
            ++found;
        }
    }
    if (!found) Serial.println(F("No I2C device acknowledged."));
    // An address alone does NOT identify the chip; run test 03 next.
}
