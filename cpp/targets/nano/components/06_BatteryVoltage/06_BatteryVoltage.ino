#include <Arduino.h>

// OPTIONAL: your current parts list has no battery measurement circuit.
// A 6 V battery must NOT be connected directly to a classic Nano analog pin.
// Add/verify a divider first, enter its measured values, then enable this test.
const bool dividerConnected = false;
const uint8_t batteryPin = A0;
const double topResistanceOhms = 0;    // Battery+ to A0.
const double bottomResistanceOhms = 0; // A0 to common GND.
const double adcReferenceVolts = 5.0;  // Replace with measured AVcc/AREF.
bool ready = false;
uint32_t previousRead = 0;

void setup() {
    Serial.begin(115200);
    ready = dividerConnected && topResistanceOhms > 0 && bottomResistanceOhms > 0 &&
            adcReferenceVolts > 0;
    if (!ready) {
        Serial.println(F("Disabled: optional divider is not configured. Read wiring-reference.md."));
        return;
    }
    analogReference(DEFAULT);
    Serial.println(F("adc_counts,estimated_battery_volts"));
}

void loop() {
    if (!ready) return;
    uint32_t now = millis();
    if (now - previousRead < 250UL) return;
    previousRead = now;
    uint32_t total = 0;
    for (uint8_t i = 0; i < 16; ++i) total += analogRead(batteryPin);
    double counts = total / 16.0;
    double voltage = counts / 1023.0 * adcReferenceVolts *
                     (topResistanceOhms + bottomResistanceOhms) / bottomResistanceOhms;
    Serial.print(counts, 1); Serial.print(','); Serial.println(voltage, 3);
    if (counts >= 1022) Serial.println(F("ADC near full scale: stop and verify divider/input voltage."));
}
