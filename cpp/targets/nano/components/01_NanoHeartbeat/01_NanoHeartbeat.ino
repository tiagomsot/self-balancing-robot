#include <Arduino.h>

// First test: only the Nano and USB. Disconnect the motor supply.
uint32_t previousBlink = 0;
bool ledOn = false;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.println(F("Nano heartbeat: built-in LED changes every 500 ms."));
}

void loop() {
    uint32_t now = millis();
    if (now - previousBlink < 500UL) return;
    previousBlink = now;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
    Serial.println(ledOn ? F("LED on") : F("LED off"));
    // No delay(): other work could run between these updates.
}
