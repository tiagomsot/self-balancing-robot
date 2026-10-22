#include <RobotCore.h>
#include "GeneratedConfig.h"
#include "nano_io.h"

// Read ../sketch-walkthrough.md alongside this file. setup() runs once;
// loop() is called repeatedly by Arduino's runtime.
// The controller object keeps its state between calls; do not recreate it
// inside loop(). nanoSettings() supplies the generated gains and limits.
robot::Controller controller(nanoSettings());
uint32_t previousTick=0;
bool hardwareReady=false, armed=false, haveFreshMeasurement=false;

void setup() {
    // 1. Prepare communication and hardware. A successful start never arms.
    Serial.begin(115200);
    hardwareReady=beginHardware();
    disableMotors();
    Serial.println(F("Nano: shared C++ controller, BALANCE ONLY (no encoders)."));
    if(!hardwareReady) Serial.println(F("Hardware unconfigured/unavailable; outputs disabled."));
    Serial.println(F("After wiring/calibration checks: 'a' arms, 'd' disarms; reset clears a fault."));
    previousTick=micros();
}

void loop() {
    // 2. Schedule one control tick. The algorithm only receives elapsed time.
    uint32_t now=micros();
    uint32_t elapsed=now-previousTick; // Unsigned subtraction handles counter wrap.
    if(elapsed<5000UL) return;        // Target 200 Hz. No blocking delay().
    previousTick=now;
    // 3. Handle a human command without pausing the balance loop.
    if(Serial.available()) {
        char key=Serial.read();
        if(key=='d') {armed=false;disableMotors();}
        if(key=='a'&&hardwareReady&&haveFreshMeasurement) armed=true;
    }
    // 4. Read/calibrate/filter the IMU, even while disarmed. During the first
    // 500 successful samples readHardware returns false: hold the IMU still.
    robot::ControlState state={}; // Fresh struct, initially all zero/false.
    double dt=elapsed*1e-6;
    haveFreshMeasurement=hardwareReady&&readHardware(dt,state);
    // 5. Stop outputs on bad data or a tick delayed beyond 50 ms.
    if(!haveFreshMeasurement||elapsed>50000UL) {armed=false;disableMotors();return;}
    if(!armed) {disableMotors();return;}
    // 6. Ask for upright balance, then translate the result to motor outputs.
    robot::MotionTarget hold={0,0}; // Requested forward speed and yaw rate.
    // No encoders: no position/velocity loop or measured-distance routine.
    // Gazebo and MuJoCo use this SAME IMU-only estimator/controller configuration.
    robot::MotorCommand command=controller.update(state,hold,dt,false);
    writeMotors(command);
    if(command.fault!=robot::NO_FAULT) armed=false; // Core fault stays latched.
}
