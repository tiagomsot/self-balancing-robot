#pragma once
#include <RobotCore.h>
bool beginHardware();
bool readHardware(double dt, robot::ControlState& state);
void writeMotors(const robot::MotorCommand& command);
void disableMotors();
