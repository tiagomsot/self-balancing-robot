#include "RobotCore.h"
namespace robot {
const char* faultName(Fault f) {
    switch(f) {
    case NO_FAULT:return "No fault"; case INVALID_INPUT:return "Invalid sensor or command";
    case FALLEN:return "Body exceeded the fall angle"; case STALE_SENSOR:return "Sensor data stale";
    case MISSING_WHEEL_FEEDBACK:return "This mode requires wheel feedback";
    case INVALID_TIME:return "Invalid or reversed time"; case PHASE_TIMEOUT:return "Motion phase timed out";
    case QUEUE_FULL:return "Routine queue full"; case INVALID_CONFIGURATION:return "Invalid configuration";
    default:return "External fault";
    }
}
const char* phaseName(Phase p) {
    switch(p) {
    case WAIT_BALANCE:return "WAIT_BALANCE";case DRIVE:return "DRIVE";case STOP:return "STOP";
    case TURN:return "TURN";case TURN_SETTLE:return "TURN_SETTLE";case DONE:return "DONE";
    case WAIT:return "WAIT";default:return "FAULT";
    }
}
}
