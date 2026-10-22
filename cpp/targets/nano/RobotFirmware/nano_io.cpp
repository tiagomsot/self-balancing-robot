#include <Arduino.h>
#include <math.h>
#include "GeneratedConfig.h"
#include "nano_io.h"
#include "../common/Mpu6050Device.h"
#include "../common/NanoMotorDevice.h"

static robot::ImuEstimator imuEstimator;
static nanoio::Mpu6050Device imu;
static nanoio::NanoMotorDevice motors;
static double gyroXBias=0, gyroYBias=0;
static uint16_t calibrationSamples=0;
static bool ready=false;
bool beginHardware() {
    ready=false;
    if(!motors.begin()) return false; // Validates BoardConfig before touching GPIO.
    ready=imu.begin(imuAddress);
    return ready;
}
bool readHardware(double dt,robot::ControlState& state) {
    if(!ready) return false;
    nanoio::MpuSample sample;
    if(!imu.read(sample)) return false;
    // These are SENSOR axes. Remap here if the real mounting differs from
    // the required robot frame: x forward, y left, z up.
    const double ax=sample.ax, ay=sample.ay, az=sample.az;
    double gx=sample.gx*robot::pi/180, gy=sample.gy*robot::pi/180;
    // Calibrate while stationary; control remains disabled for the first 2.5 s.
    if(calibrationSamples<500) {
        gyroXBias+=gx;gyroYBias+=gy;++calibrationSamples;
        if(calibrationSamples==500){gyroXBias/=500;gyroYBias/=500;}
        return false;
    }
    gx-=gyroXBias;gy-=gyroYBias;
    const robot::Real measurements[]={ax,ay,az,gx,gy,sample.gz*robot::pi/180};
    return imuEstimator.update(measurements,dt,state);
}
static int torqueToPwm(double request) {
    double duty=robot::dutyForTorque(request,fullPwmTorque,pwmLevels,commandDeadband);
    // Quantization is shared C++; analogWrite itself expects 0..255.
    return static_cast<int>(fabs(duty)*255+.5)*(duty<0?-1:1);
}
void disableMotors() {
    motors.stop();
}
void writeMotors(const robot::MotorCommand& c) {
    if(!ready||!c.enabled){disableMotors();return;}
    motors.writePwm(torqueToPwm(c.left),torqueToPwm(c.right));
}
