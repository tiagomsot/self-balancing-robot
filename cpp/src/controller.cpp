#include "RobotCore.h"
#include <math.h>
namespace robot {
MotionReference::MotionReference(Real a, Real ya, Real lead):position(0),velocity(0),yaw_velocity(0),
    acceleration_(a),yaw_acceleration_(ya),lead_(lead),initialized_(false) {}
void MotionReference::update(Real distance, Real vc, Real wc, Real dt, Real* o) {
    if (!initialized_) { position=distance; initialized_=true; }
    dt=dt<0?0:(dt>.05?.05:dt);
    Real old=velocity;
    velocity+=clip(vc-velocity,acceleration_*dt);
    yaw_velocity+=clip(wc-yaw_velocity,yaw_acceleration_*dt);
    position+=(old+velocity)*dt/2;
    position=distance+clip(position-distance,lead_);
    o[0]=position; o[1]=velocity; o[2]=yaw_velocity;
}
YawController::YawController(Real kp,Real ki,Real limit):integral(0),kp_(kp),ki_(ki),limit_(limit) {}
Real YawController::update(Real target, Real measured, Real dt, Real balance, Real total_limit) {
    dt=dt<0?0:(dt>.05?.05:dt);
    Real available=fmin(limit_,fmax(0,total_limit-fabs(clip(balance,total_limit))));
    Real error=target-measured, proposed=clip(integral+ki_*error*dt,limit_);
    Real output=kp_*error+proposed;
    if (!(fabs(output)>available && error*output>0)) integral=proposed;
    if (fabs(target)<.005 && fabs(measured)<.02) integral*=exp(-dt/.5);
    return clip(kp_*error+integral,available);
}
void allocateTorque(Real balance,Real difference,Real limit,Real* out) {
    Real half=clip(balance,limit)/2;
    Real steering=clip(difference/2,fmax(0,limit/2-fabs(half)));
    out[0]=half-steering; out[1]=half+steering;
}
Controller::Controller(const BalanceSettings& p):p_(p),
    reference_(p.acceleration,p.yaw_acceleration,p.lead_limit),yaw_(p.yaw_kp,p.yaw_ki,p.yaw_limit),
    fault_(NO_FAULT),disarmed_(false),pitch_integral_(0) {
    const Real values[]={p.kp,p.kd,p.torque_limit,p.position_gain,p.velocity_gain,p.tilt_limit,
        p.fall_limit,p.sensor_timeout,p.acceleration,p.yaw_acceleration,p.lead_limit,
        p.yaw_kp,p.yaw_ki,p.yaw_limit,p.max_speed,p.max_yaw_rate,p.ki};
    for (uint8_t i=0;i<17;++i) if (!finite(values[i])||values[i]<0) fault_=INVALID_CONFIGURATION;
    if (p.kp<=0||p.torque_limit<=0||p.fall_limit<=0||p.fall_limit>=pi/2||p.sensor_timeout<=0)
        fault_=INVALID_CONFIGURATION;
}
void Controller::disarm() { disarmed_=true; }
MotorCommand Controller::update(const ControlState& s,MotionTarget target,Real dt,bool motion) {
    MotorCommand out={0,0,0,0,0,false,fault_};
    if (disarmed_||fault_!=NO_FAULT) return out;
    const Real values[]={s.pitch,s.pitch_rate,s.yaw_rate,s.total_tilt,
                         s.imu_age,target.velocity,target.yaw_rate};
    for (uint8_t i=0;i<7;++i) if (!finite(values[i])) fault_=INVALID_INPUT;
    if (s.wheel_feedback&&(!finite(s.position)||!finite(s.velocity)||!finite(s.wheel_age))) fault_=INVALID_INPUT;
    if (!finite(dt)||dt<0) fault_=INVALID_TIME;
    if (s.total_tilt>p_.fall_limit) fault_=FALLEN;
    if (s.imu_age<0||(s.wheel_feedback&&s.wheel_age<0)) fault_=INVALID_INPUT;
    if (s.imu_age>p_.sensor_timeout||(s.wheel_feedback&&s.wheel_age>p_.sensor_timeout)) fault_=STALE_SENSOR;
    if (!s.wheel_feedback&&(motion||p_.position_gain>0||p_.velocity_gain>0)) fault_=MISSING_WHEEL_FEEDBACK;
    if (fault_!=NO_FAULT) { out.fault=fault_; return out; }
    Real reference[3]={0,0,0};
    if (motion) reference_.update(s.position,clip(target.velocity,p_.max_speed),
                                 clip(target.yaw_rate,p_.max_yaw_rate),dt,reference);
    out.target_tilt=s.wheel_feedback?clip(-p_.position_gain*(s.position-reference[0])
                         -p_.velocity_gain*(s.velocity-reference[1]),p_.tilt_limit):0;
    Real error=s.pitch-out.target_tilt;
    Real proposed=clip(pitch_integral_+p_.ki*error*fmin(dt,.05),p_.torque_limit*.5);
    Real raw=p_.kp*error+p_.kd*s.pitch_rate+proposed;
    if(!(fabs(raw)>p_.torque_limit&&error*raw>0))pitch_integral_=proposed;
    out.requested=p_.kp*error+p_.kd*s.pitch_rate+pitch_integral_;
    out.bounded=clip(out.requested,p_.torque_limit);
    Real difference=motion?yaw_.update(reference[2],s.yaw_rate,dt,out.bounded,p_.torque_limit):0;
    Real wheels[2]; allocateTorque(out.bounded,difference,p_.torque_limit,wheels);
    out.left=wheels[0]; out.right=wheels[1]; out.enabled=true; return out;
}
}
