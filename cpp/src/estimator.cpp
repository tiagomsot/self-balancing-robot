#include "RobotCore.h"
#include <math.h>
namespace robot {
Real clip(Real v, Real limit) { return v < -limit ? -limit : (v > limit ? limit : v); }
Real wrap(Real v) { return atan2(sin(v), cos(v)); }
bool finite(Real v) { return isfinite(v); }
void attitude(Real x, Real y, Real z, Real w, Real* o) {
    o[0]=atan2(2*(w*x+y*z),1-2*(x*x+y*y));
    o[1]=asin(clip(2*(w*y-z*x),1));
    o[2]=atan2(2*(w*z+x*y),1-2*(y*y+z*z));
}
void quaternion(Real r, Real p, Real y, Real* o) {
    Real cr=cos(r/2), sr=sin(r/2), cp=cos(p/2), sp=sin(p/2), cy=cos(y/2), sy=sin(y/2);
    o[0]=sr*cp*cy-cr*sp*sy; o[1]=cr*sp*cy+sr*cp*sy;
    o[2]=cr*cp*sy-sr*sp*cy; o[3]=cr*cp*cy+sr*sp*sy;
}
Real headingRate(Real r, Real p, Real gy, Real gz) { return (sin(r)*gy+cos(r)*gz)/cos(p); }
Odometry::Odometry(): x_(0),y_(0),origin_(0),previous_distance_(0),previous_yaw_(0),initialized_(false) {}
void Odometry::update(Real distance, Real world_yaw, Real* o) {
    if (!initialized_) { origin_=world_yaw; previous_distance_=distance; initialized_=true; }
    Real yaw=wrap(world_yaw-origin_), midpoint=previous_yaw_+wrap(yaw-previous_yaw_)/2;
    Real delta=distance-previous_distance_;
    x_+=delta*cos(midpoint); y_+=delta*sin(midpoint);
    previous_distance_=distance; previous_yaw_=yaw;
    o[0]=x_; o[1]=y_; o[2]=yaw;
}
PitchEstimator::PitchEstimator(Real tau):angle_(0),time_constant_(tau),initialized_(false) {}
bool PitchEstimator::update(Real ax, Real az, Real gy, Real dt, Real* out) {
    if (!finite(ax)||!finite(az)||!finite(gy)||!finite(dt)||dt<=0||dt>.05) return false;
    // Do not interpret near-free-fall or an impact as the gravity direction.
    // Inputs are in g, as read from the MPU-6050. Propagate the gyro instead.
    if (ax*ax+az*az<.25*.25||ax*ax+az*az>1.75*1.75) {
        if(!initialized_)return false;
        angle_+=gy*dt;*out=angle_;return finite(angle_);
    }
    Real measured=atan2(-ax,az);
    if (!initialized_) { angle_=measured; initialized_=true; }
    Real alpha=time_constant_/(time_constant_+dt);
    angle_=alpha*(angle_+gy*dt)+(1-alpha)*measured;
    *out=angle_; return finite(angle_);
}
void quantizeImu(Real* a) {
    for (uint8_t i=0;i<6;++i) {
        if(!finite(a[i]))continue;
        Real scale=i<3?16384:131*180/pi;
        a[i]=fmax(-32768,fmin(32767,round(a[i]*scale)))/scale;
    }
}
bool ImuEstimator::update(const Real* a, Real dt, ControlState& s) {
    for (uint8_t i=0;i<6;++i) if (!finite(a[i])) return false;
    Real pitch,roll;
    if (!pitch_.update(a[0],a[2],a[4],dt,&pitch)||
        !roll_.update(-a[1],a[2],a[3],dt,&roll)) return false;
    s.pitch=pitch;s.pitch_rate=a[4];s.position=s.velocity=0;s.yaw_rate=a[5];
    s.total_tilt=acos(clip(cos(roll)*cos(pitch),1));
    s.imu_age=s.wheel_age=0;s.wheel_feedback=false;
    return true;
}
}
