// Desktop-only C ABI for Python/ROS and simulator bindings. Heap use is confined
// to object construction/destruction here; the portable control tick never allocates.
#include "RobotCore.h"
#include <new>
using namespace robot;
extern "C" {
int robot_api_version() {return 3;}
void* robot_imu_create(){return new(std::nothrow) ImuEstimator();}
void robot_imu_delete(void* p){delete static_cast<ImuEstimator*>(p);}
void robot_imu_update(void* p,const double* a,double dt,int quantize,double* o){
    Real sample[6];for(int i=0;i<6;++i)sample[i]=a[i];
    if(quantize)quantizeImu(sample);
    ControlState s={};bool valid=static_cast<ImuEstimator*>(p)->update(sample,dt,s);
    o[0]=s.pitch;o[1]=s.pitch_rate;o[2]=s.total_tilt;o[3]=s.yaw_rate;o[4]=valid?1:0;
}
void robot_attitude(const double* a,double* o){attitude(a[0],a[1],a[2],a[3],o);}
void robot_quaternion(const double* a,double* o){quaternion(a[0],a[1],a[2],o);}
double robot_wrap(double a){return wrap(a);}
double robot_heading_rate(const double* a){return headingRate(a[0],a[1],a[2],a[3]);}
void robot_allocate(const double* a,double* o){allocateTorque(a[0],a[1],a[2],o);}
double robot_duty(double r,double f,int levels,double deadband){return dutyForTorque(r,f,levels,deadband);}
const char* robot_fault_name(int f){return faultName(static_cast<Fault>(f));}
void* robot_reference_create(double a,double ya,double lead){return new(std::nothrow) MotionReference(a,ya,lead);}
void robot_reference_delete(void* p){delete static_cast<MotionReference*>(p);}
void robot_reference_update(void* p,const double* a,double* o){static_cast<MotionReference*>(p)->update(a[0],a[1],a[2],a[3],o);}
void* robot_yaw_create(double kp,double ki,double limit){return new(std::nothrow) YawController(kp,ki,limit);}
void robot_yaw_delete(void* p){delete static_cast<YawController*>(p);}
double robot_yaw_update(void* p,const double* a){return static_cast<YawController*>(p)->update(a[0],a[1],a[2],a[3],a[4]);}
double robot_yaw_integral(void* p){return static_cast<YawController*>(p)->integral;}
void* robot_odometry_create(){return new(std::nothrow) Odometry();}
void robot_odometry_delete(void* p){delete static_cast<Odometry*>(p);}
void robot_odometry_update(void* p,const double* a,double* o){static_cast<Odometry*>(p)->update(a[0],a[1],o);}
void* robot_controller_create(const double* a){
    BalanceSettings p={a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],a[10],a[11],a[12],a[13],a[14],a[15],a[16]};
    return new(std::nothrow) Controller(p);
}
void robot_controller_delete(void* p){delete static_cast<Controller*>(p);}
void robot_controller_update(void* p,const double* a,double* o){
    ControlState s={a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8]!=0};
    MotionTarget target={a[9],a[10]};
    MotorCommand c=static_cast<Controller*>(p)->update(s,target,a[11],a[12]!=0);
    o[0]=c.requested;o[1]=c.bounded;o[2]=c.target_tilt;o[3]=c.left;o[4]=c.right;o[5]=c.enabled?1:0;o[6]=c.fault;
}
void* robot_routine_create(const double* a){
    RoutineSettings p={a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],a[10],a[11],a[12],a[13],a[14],static_cast<uint16_t>(a[15])};
    Routine* routine=new(std::nothrow) Routine(p);
    if(routine) defineRoutine(*routine);
    return routine;
}
void robot_routine_delete(void* p){delete static_cast<Routine*>(p);}
void robot_routine_fail(void* p){static_cast<Routine*>(p)->fail(EXTERNAL_FAULT);}
void robot_routine_update(void* p,const double* a,double* o){
    Feedback s={a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7]};
    Routine* r=static_cast<Routine*>(p); MotionTarget c=r->update(s,a[8]!=0);
    o[0]=c.velocity;o[1]=c.yaw_rate;o[2]=r->phase();o[3]=r->legs();o[4]=r->progress();o[5]=r->remaining();
    o[6]=r->fault();o[7]=r->startX();o[8]=r->startY();o[9]=r->legHeading();o[10]=r->stepIndex();
}
}
