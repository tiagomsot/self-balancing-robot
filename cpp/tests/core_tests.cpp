#include "RobotCore.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
using namespace robot;
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);}} while(0)
static BalanceSettings balance() {
    BalanceSettings p={1,.1,.04,.15,.35,5*pi/180,35*pi/180,.25,.15,.35,.08,.01,.02,.012,.25,50*pi/180};return p;
}
static RoutineSettings settings() {
    RoutineSettings p={.5,.25,.15,50*pi/180,.35,.8,1.2,.2,.005,3*pi/180,.01,.05,.3,.2,40,2};return p;
}
int main() {
    Real wheels[2];
    for(int i=-10;i<=10;++i) for(int j=-10;j<=10;++j) {
        allocateTorque(i*.01,j*.01,.04,wheels);
        CHECK(fabs(wheels[0])<=.020000001&&fabs(wheels[1])<=.020000001);
        CHECK(fabs(wheels[0]+wheels[1]-clip(i*.01,.04))<1e-12);
    }
    CHECK(dutyForTorque(.5,1,1,0)==0); // Explicit tie-to-even.
    CHECK(dutyForTorque(1.5,2,2,0)==1);
    CHECK(dutyForTorque(NAN,1,255,0)==0);
    Controller control(balance());
    ControlState state={.1,0,0,0,0,.1,0,0,true}; MotionTarget target={0,0};
    MotorCommand command=control.update(state,target,.005,false);
    CHECK(command.enabled&&command.bounded==.04);
    state.total_tilt=1;command=control.update(state,target,.005,false);
    CHECK(!command.enabled&&command.fault==FALLEN);
    state.total_tilt=0;CHECK(control.update(state,target,.005,false).fault==FALLEN);
    Controller stale(balance());state.imu_age=.3;CHECK(stale.update(state,target,.005,false).fault==STALE_SENSOR);
    Controller no_feedback(balance());state.imu_age=0;state.wheel_feedback=false;
    CHECK(no_feedback.update(state,target,.005,false).fault==MISSING_WHEEL_FEEDBACK);
    BalanceSettings no_encoders=balance();no_encoders.position_gain=no_encoders.velocity_gain=0;
    Controller pitch_only(no_encoders);CHECK(pitch_only.update(state,target,.005,false).enabled);
    Controller bad_time(balance());state.wheel_feedback=true;
    CHECK(bad_time.update(state,target,-.1,false).fault==INVALID_TIME);
    Routine routine(settings());defineRoutine(routine);
    Feedback s={0,0,0,0,0,0,0,0};Real heading=0;
    unsigned steps=0;
    for(;steps<15000&&routine.phase()!=DONE;++steps) {
        s.time_s=steps*.02;MotionTarget c=routine.update(s);
        s.speed=c.velocity;s.yaw_rate=c.yaw_rate;heading+=c.yaw_rate*.02;s.yaw=wrap(heading);
        s.x+=c.velocity*cos(heading)*.02;s.y+=c.velocity*sin(heading)*.02;
    }
    CHECK(routine.phase()==DONE&&routine.legs()==4&&hypot(s.x,s.y)<.03);
    Routine overflow(settings());for(int i=0;i<17;++i) overflow.wait(1);
    CHECK(overflow.fault()==QUEUE_FULL);
    Routine missing(settings());missing.move(.5,.2);s.time_s=0;
    CHECK(missing.update(s,false).velocity==0&&missing.fault()==MISSING_WHEEL_FEEDBACK);
    Routine waiting(settings());waiting.wait(1);s.time_s=0;s.speed=s.yaw_rate=0;
    CHECK(waiting.update(s).velocity==0&&waiting.phase()==WAIT);
    for(int i=1;i<100;++i){s.time_s=i*.005;waiting.update(s);CHECK(waiting.phase()==WAIT);}
    s.time_s=1;waiting.update(s);CHECK(waiting.phase()==DONE);
    Routine reverse(settings());reverse.wait(1);s.time_s=2;reverse.update(s);s.time_s=1;reverse.update(s);
    CHECK(reverse.fault()==INVALID_TIME);
    PitchEstimator estimator;Real pitch=0;
    CHECK(estimator.update(-sin(.1),cos(.1),0,.005,&pitch)&&fabs(pitch-.1)<1e-9);
    CHECK(estimator.update(0,0,0,.005,&pitch)); // Free-fall: propagate gyro after initialization.
    PitchEstimator empty;CHECK(!empty.update(0,0,0,.005,&pitch));
    state.wheel_feedback=false;state.position=NAN;state.velocity=NAN;state.wheel_age=NAN;
    Controller imu_only(no_encoders);
    CHECK(imu_only.update(state,target,.005,false).enabled); // Wheel channels are not read.
    printf("Portable core checks passed; sequence used %u ticks. Controller=%zu bytes, Routine=%zu bytes.\n",
           steps,sizeof(Controller),sizeof(Routine));
}
