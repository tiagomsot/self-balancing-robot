// Host adapter: the SAME Controller as Nano/Gazebo drives CUDA physics worlds.
#include "RobotCore.h"
#include <math.h>
#include <new>
using namespace robot;
struct NativeBatch { int count; Controller** controllers; ImuEstimator* imus; };
extern "C" {
void* robot_batch_create(int count,const double* a,const double* gains) {
    if(count<1||count>4096) return 0;
    NativeBatch* b=new(std::nothrow) NativeBatch;
    if(!b) return 0;
    b->count=count;b->controllers=new(std::nothrow) Controller*[count]();
    if(!b->controllers){delete b;return 0;}
    b->imus=new(std::nothrow) ImuEstimator[count];
    if(!b->imus){delete[] b->controllers;delete b;return 0;}
    for(int i=0;i<count;++i) {
        const double* g=gains+6*i;
        BalanceSettings p={g[0],g[1],g[4],g[2],g[3],a[5],a[6],a[7],a[8],a[9],a[10],a[11],a[12],a[13],a[14],a[15],g[5]};
        b->controllers[i]=new(std::nothrow) Controller(p);
        if(!b->controllers[i]) {
            for(int j=0;j<i;++j) delete b->controllers[j];
            delete[] b->controllers;delete[] b->imus;delete b;return 0;
        }
    }
    return b;
}
void robot_batch_delete(void* ptr) {
    NativeBatch* b=static_cast<NativeBatch*>(ptr);
    if(!b) return;
    for(int i=0;i<b->count;++i) delete b->controllers[i];
    delete[] b->controllers;delete[] b->imus;delete b;
}
void robot_batch_prime_imu(void* ptr,const double* samples) {
    NativeBatch* b=static_cast<NativeBatch*>(ptr);
    for(int i=0;i<b->count;++i) {
        Real a[6];for(int j=0;j<6;++j)a[j]=samples[6*i+j];quantizeImu(a);
        ControlState state={};b->imus[i].update(a,.005,state);
    }
}
void robot_batch_update(void* ptr,const double* velocities,const double* samples,
                        const double* p,double* torques,double* diagnostic) {
    NativeBatch* b=static_cast<NativeBatch*>(ptr);
    for(int i=0;i<b->count;++i) {
        // Only motor physics below uses shaft speed. The controller sees IMU only.
        const double* v=velocities+8*i;
        Real a[6];for(int j=0;j<6;++j)a[j]=samples[6*i+j];quantizeImu(a);
        ControlState s={};
        if(!b->imus[i].update(a,.005,s))s.imu_age=1;
        MotionTarget hold={0,0};MotorCommand c=b->controllers[i]->update(s,hold,.005,false);
        if(c.fault!=NO_FAULT) diagnostic[4*i+3]=1;
        diagnostic[4*i+2]=fmax(diagnostic[4*i+2],fabs(c.bounded));
        for(int motor=0;motor<2;++motor) {
            double request=motor==0?c.left:c.right;
            double duty=dutyForTorque(request,p[3],static_cast<int>(p[4]),p[5]);
            double on=0,current=0,shaft_speed=v[6+motor];
            if(duty!=0) {
                on=((duty>0?1:-1)*fmax(0,p[6]-p[7])-p[8]*shaft_speed)/p[9];
                current=fabs(duty)*on;
            }
            torques[2*i+motor]=p[10]*current-p[11]*shaft_speed;
            diagnostic[4*i]=fmax(diagnostic[4*i],fabs(current));
            diagnostic[4*i+1]=fmax(diagnostic[4*i+1],fabs(on));
        }
    }
}
}
