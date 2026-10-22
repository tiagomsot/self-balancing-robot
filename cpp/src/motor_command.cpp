#include "RobotCore.h"
#include <math.h>
namespace robot {
Real dutyForTorque(Real request,Real full,int levels,Real deadband) {
    if (!finite(request)||!finite(full)||full<=0||levels<1||!finite(deadband)||deadband<0||deadband>=1) return 0;
    // Explicit round-to-even, matching the Python motor model at half steps.
    Real raw=clip(request/full,1)*levels, magnitude=fabs(raw), low=floor(magnitude);
    Real fraction=magnitude-low;
    if (fraction>.5||(fraction==.5&&fmod(low,2)!=0)) low+=1;
    Real duty=(raw<0?-low:low)/levels;
    return fabs(duty)<deadband?0:duty;
}
}
