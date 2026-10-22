#include "RobotCore.h"
#include <math.h>
namespace robot {
Routine::Routine(const RoutineSettings& p):p_(p),count_(0),index_(0),repeat_index_(0),
    phase_(WAIT_BALANCE),fault_(NO_FAULT),legs_(0),entered_(0),quiet_since_(0),previous_time_(0),
    last_yaw_(0),unwrapped_yaw_(0),heading_target_(0),turn_target_(0),start_x_(0),start_y_(0),
    leg_heading_(0),goal_x_(0),goal_y_(0),progress_(0),remaining_(0),initialized_(false),
    step_started_(false),dwelling_(false),have_goal_(false),after_turn_(false) {
    const Real v[]={p.leg_distance,p.speed,p.acceleration,p.turn_rate,p.turn_acceleration,
        p.position_gain,p.heading_gain,p.heading_damping,p.position_tolerance,
        p.heading_tolerance,p.stop_speed,p.stop_yaw_rate,p.settle_time,p.turn_settle_time,p.timeout};
    for(uint8_t i=0;i<15;++i) if(!finite(v[i])||v[i]<=0) fail(INVALID_CONFIGURATION);
    if(p.speed>.25||p.turn_rate>50*pi/180+1e-12) fail(INVALID_CONFIGURATION);
}
void Routine::fail(Fault f) { if(fault_==NO_FAULT) fault_=f; phase_=FAULT; }
void Routine::append(Kind kind,Real value,Real limit) {
    if(initialized_) { fail(INVALID_CONFIGURATION); return; }
    if(count_>=capacity) { fail(QUEUE_FULL); return; }
    if(!finite(value)||!finite(limit)) { fail(INVALID_CONFIGURATION); return; }
    steps_[count_].kind=kind;steps_[count_].value=value;steps_[count_].limit=limit;++count_;
}
void Routine::waitUntilBalanced(Real seconds) {
    if(seconds<0) fail(INVALID_CONFIGURATION);
    append(BALANCE_STEP,seconds,0);
    if(count_==1) repeat_index_=1;
}
void Routine::move(Real metres,Real speed) {
    if(metres==0||speed<=0||speed>p_.speed) fail(INVALID_CONFIGURATION);
    append(MOVE_STEP,metres,speed);
}
void Routine::stop(Real seconds) {
    if(seconds<0) fail(INVALID_CONFIGURATION);
    append(STOP_STEP,seconds,0);
}
void Routine::rotate(Real degrees,Real degrees_per_s) {
    if(degrees==0||degrees_per_s<=0||degrees_per_s*pi/180>p_.turn_rate+1e-12) fail(INVALID_CONFIGURATION);
    append(ROTATE_STEP,degrees*pi/180,degrees_per_s*pi/180);
}
void Routine::wait(Real seconds) {
    if(seconds<0) fail(INVALID_CONFIGURATION);
    append(WAIT_STEP,seconds,0);
}
void Routine::repeat() { append(REPEAT_STEP,0,0); }
bool Routine::quiet(const Feedback& s) const {
    return fabs(s.pitch)<pi/180&&fabs(s.pitch_rate)<5*pi/180&&fabs(s.speed)<p_.stop_speed
           &&fabs(s.yaw_rate)<p_.stop_yaw_rate;
}
bool Routine::dwell(const Feedback& s,bool condition,Real seconds) {
    if(!condition) dwelling_=false;
    else if(!dwelling_) {quiet_since_=s.time_s;dwelling_=true;}
    return dwelling_&&s.time_s-quiet_since_>=seconds;
}
Real Routine::headingCommand(const Feedback& s) const {
    return clip(p_.heading_gain*wrap(heading_target_-s.yaw)-p_.heading_damping*s.yaw_rate,p_.turn_rate);
}
Real Routine::centerCommand(const Feedback& s) const {
    if(!have_goal_) return 0;
    Real error=(goal_x_-s.x)*cos(s.yaw)+(goal_y_-s.y)*sin(s.yaw);
    return fabs(error)>.002?clip(.4*error,p_.stop_speed):0;
}
void Routine::begin(const Feedback& s) {
    // Repeat is an instruction, not a timed phase. Bound traversal so an empty
    // or malformed loop cannot hang the fast control tick.
    uint8_t traversed=0;
    while(index_<count_&&steps_[index_].kind==REPEAT_STEP) {
        index_=repeat_index_;
        if(++traversed>=capacity) {fail(INVALID_CONFIGURATION);return;}
    }
    if(index_>=count_) {phase_=DONE;step_started_=true;return;}
    const Step& step=steps_[index_];
    entered_=s.time_s;dwelling_=false;step_started_=true;
    if(step.kind==BALANCE_STEP) phase_=WAIT_BALANCE;
    else if(step.kind==MOVE_STEP) {
        phase_=DRIVE;after_turn_=false;
        start_x_=s.x;start_y_=s.y;leg_heading_=heading_target_;
        goal_x_=start_x_+step.value*cos(leg_heading_);
        goal_y_=start_y_+step.value*sin(leg_heading_);
        have_goal_=true;progress_=0;remaining_=step.value;
    } else if(step.kind==ROTATE_STEP) {
        phase_=TURN;turn_target_=unwrapped_yaw_+step.value;
        heading_target_=wrap(turn_target_);after_turn_=true;
        if(!have_goal_) {goal_x_=s.x;goal_y_=s.y;have_goal_=true;}
    } else if(step.kind==STOP_STEP) {
        phase_=after_turn_?TURN_SETTLE:STOP;
        if(!have_goal_) {goal_x_=s.x;goal_y_=s.y;have_goal_=true;}
    } else phase_=after_turn_?TURN_SETTLE:WAIT;
}
void Routine::advance(const Feedback& s) { ++index_;step_started_=false;begin(s); }
MotionTarget Routine::update(const Feedback& s,bool wheels) {
    MotionTarget out={0,0};
    const Real values[]={s.time_s,s.x,s.y,s.yaw,s.pitch,s.pitch_rate,s.speed,s.yaw_rate};
    for(uint8_t i=0;i<8;++i) if(!finite(values[i])) {fail(INVALID_INPUT);return out;}
    if(initialized_&&s.time_s<previous_time_) {fail(INVALID_TIME);return out;}
    previous_time_=s.time_s;
    if(fault_!=NO_FAULT) return out;
    if(!initialized_) {
        initialized_=true;heading_target_=last_yaw_=unwrapped_yaw_=turn_target_=s.yaw;
    } else unwrapped_yaw_+=wrap(s.yaw-last_yaw_);
    last_yaw_=s.yaw;
    if(!step_started_) begin(s);
    if(phase_==FAULT) return out;
    out.yaw_rate=headingCommand(s);
    if(phase_==DONE) return out;
    const Step step=steps_[index_];
    if(!wheels&&(step.kind==MOVE_STEP||step.kind==ROTATE_STEP)) {
        fail(MISSING_WHEEL_FEEDBACK);out.yaw_rate=0;return out;
    }
    if(step.kind!=WAIT_STEP&&s.time_s-entered_>p_.timeout) {
        fail(PHASE_TIMEOUT);out.yaw_rate=0;return out;
    }
    if(step.kind==BALANCE_STEP) {
        if(dwell(s,quiet(s),step.value)) advance(s);
    } else if(step.kind==MOVE_STEP) {
        progress_=(s.x-start_x_)*cos(leg_heading_)+(s.y-start_y_)*sin(leg_heading_);
        remaining_=(goal_x_-s.x)*cos(leg_heading_)+(goal_y_-s.y)*sin(leg_heading_);
        Real limit=fmin(step.limit,sqrt(2*p_.acceleration*fabs(remaining_)));
        out.velocity=clip(p_.position_gain*remaining_,limit);
        if(fabs(remaining_)<=p_.position_tolerance&&quiet(s)) {
            ++legs_;out.velocity=0;advance(s);
        }
    } else if(step.kind==STOP_STEP) {
        Real error=(goal_x_-s.x)*cos(leg_heading_)+(goal_y_-s.y)*sin(leg_heading_);
        out.velocity=fabs(error)>.002?clip(p_.position_gain*error,p_.stop_speed):0;
        bool aligned=fabs(wrap(heading_target_-s.yaw))<p_.heading_tolerance;
        if(dwell(s,quiet(s)&&fabs(error)<p_.position_tolerance&&aligned,step.value)) {
            if(p_.cycles&&legs_>=2*p_.cycles) phase_=DONE;
            else advance(s);
        }
    } else if(step.kind==ROTATE_STEP) {
        Real error=turn_target_-unwrapped_yaw_;
        Real limit=fmin(step.limit,sqrt(2*p_.turn_acceleration*fabs(error)));
        out.yaw_rate=clip(p_.heading_gain*error-p_.heading_damping*s.yaw_rate,limit);
        out.velocity=centerCommand(s);
        bool centered=hypot(goal_x_-s.x,goal_y_-s.y)<2*p_.position_tolerance;
        if(fabs(error)<p_.heading_tolerance&&quiet(s)&&centered) advance(s);
    } else if(step.kind==WAIT_STEP) {
        out.velocity=centerCommand(s);
        bool settled=quiet(s)&&fabs(wrap(heading_target_-s.yaw))<p_.heading_tolerance;
        if(after_turn_?dwell(s,settled,step.value):(s.time_s-entered_>=step.value)) advance(s);
    }
    if(phase_==FAULT) {out.velocity=0;out.yaw_rate=0;}
    return out;
}
}
