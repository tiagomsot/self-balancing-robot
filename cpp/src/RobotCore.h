#ifndef BALANCING_ROBOT_CORE_H
#define BALANCING_ROBOT_CORE_H

// Portable API: no ROS, Arduino, simulator, heap or operating-system dependency.
#include <stdint.h>

namespace robot {
typedef double Real; // AVR's double is 32-bit; desktop builds normally use 64-bit.
const Real pi = 3.14159265358979323846;
Real clip(Real value, Real limit);
Real wrap(Real angle);
bool finite(Real value);
void attitude(Real x, Real y, Real z, Real w, Real* roll_pitch_yaw);
void quaternion(Real roll, Real pitch, Real yaw, Real* xyzw);
Real headingRate(Real roll, Real pitch, Real gyro_y, Real gyro_z);
void allocateTorque(Real balance, Real difference, Real limit, Real* left_right);
Real dutyForTorque(Real requested, Real full_pwm_torque, int levels, Real deadband);

enum Fault { NO_FAULT, INVALID_INPUT, FALLEN, STALE_SENSOR, MISSING_WHEEL_FEEDBACK,
             INVALID_TIME, PHASE_TIMEOUT, QUEUE_FULL, INVALID_CONFIGURATION, EXTERNAL_FAULT };
const char* faultName(Fault fault);

struct MotionTarget { Real velocity, yaw_rate; };
struct ControlState {
    Real pitch, pitch_rate, position, velocity, yaw_rate, total_tilt;
    Real imu_age, wheel_age;
    bool wheel_feedback;
};
struct BalanceSettings {
    Real kp, kd, torque_limit, position_gain, velocity_gain, tilt_limit, fall_limit;
    Real sensor_timeout, acceleration, yaw_acceleration, lead_limit;
    Real yaw_kp, yaw_ki, yaw_limit, max_speed, max_yaw_rate;
    Real ki; // Pitch integral gain; zero gives PD. Integral output is limited.
};
struct MotorCommand {
    Real requested, bounded, target_tilt, left, right;
    bool enabled;
    Fault fault;
};

class MotionReference {
public:
    MotionReference(Real acceleration=.15, Real yaw_acceleration=.35, Real lead=.08);
    void update(Real distance, Real velocity_command, Real yaw_command, Real dt, Real* output);
    Real position, velocity, yaw_velocity;
private:
    Real acceleration_, yaw_acceleration_, lead_;
    bool initialized_;
};
class YawController {
public:
    YawController(Real kp=.01, Real ki=.02, Real limit=.012);
    Real update(Real target, Real measured, Real dt, Real balance, Real total_limit);
    Real integral;
private:
    Real kp_, ki_, limit_;
};
class Odometry {
public:
    Odometry();
    void update(Real distance, Real world_yaw, Real* output);
private:
    Real x_, y_, origin_, previous_distance_, previous_yaw_;
    bool initialized_;
};
class PitchEstimator {
public:
    PitchEstimator(Real time_constant=.25);
    bool update(Real accel_x, Real accel_z, Real gyro_y, Real dt, Real* pitch);
private:
    Real angle_, time_constant_;
    bool initialized_;
};
// Calibrated accelerometer (g) and gyro (rad/s), in x-forward/y-left/z-up axes.
// No wheel, position, velocity or simulator attitude enters this estimator.
void quantizeImu(Real* accel_gyro);
class ImuEstimator {
public:
    bool update(const Real* accel_gyro, Real dt, ControlState& state);
private:
    PitchEstimator pitch_, roll_;
};
class Controller {
public:
    explicit Controller(const BalanceSettings& settings);
    MotorCommand update(const ControlState& state, MotionTarget target, Real dt, bool motion);
    void disarm();
    Fault fault() const { return fault_; }
private:
    BalanceSettings p_;
    MotionReference reference_;
    YawController yaw_;
    Fault fault_;
    bool disarmed_;
    Real pitch_integral_;
};

struct Feedback {
    Real time_s, x, y, yaw, pitch, pitch_rate, speed, yaw_rate;
};
struct RoutineSettings {
    Real leg_distance, speed, acceleration, turn_rate, turn_acceleration;
    Real position_gain, heading_gain, heading_damping, position_tolerance;
    Real heading_tolerance, stop_speed, stop_yaw_rate, settle_time, turn_settle_time, timeout;
    uint16_t cycles;
};
enum Phase { WAIT_BALANCE, DRIVE, STOP, TURN, TURN_SETTLE, DONE, FAULT, WAIT };
const char* phaseName(Phase phase);

// Calls below append steps ONCE. update() executes them without blocking.
class Routine {
public:
    static const uint8_t capacity = 16;
    explicit Routine(const RoutineSettings& settings);
    void waitUntilBalanced(Real seconds);
    void move(Real metres, Real maximum_speed);
    void stop(Real settled_seconds=0);
    void rotate(Real degrees, Real maximum_degrees_per_second);
    void wait(Real seconds);
    void repeat();
    MotionTarget update(const Feedback& state, bool wheel_feedback=true);
    void fail(Fault reason);
    Phase phase() const { return phase_; }
    Fault fault() const { return fault_; }
    uint16_t legs() const { return legs_; }
    Real progress() const { return progress_; }
    Real remaining() const { return remaining_; }
    Real startX() const { return start_x_; }
    Real startY() const { return start_y_; }
    Real legHeading() const { return leg_heading_; }
    uint8_t stepIndex() const { return index_; }
    uint8_t stepCount() const { return count_; }
    const RoutineSettings& settings() const { return p_; }
private:
    enum Kind { BALANCE_STEP, MOVE_STEP, STOP_STEP, ROTATE_STEP, WAIT_STEP, REPEAT_STEP };
    struct Step { Kind kind; Real value, limit; };
    void append(Kind kind, Real value, Real limit);
    void begin(const Feedback& state);
    void advance(const Feedback& state);
    bool quiet(const Feedback& state) const;
    bool dwell(const Feedback& state, bool condition, Real seconds);
    Real headingCommand(const Feedback& state) const;
    Real centerCommand(const Feedback& state) const;
    RoutineSettings p_;
    Step steps_[capacity];
    uint8_t count_, index_, repeat_index_;
    Phase phase_;
    Fault fault_;
    uint16_t legs_;
    Real entered_, quiet_since_, previous_time_, last_yaw_, unwrapped_yaw_;
    Real heading_target_, turn_target_, start_x_, start_y_, leg_heading_;
    Real goal_x_, goal_y_, progress_, remaining_;
    bool initialized_, step_started_, dwelling_, have_goal_, after_turn_;
};

// Defined in my_routine.cpp. Every target calls the same function.
void defineRoutine(Routine& robot);
} // namespace robot
#endif
