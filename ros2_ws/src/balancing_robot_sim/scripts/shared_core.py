"""Thin desktop binding to cpp/src. No controller or sequence equations here."""
import ctypes as c
import math
import os
from pathlib import Path

HERE = Path(__file__).absolute().parent  # Preserve the installed module's symlink location.
_paths = []
if os.environ.get("ROBOT_CORE_LIBRARY"):
    _paths.append(Path(os.environ["ROBOT_CORE_LIBRARY"]))
_paths.append(HERE / "librobot_core.so")
for parent in HERE.parents:
    _paths.append(parent / "cpp/build/librobot_core.so")
_library = next((path for path in _paths if path.is_file()), None)
if _library is None:
    raise RuntimeError("Shared C++ core is not built. In Ubuntu run: bash build_cpp.sh test")
lib = c.CDLL(str(_library))
LIBRARY_PATH = _library.resolve()
D, P, V = c.c_double, c.POINTER(c.c_double), c.c_void_p


def bind(name, args, result=None):
    fn = getattr(lib, "robot_"+name)
    fn.argtypes, fn.restype = args, result
    return fn


if bind("api_version", [], c.c_int)() != 3:
    raise RuntimeError("Shared C++ ABI mismatch; rebuild the ROS package")
_attitude, _quaternion = bind("attitude", [P, P]), bind("quaternion", [P, P])
wrap = bind("wrap", [D], D)
_heading_rate = bind("heading_rate", [P], D)
_allocate = bind("allocate", [P, P])
duty_for_torque = bind("duty", [D, D, c.c_int, D], D)
_fault_name = bind("fault_name", [c.c_int], c.c_char_p)


def array(values):
    return (D*len(values))(*values)


def vector(fn, values, count):
    output = (D*count)()
    fn(array(values), output)
    return tuple(output)


def attitude(x, y, z, w):
    return vector(_attitude, (x, y, z, w), 3)


def quaternion(roll, pitch, yaw):
    return vector(_quaternion, (roll, pitch, yaw), 4)


def yaw_rate(roll, pitch, gyro_y, gyro_z):
    return _heading_rate(array((roll, pitch, gyro_y, gyro_z)))


def allocate_torque(balance, steering, limit):
    return vector(_allocate, (balance, steering, limit), 2)


def fault_name(code):
    return _fault_name(int(code)).decode()


class Owned:
    def __init__(self, pointer, destroy):
        if not pointer:
            raise MemoryError("Could not construct C++ core object")
        self.pointer, self.destroy = pointer, destroy

    def __del__(self):
        if getattr(self, "pointer", None):
            self.destroy(self.pointer)
            self.pointer = None


_ref_new = bind("reference_create", [D, D, D], V)
_ref_delete = bind("reference_delete", [V])
_ref_update = bind("reference_update", [V, P, P])


class MotionReference(Owned):
    def __init__(self, acceleration=.15, yaw_acceleration=.35, lead_limit=.08):
        super().__init__(_ref_new(acceleration, yaw_acceleration, lead_limit), _ref_delete)
        self.position = None
        self.velocity = self.yaw_velocity = 0.0

    def update(self, distance, velocity_command, yaw_command, dt):
        output = (D*3)()
        _ref_update(self.pointer, array((distance, velocity_command, yaw_command, dt)), output)
        self.position, self.velocity, self.yaw_velocity = output
        return tuple(output)


_yaw_new = bind("yaw_create", [D, D, D], V)
_yaw_delete = bind("yaw_delete", [V])
_yaw_update = bind("yaw_update", [V, P], D)
_yaw_integral = bind("yaw_integral", [V], D)


class YawController(Owned):
    def __init__(self, kp=.01, ki=.02, limit=.012):
        super().__init__(_yaw_new(kp, ki, limit), _yaw_delete)

    @property
    def integral(self):
        return _yaw_integral(self.pointer)

    def update(self, target_rate, measured_rate, dt, balance_torque, total_limit):
        return _yaw_update(self.pointer, array((target_rate, measured_rate, dt, balance_torque, total_limit)))


_odom_new = bind("odometry_create", [], V)
_odom_delete = bind("odometry_delete", [V])
_odom_update = bind("odometry_update", [V, P, P])


class PlanarOdometry(Owned):
    def __init__(self):
        super().__init__(_odom_new(), _odom_delete)
        self.x = self.y = 0.0

    def update(self, distance, world_yaw):
        output = (D*3)()
        _odom_update(self.pointer, array((distance, world_yaw)), output)
        self.x, self.y = output[:2]
        return tuple(output)


_control_new = bind("controller_create", [P], V)
_control_delete = bind("controller_delete", [V])
_control_update = bind("controller_update", [V, P, P])


def controller_parameters(p):
    return (p["kp"], p["kd"], p["total_torque_limit_nm"], p["position_gain"], p["velocity_gain"],
                  math.radians(p["target_tilt_limit_deg"]), math.radians(p["fall_cutoff_deg"]),
                  p["sensor_timeout_s"], p["motion_acceleration_m_s2"], p["yaw_acceleration_rad_s2"],
                  p.get("reference_lead_limit_m", .08), p["yaw_rate_gain"], p["yaw_rate_integral_gain"],
                  p["yaw_torque_limit_nm"], p["max_speed_m_s"], p["max_yaw_rate_rad_s"], p.get("ki", 0.0))


class Controller(Owned):
    def __init__(self, settings):
        if not settings.get("wheel_feedback_enabled", False) and (
                settings["position_gain"] or settings["velocity_gain"] or settings.get("motion_enabled", False)):
            raise ValueError("This robot has no encoders: position/speed feedback and measured-distance motion are unavailable")
        super().__init__(_control_new(array(controller_parameters(settings))), _control_delete)

    def update(self, pitch, rate, position, velocity, heading_rate, total_tilt,
               imu_age, wheel_age, wheel_feedback, velocity_command, yaw_command, dt, motion):
        output = (D*7)()
        _control_update(self.pointer, array((pitch, rate, position, velocity, heading_rate, total_tilt,
            imu_age, wheel_age, wheel_feedback, velocity_command, yaw_command, dt, motion)), output)
        return tuple(output)


_routine_new = bind("routine_create", [P], V)
_routine_delete = bind("routine_delete", [V])
_routine_fail = bind("routine_fail", [V])
_routine_update = bind("routine_update", [V, P, P])


class Routine(Owned):
    def __init__(self, settings):
        p = settings
        values = (p.leg_distance_m, p.speed_m_s, p.acceleration_m_s2, p.turn_rate_rad_s,
                  p.turn_acceleration_rad_s2, p.position_gain_s_inv, p.heading_gain_s_inv,
                  p.heading_damping, p.position_tolerance_m, math.radians(p.heading_tolerance_deg),
                  p.stop_speed_m_s, p.stop_yaw_rate_rad_s, p.settle_time_s, p.turn_settle_time_s,
                  p.phase_timeout_s, p.cycles)
        super().__init__(_routine_new(array(values)), _routine_delete)

    def update(self, state, wheel_feedback=True):
        output = (D*11)()
        _routine_update(self.pointer, array((state.time_s, state.x, state.y, state.yaw, state.pitch,
            state.pitch_rate, state.speed, state.yaw_rate, wheel_feedback)), output)
        return tuple(output)

    def fail(self):
        _routine_fail(self.pointer)


_batch_new = bind("batch_create", [c.c_int, P, P], V)
_batch_delete = bind("batch_delete", [V])
_batch_update = bind("batch_update", [V, P, P, P, P, P])
_batch_prime = bind("batch_prime_imu", [V, P])
_imu_new = bind("imu_create", [], V)
_imu_delete = bind("imu_delete", [V])
_imu_update = bind("imu_update", [V, P, D, c.c_int, P])


class ImuEstimator(Owned):
    def __init__(self):
        super().__init__(_imu_new(), _imu_delete)

    def update(self, measurements, dt=.005, quantize=True):
        """Six values: acceleration in g followed by angular velocity in rad/s."""
        if len(measurements) != 6:
            raise ValueError("Need three accelerometer and three gyro channels")
        output = (D*5)()
        _imu_update(self.pointer, array(measurements), dt, int(quantize), output)
        return tuple(output)


class NativeBatch(Owned):
    def __init__(self, settings, gains):
        import numpy as np
        if settings.get("wheel_feedback_enabled", False) or np.any(np.asarray(gains)[:, 2:4] != 0):
            raise ValueError("GPU hardware mode is IMU-only; wheel gains must be zero")
        self.count = len(gains)
        if np.asarray(gains).shape != (self.count, 6):
            raise ValueError("Need Kp, Kd, zero position/speed gains, torque cap and Ki")
        self.diagnostic = np.zeros((self.count, 4), dtype=np.float64)
        self.torques = np.zeros((self.count, 2), dtype=np.float64)
        super().__init__(_batch_new(self.count, array(controller_parameters(settings)), array(gains.ravel())), _batch_delete)

    def prime_imu(self, measurements):
        import numpy as np
        samples = np.ascontiguousarray(measurements, dtype=np.float64)
        if samples.shape != (self.count, 6):
            raise ValueError("Invalid IMU calibration array")
        _batch_prime(self.pointer, samples.ctypes.data_as(P))

    def update(self, velocities, measurements, parameters):
        import numpy as np
        arrays = [np.ascontiguousarray(a, dtype=np.float64) for a in (velocities, measurements, parameters)]
        if arrays[0].shape != (self.count, 8) or arrays[1].shape != (self.count, 6) or arrays[2].size != 12:
            raise ValueError("Invalid MuJoCo adapter array shapes")
        _batch_update(self.pointer, *(a.ctypes.data_as(P) for a in arrays),
                      self.torques.ctypes.data_as(P), self.diagnostic.ctypes.data_as(P))
        return self.torques, self.diagnostic
