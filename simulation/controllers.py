"""Working reference controllers; your exercise lives in student_controller.py."""

from .model import RobotParameters

# Ideal pitch poles -8 +/- 8j for the current geometry. The ROS motor-model
# controller is tuned separately; these are delivered-torque reference gains.
_robot = RobotParameters()
_determinant = _robot.effective_mass*_robot.axle_inertia-_robot.coupling**2
_angle_coefficient = _robot.effective_mass*_robot.coupling*_robot.gravity_m_s2/_determinant
_torque_coefficient = (_robot.effective_mass+_robot.coupling/_robot.wheel_radius_m)/_determinant
REFERENCE_KP = (_angle_coefficient+128)/_torque_coefficient
REFERENCE_KD = 16/_torque_coefficient


def reference_pd(angle_rad, angular_rate_rad_s, kp, kd, target_angle_rad=0.0):
    """Total requested torque, split equally between two wheels by this model.

    Positive forward lean requires positive wheel torque with our conventions.
    The rate term damps motion; positive angular rate calls for earlier correction.
    """
    error = angle_rad - target_angle_rad
    return kp * error + kd * angular_rate_rad_s


def make_controller(name, kp, kd):
    if name == "reference":
        function = reference_pd
    elif name == "student":
        from student_controller import student_pd
        function = student_pd
    elif name == "off":
        return lambda angle, rate: 0.0
    else:
        raise ValueError(f"Unknown controller: {name}")
    return lambda angle, rate: function(angle, rate, kp, kd)
