"""Your first contribution: implement this one PD function.

Read docs/first-experiment.md, then try:
    python run_simulation.py --controller student
"""


def student_pd(angle_rad, angular_rate_rad_s, kp, kd, target_angle_rad=0.0):
    """Return the TOTAL requested wheel torque in N m (not PWM).

    Inputs are radians and radians/second. Positive angle means leaning forward;
    positive motor torque drives the wheels forward. Keep this function small:
    the simulation already handles timing, limits, integration, and plotting.
    """
    angle_error = angle_rad - target_angle_rad

    # TODO: Calculate the proportional contribution using kp and angle_error.
    # TODO: Calculate the rate contribution using kd and angular_rate_rad_s.
    # TODO: Return their combined torque, as a number.
    raise NotImplementedError(
        "Complete the PD function in student_controller.py first. "
        "Use --controller reference to run the working example."
    )
