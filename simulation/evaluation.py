"""Metrics from sampled motion and held torque commands; no smoothing of signals."""

import numpy as np


def significant_crossings(values, threshold=0.1):
    """Count side changes only after reaching +/-threshold, ignoring tiny chatter."""
    previous, count = 0, 0
    for value in values:
        side = 1 if value >= threshold else -1 if value <= -threshold else 0
        if side:
            if previous and side != previous:
                count += 1
            previous = side
    return count


def evaluate(time_s, state, requested, applied, fell, wheel_radius_m):
    time_s, state, requested, applied = [np.asarray(a, dtype=float) for a in
                                       (time_s, state, requested, applied)]
    n = len(time_s)
    if (time_s.shape != (n,) or n < 2 or state.shape != (n, 4)
            or requested.shape != (n,) or applied.shape != (n,)):
        raise ValueError("A trajectory needs >=2 samples with time, four states, and two torque columns.")
    if not all(np.all(np.isfinite(a)) for a in (time_s, state, requested, applied)):
        raise ValueError("Trajectory contains non-finite values.")
    dt = np.diff(time_s)
    if np.any(dt <= 0) or not np.isfinite(wheel_radius_m) or wheel_radius_m <= 0:
        raise ValueError("Time must strictly increase and wheel radius must be positive.")
    duration = float(time_s[-1] - time_s[0])

    def integral(values):
        # Trapezoidal interpolation for continuous, sampled state/error signals.
        return float(np.sum((values[:-1] + values[1:]) * dt / 2))

    angle = np.rad2deg(state[:, 2])
    rate = np.rad2deg(state[:, 3])
    in_band = (np.abs(angle) <= 1) & (np.abs(rate) <= 5)
    remains = np.logical_and.accumulate(in_band[::-1])[::-1]
    eligible = remains & (time_s <= time_s[-1] - 0.5 + 1e-12)
    settled = None if fell or not np.any(eligible) else float(time_s[np.flatnonzero(eligible)[0]] - time_s[0])
    # Overshoot here means passing upright to the opposite side of the initial lean.
    initial = float(angle[0])
    overshoot = None if abs(initial) < 1e-9 else float(max(0, np.max(-np.sign(initial) * angle)))
    clipped = np.abs(requested[:-1] - applied[:-1]) > 1e-12
    clipping_duration = float(np.sum(dt[clipped]))
    relative_output_rate = state[:, 1] / wheel_radius_m - state[:, 3]
    return {
        "fell": bool(fell),
        "simulated_duration_s": duration,
        "final_tilt_deg": float(angle[-1]),
        "max_abs_tilt_deg": float(np.max(np.abs(angle))),
        "pitch_settling_time_s": settled,
        "settling_definition": "|tilt| <= 1 deg and |rate| <= 5 deg/s through the end, for >= 0.5 s",
        "opposite_side_overshoot_deg": overshoot,
        "overshoot_percent_of_initial_tilt": None if overshoot is None else overshoot / abs(initial) * 100,
        "significant_upright_crossings": significant_crossings(angle),
        "crossing_threshold_deg": 0.1,
        "rms_tilt_deg": float(np.sqrt(integral(angle**2) / duration)),
        "integral_absolute_tilt_deg_s": integral(np.abs(angle)),
        "peak_abs_pitch_rate_deg_s": float(np.max(np.abs(rate))),
        "final_axle_position_m": float(state[-1, 0]),
        "final_axle_speed_m_s": float(state[-1, 1]),
        "peak_abs_axle_speed_m_s": float(np.max(np.abs(state[:, 1]))),
        "max_abs_axle_excursion_m": float(np.max(np.abs(state[:, 0] - state[0, 0]))),
        "peak_gearbox_relative_speed_rpm": float(np.max(np.abs(relative_output_rate)) * 60 / (2*np.pi)),
        "peak_requested_total_torque_Nm": float(np.max(np.abs(requested))),
        "peak_applied_total_torque_Nm": float(np.max(np.abs(applied))),
        "saturated_control_fraction": clipping_duration / duration,
        "saturated_duration_s": clipping_duration,
        # Actuation is zero-order held; the final logged command has no following interval.
        "total_torque_squared_effort_Nm2_s": float(np.sum(applied[:-1]**2 * dt)),
    }
