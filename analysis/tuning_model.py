"""Batched, planar screening model for tuning the existing ROS controller.

Identical wheels roll without slip. ROS scheduling, contact, and 3-D motion are
absent: shortlisted gains must also be checked in Gazebo. The averaged coast-
PWM equations match motor_model.py; these are not hardware measurements.
"""

from dataclasses import dataclass, replace
from functools import lru_cache
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ros2_ws/src/balancing_robot_sim/scripts"))
from motor_model import MotorParameters
from simulation.model import RobotParameters

GAIN_NAMES = ("kp", "kd", "position_gain", "velocity_gain")
PARAMETER_NAMES = (*GAIN_NAMES, "total_torque_limit_nm")
BASELINE = np.array([0.2192027027, 0.0124864865, 0.10, 0.18])
# Restricted after the initial wide search's aggressive winner failed Gazebo.
BOUNDS = ((0.18, 0.65), (0.010, 0.050), (0.04, 0.6), (0.10, 0.6))


@lru_cache(maxsize=16)
def motor_arrays(parameters):
    names = (*vars(parameters[0]), "resistance_ohm", "driver_resistance_ohm",
             "back_emf_v_s_rad", "torque_constant_nm_a", "viscous_drag_nm_s_rad")
    return {name: np.array([getattr(p, name) for p in parameters]) for name in names}


@lru_cache(maxsize=16)
def robot_arrays(parameters):
    return tuple(np.array([getattr(p, name) for p in parameters]) for name in
                 ("effective_mass", "coupling", "axle_inertia", "wheel_radius_m", "gravity_m_s2"))


@dataclass(frozen=True)
class Scenario:
    name: str
    tilt_deg: float = 5.0
    motor: MotorParameters = MotorParameters()
    robot: RobotParameters = RobotParameters()
    velocity_m_s: float = 0.0
    rate_deg_s: float = 0.0
    feedback_delay_steps: int = 0
    kick_time_s: float = -1.0
    kick_velocity_m_s: float = 0.0


def training_scenarios():
    p = MotorParameters()
    body_mass = RobotParameters().body_mass_kg
    stress = replace(p, command_deadband=0.08, current_time_constant_s=0.002)
    battery = replace(p, battery_voltage_v=5.5, battery_resistance_ohm=0.5)
    drop = replace(p, driver_drop_at_rating_v=3.6, driver_drop_intercept_v=2.6)
    combined = replace(stress, battery_voltage_v=5.5, battery_resistance_ohm=0.5,
                       driver_drop_at_rating_v=3.6, driver_drop_intercept_v=2.6)
    cases = [Scenario(f"{name}_{tilt:+g}deg", tilt, motor)
             for name, motor in (("nominal", p), ("deadband_lag", stress),
                                 ("lower_battery", battery), ("higher_drop", drop))
             for tilt in (5.0, 10.0)]
    cases += [Scenario("combined_5deg", 5, combined),
              Scenario("combined_10deg", 10, combined),
              Scenario("light_body", robot=replace(RobotParameters(), body_mass_kg=0.8*body_mass)),
              Scenario("heavy_body", robot=replace(RobotParameters(), body_mass_kg=1.2*body_mass)),
              Scenario("slower_motor", motor=replace(p, no_load_speed_rpm=160)),
              Scenario("weaker_motor", motor=replace(p, stall_torque_nm=0.0784532 * 0.8)),
              Scenario("one_sample_delay", feedback_delay_steps=1),
              Scenario("moving_start", tilt_deg=0, velocity_m_s=0.10)]
    return cases


def holdout_scenarios():
    # Kept out of the objective. Includes opposite sign, intermediate conditions,
    # a later velocity impulse, and a longer observation window in the CLI.
    p = MotorParameters()
    return [
        Scenario("negative_7deg", -7),
        Scenario("negative_deadband_10deg", -10,
                 replace(p, command_deadband=0.08, current_time_constant_s=0.002)),
        Scenario("intermediate_battery", 8,
                 replace(p, battery_voltage_v=5.7, battery_resistance_ohm=0.3)),
        Scenario("mass_and_motor", -6, replace(p, no_load_speed_rpm=180),
                 replace(RobotParameters(), body_mass_kg=1.1*RobotParameters().body_mass_kg)),
        Scenario("two_sample_delay", 5, feedback_delay_steps=2),
        Scenario("velocity_kick", 5, kick_time_s=4, kick_velocity_m_s=0.10),
        Scenario("deadband_velocity_kick", 0,
                 replace(p, command_deadband=0.08, current_time_constant_s=0.002),
                 kick_time_s=4, kick_velocity_m_s=-0.10),
        Scenario("initial_pitch_rate", -3, rate_deg_s=15),
    ]


def timing_stress_scenarios():
    """Additional design cases after the first search exposed delay sensitivity."""
    p = MotorParameters()
    return [
        Scenario("two_sample_delay", feedback_delay_steps=2),
        Scenario("three_sample_delay", feedback_delay_steps=3),
        Scenario("delayed_deadband_lag", motor=replace(
            p, command_deadband=0.08, current_time_constant_s=0.002), feedback_delay_steps=2),
        Scenario("delayed_lower_battery_10deg", 10, replace(
            p, battery_voltage_v=5.5, battery_resistance_ohm=0.5), feedback_delay_steps=1),
        Scenario("delayed_combined_5deg", 5, replace(
            p, command_deadband=0.08, current_time_constant_s=0.002,
            battery_voltage_v=5.5, battery_resistance_ohm=0.5,
            driver_drop_at_rating_v=3.6, driver_drop_intercept_v=2.6), feedback_delay_steps=1),
    ]


def timing_holdout_scenarios():
    cases = [c for c in holdout_scenarios() if c.name != "two_sample_delay"]
    return cases + [
        Scenario("four_sample_delay", feedback_delay_steps=4),
        Scenario("delayed_deadband_kick", -7, replace(
            MotorParameters(), command_deadband=0.06, current_time_constant_s=0.003),
            feedback_delay_steps=1, kick_time_s=6, kick_velocity_m_s=0.08),
    ]


def coast_step(request, speed, current, dt, motors):
    """Symmetric two-motor equivalent of MotorDriverModel.step, vectorized.

    request/speed/current have shape (candidate, scenario). Returns per-motor
    torque, current, ON estimate, RMS estimate, shared bus and total driver loss.
    The closed-form shared source solution is checked against the independent
    bisection implementation in motor_model.py.
    """
    field = motor_arrays(tuple(motors)).__getitem__

    resistance = field("resistance_ohm") + field("driver_resistance_ohm")
    intercept = field("driver_drop_intercept_v")
    raw = request / field("command_full_pwm_torque_nm")
    levels = field("pwm_levels")
    duty = np.rint(np.clip(raw, -1, 1) * levels) / levels
    duty = np.where(np.abs(duty) < field("command_deadband"), 0, duty)
    fraction = np.abs(duty)
    sign = np.sign(duty)
    emf = field("back_emf_v_s_rad") * speed
    # Solve V + 2*Rb*d*max(0, (max(0,V-V0)-sign*emf)/R) = Voc.
    a = 2 * field("battery_resistance_ohm") * fraction / resistance
    open_bus = field("battery_voltage_v")
    active_bus = (open_bus + a * (intercept + sign * emf)) / (1 + a)
    below_intercept_bus = open_bus + a * sign * emf
    bus = np.where(active_bus >= intercept, active_bus, below_intercept_bus)
    bus = np.clip(bus, 0, open_bus)
    on = np.where(duty != 0, (sign * np.maximum(0, bus - intercept) - emf) / resistance, 0)
    target_current = fraction * on
    tau = field("current_time_constant_s")
    alpha = -np.expm1(-dt / np.maximum(tau, 1e-100))
    updated = np.where(tau > 0, current + alpha * (target_current - current), target_current)
    rms = np.maximum(np.sqrt(fraction) * np.abs(on), np.where(tau > 0, np.abs(updated), 0))
    on = np.where(tau > 0, np.copysign(np.maximum(np.abs(on), np.abs(updated)),
                                      np.where(on != 0, on, updated)), on)
    quasi_on = np.where(duty != 0, (sign * np.maximum(0, bus - intercept) - emf) / resistance, 0)
    loss = fraction * (intercept * np.abs(quasi_on) + field("driver_resistance_ohm") * quasi_on**2)
    loss = np.maximum(loss, np.where(tau > 0, field("driver_resistance_ohm") * updated**2, 0))
    torque = field("torque_constant_nm_a") * updated - field("viscous_drag_nm_s_rad") * speed
    return torque, updated, on, rms, bus, 2 * loss


def mechanical_derivative(state, torque, robots):
    """Same nonlinear equations as simulation.model.dynamics, in batch form."""
    angle, rate = state[..., 2], state[..., 3]
    mass, coupling, inertia, radius, gravity = robot_arrays(tuple(robots))
    c = coupling * np.cos(angle)
    first = torque / radius + coupling * np.sin(angle) * rate**2
    second = coupling * gravity * np.sin(angle) - torque
    determinant = mass * inertia - c**2
    return np.stack((state[..., 1], (inertia * first - c * second) / determinant,
                     rate, (mass * second - c * first) / determinant), axis=-1)


def score_metrics(m):
    """Dimensionless performance cost plus explicit response/electrical penalties.

    Time averages give equal weight to equal durations. Worst-case aggregation
    happens separately in objective(). Current squared is an effort proxy, not
    battery energy. Thresholds constrain predictions, not hardware compliance.
    """
    performance = (0.6 * m["mean_pitch_squared"] / np.deg2rad(5)**2
                   + 0.25 * m["mean_position_squared"] / 0.10**2
                   + 0.05 * m["mean_velocity_squared"] / 0.20**2
                   + 0.05 * m["mean_current_squared"] / 0.30**2
                   + 0.025 * m["mean_rate_squared"] / np.deg2rad(30)**2
                   + 1.5 * m["tail_mean_pitch_squared"] / np.deg2rad(1)**2
                   + 0.3 * m["tail_mean_rate_squared"] / np.deg2rad(5)**2
                   + 0.5 * m["tail_mean_position_squared"] / 0.05**2
                   + 0.5 * m["tail_mean_velocity_squared"] / 0.03**2
                   + 0.2 * m["settling_time_s"] / m["duration_s"])
    excesses = [
        m["maximum_pitch_deg"] / 20 - 1,
        m["tail_max_pitch_deg"] / 1 - 1,
        m["tail_max_rate_deg_s"] / 5 - 1,
        np.abs(m["final_position_m"]) / 0.1 - 1,
        np.abs(m["final_velocity_m_s"]) / 0.05 - 1,
        m["maximum_current_a"] / 0.6 - 1,
        m["maximum_on_estimate_a"] / 1.2 - 1,
        5.0 / np.maximum(m["minimum_bus_v"], 0.01) - 1,
        m["maximum_steady_junction_estimate_c"] / 150 - 1,
    ]
    penalty = sum(30 * np.maximum(0, e)**2 + 3 * (e > 1e-9) for e in excesses)
    return performance + penalty + 10000 * m["fell"]


def simulate(gains, cases, seconds=10.0, dt=0.005, physics_substeps=1, trace=False,
             tail_seconds=1.0):
    """Screen four gains and an optional request cap with sampled control.

    Control and motor current update every dt; torque is held over RK4 steps.
    Trace is intended for one candidate and includes the final endpoint.
    """
    gains = np.atleast_2d(np.asarray(gains, dtype=float))
    if gains.shape[1] not in (4, 5) or not np.all(np.isfinite(gains)) or np.any(gains < 0):
        raise ValueError("Expected four nonnegative gains, optionally followed by a torque cap")
    caps = gains[:, 4, None] if gains.shape[1] == 5 else np.full((len(gains), 1), 0.060)
    if np.any(caps <= 0) or np.any(caps > 0.060):
        raise ValueError("Torque cap must be positive and at most the original 0.060 N m")
    if not cases or not np.isfinite(seconds) or seconds < 2 or dt <= 0 or not np.isfinite(dt):
        raise ValueError("Need scenarios, >=2 seconds, and a finite positive control step")
    if not np.isfinite(tail_seconds) or not 0 < tail_seconds <= seconds:
        raise ValueError("Scoring tail must be positive and no longer than the run")
    steps = round(seconds / dt)
    if abs(steps * dt - seconds) > 1e-8 or physics_substeps < 1:
        raise ValueError("Duration must be divisible by dt; need >=1 physics substeps")
    if any(c.motor.pwm_off_mode != "coast" or c.feedback_delay_steps < 0 for c in cases):
        raise ValueError("Screening supports coast PWM and nonnegative sample delays")
    shape = (len(gains), len(cases))
    state = np.zeros(shape + (4,))
    state[..., 1] = [c.velocity_m_s for c in cases]
    state[..., 2] = np.deg2rad([c.tilt_deg for c in cases])
    state[..., 3] = np.deg2rad([c.rate_deg_s for c in cases])
    robots, motors = [c.robot for c in cases], [c.motor for c in cases]
    radius = np.array([p.wheel_radius_m for p in robots])
    kp, kd, kx, kv = (gains[:, i, None] for i in range(4))
    current = np.zeros(shape)
    fell = np.zeros(shape, dtype=bool)
    max_state = np.abs(state).copy()
    sums = np.zeros(shape + (5,))  # pitch, x, v, current, pitch rate squared
    tail_sums = sums.copy()
    tail_max = np.zeros(shape + (4,))
    max_current = np.zeros(shape)
    max_on = max_current.copy()
    max_rms = max_current.copy()
    max_temperature = max_current.copy()
    min_bus = np.full(shape, np.inf)
    last_outside = np.full(shape, -dt)
    histories = [state.copy() for _ in range(max(c.feedback_delay_steps for c in cases) + 1)]
    traces = []
    tail_count = 0
    for step in range(steps):
        now = step * dt
        for j, c in enumerate(cases):
            if c.kick_time_s >= 0 and step == round(c.kick_time_s / dt):
                state[:, j, 1] += c.kick_velocity_m_s
        histories.pop(0)
        histories.append(state.copy())
        sensed = np.stack([histories[-1-c.feedback_delay_steps][:, j]
                           for j, c in enumerate(cases)], axis=1)
        target = np.clip(-kx * sensed[..., 0] - kv * sensed[..., 1],
                         -np.deg2rad(5), np.deg2rad(5))
        request = np.clip(kp * (sensed[..., 2] - target) + kd * sensed[..., 3], -caps, caps)
        request = np.where(fell, 0, request)
        torque, current, on, rms, bus, loss = coast_step(
            request / 2, state[..., 1] / radius - state[..., 3], current, dt, motors)
        max_current = np.maximum(max_current, np.abs(current))
        max_on = np.maximum(max_on, np.abs(on))
        max_rms = np.maximum(max_rms, rms)
        min_bus = np.minimum(min_bus, bus)
        temperature = np.array([p.ambient_temperature_c for p in motors]) + loss * np.array(
            [p.driver_thermal_resistance_k_w for p in motors])
        max_temperature = np.maximum(max_temperature, temperature)
        squared = np.stack((state[..., 2]**2, state[..., 0]**2, state[..., 1]**2,
                            current**2, state[..., 3]**2), axis=-1)
        sums += squared
        if now >= seconds - tail_seconds - 1e-9:
            tail_sums += squared
            tail_max = np.maximum(tail_max, np.abs(state))
            tail_count += 1
        outside = ((np.abs(state[..., 2]) > np.deg2rad(1)) |
                   (np.abs(state[..., 3]) > np.deg2rad(5)) |
                   (np.abs(state[..., 0]) > 0.02) | (np.abs(state[..., 1]) > 0.02))
        last_outside = np.where(outside, now, last_outside)
        if trace:
            traces.append(np.concatenate((state, request[..., None], 2*torque[..., None],
                                           current[..., None], bus[..., None]), axis=-1))
        h = dt / physics_substeps
        for _ in range(physics_substeps):
            k1 = mechanical_derivative(state, 2 * torque, robots)
            k2 = mechanical_derivative(state + h*k1/2, 2 * torque, robots)
            k3 = mechanical_derivative(state + h*k2/2, 2 * torque, robots)
            k4 = mechanical_derivative(state + h*k3, 2 * torque, robots)
            proposed = state + h * (k1 + 2*k2 + 2*k3 + k4) / 6
            fallen = (~np.all(np.isfinite(proposed), axis=-1) |
                      (np.abs(proposed[..., 2]) > np.deg2rad(35)) |
                      (np.abs(proposed[..., 0]) > 5) | (np.abs(proposed[..., 1]) > 10))
            fell |= fallen
            state = np.where(fell[..., None], state, proposed)
            max_state = np.maximum(max_state, np.abs(state))
    tail_max = np.maximum(tail_max, np.abs(state))
    # Also score the final endpoint for settling; a late excursion cannot pass.
    final_outside = ((np.abs(state[..., 2]) > np.deg2rad(1)) |
                     (np.abs(state[..., 3]) > np.deg2rad(5)) |
                     (np.abs(state[..., 0]) > 0.02) | (np.abs(state[..., 1]) > 0.02))
    last_outside = np.where(final_outside, seconds, last_outside)
    settling = np.where(last_outside <= seconds - 0.5 - dt, last_outside + dt, seconds)
    metrics = dict(
        mean_pitch_squared=sums[..., 0]/steps, mean_position_squared=sums[..., 1]/steps,
        mean_velocity_squared=sums[..., 2]/steps, mean_current_squared=sums[..., 3]/steps,
        mean_rate_squared=sums[..., 4]/steps,
        tail_mean_pitch_squared=tail_sums[..., 0]/tail_count,
        tail_mean_position_squared=tail_sums[..., 1]/tail_count,
        tail_mean_velocity_squared=tail_sums[..., 2]/tail_count,
        tail_mean_rate_squared=tail_sums[..., 4]/tail_count,
        maximum_pitch_deg=np.rad2deg(max_state[..., 2]),
        maximum_position_m=max_state[..., 0], tail_max_pitch_deg=np.rad2deg(tail_max[..., 2]),
        tail_max_rate_deg_s=np.rad2deg(tail_max[..., 3]), final_position_m=state[..., 0],
        final_velocity_m_s=state[..., 1], maximum_current_a=max_current,
        maximum_on_estimate_a=max_on, maximum_rms_estimate_a=max_rms,
        maximum_steady_junction_estimate_c=max_temperature, minimum_bus_v=min_bus,
        settling_time_s=settling, duration_s=np.full(shape, seconds), fell=fell,
    )
    metrics["cost"] = score_metrics(metrics)
    metrics["response_passed"] = (~fell & (metrics["maximum_pitch_deg"] < 20)
                                  & (metrics["tail_max_pitch_deg"] < 1)
                                  & (metrics["tail_max_rate_deg_s"] < 5)
                                  & (np.abs(state[..., 0]) < 0.1)
                                  & (np.abs(state[..., 1]) < 0.05))
    metrics["electrical_prediction_passed"] = (
        (max_current <= 0.6) & (max_on <= 1.2) & (min_bus >= 5)
        & (max_temperature <= 150))
    if trace:
        final = np.concatenate((state, traces[-1][..., 4:]), axis=-1)
        metrics["trace"] = np.stack(traces + [final])
    return metrics


def objective(gains, cases, seconds=10, **options):
    costs = simulate(gains, cases, seconds=seconds, **options)["cost"]
    return np.mean(costs, axis=1) + 0.75 * np.max(costs, axis=1)


def rows_for(metrics, cases, candidate=0):
    return [{"scenario": case.name, **{
        key: value[candidate, j].item() for key, value in metrics.items() if key != "trace"
    }} for j, case in enumerate(cases)]
