"""Local pitch-stability calculation for the documented robot assumptions.

Run: python analysis/stability_check.py
This is an ideal mechanical/torque model, not an Arduino controller or Gazebo model.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
from scipy.linalg import expm
from scipy.integrate import solve_ivp
from scipy.signal import place_poles

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation.model import RobotParameters


def calculate() -> dict:
    # User dimensions, plus the explicitly provisional wheel masses in components.md.
    robot = RobotParameters()
    mass, height, depth = robot.body_mass_kg, robot.body_height_m, robot.body_depth_m
    radius, wheel_mass = robot.wheel_radius_m, robot.wheel_mass_each_kg
    length = robot.com_height_m
    gravity = robot.gravity_m_s2
    body_inertia = mass * (height**2 + depth**2) / 12
    wheel_inertia = wheel_mass * radius**2 / 2  # Solid cylinder.

    # Includes wheel rotation through the no-slip rolling constraint.
    effective_mass = mass + 2 * wheel_mass + 2 * wheel_inertia / radius**2
    coupling = mass * length
    axle_inertia = body_inertia + mass * length**2
    determinant = effective_mass * axle_inertia - coupling**2
    gravity_moment = mass * gravity * length
    assert determinant > 0

    # x = forward axle displacement; theta = body lean forward from upright.
    # Input tau is the SUM of both gearbox-output torques driving wheels forward.
    # Equal and opposite motor reaction torque on the body is included.
    mass_matrix = np.array([[effective_mass, coupling], [coupling, axle_inertia]])
    acceleration_from_angle = np.linalg.solve(mass_matrix, [0, gravity_moment])
    acceleration_from_torque = np.linalg.solve(mass_matrix, [1 / radius, -1])
    a_pitch = float(acceleration_from_angle[1])
    b_pitch = float(-acceleration_from_torque[1])

    # State order: [x, x_dot, theta, theta_dot].
    A = np.zeros((4, 4))
    A[0, 1] = A[2, 3] = 1
    A[1, 2], A[3, 2] = acceleration_from_angle
    B = np.array([[0], [acceleration_from_torque[0]], [0], [acceleration_from_torque[1]]])
    controllability = np.column_stack([np.linalg.matrix_power(A, k) @ B for k in range(4)])

    def nonlinear(state: np.ndarray, torque: float) -> np.ndarray:
        _, velocity, angle, rate = state
        c = coupling * np.cos(angle)
        mm = np.array([[effective_mass, c], [c, axle_inertia]])
        rhs = [torque / radius + coupling * np.sin(angle) * rate**2,
               gravity_moment * np.sin(angle) - torque]
        xdd, tdd = np.linalg.solve(mm, rhs)
        return np.array([velocity, xdd, rate, tdd])

    # Independent numerical differentiation checks the analytic linearization.
    eps = 1e-6
    zero = np.zeros(4)
    numerical_A = np.column_stack([
        (nonlinear(eps * np.eye(4)[k], 0) - nonlinear(-eps * np.eye(4)[k], 0)) / (2 * eps)
        for k in range(4)
    ])
    numerical_B = ((nonlinear(zero, eps) - nonlinear(zero, -eps)) / (2 * eps)).reshape(4, 1)
    np.testing.assert_allclose(A, numerical_A, rtol=1e-8, atol=1e-8)
    np.testing.assert_allclose(B, numerical_B, rtol=1e-8, atol=1e-8)

    # An illustrative ideal pitch PD law tau = kp*theta + kd*theta_dot.
    # Select pitch poles -8 +/- 8j: theta_ddot + 16*theta_dot + 128*theta = 0.
    kp = (a_pitch + 128) / b_pitch
    kd = 16 / b_pitch
    Ap = np.array([[0, 1], [a_pitch, 0]])
    Bp = np.array([[0], [-b_pitch]])
    Kp = np.array([[kp, kd]])
    closed_pitch = np.linalg.eigvals(Ap + Bp @ Kp)
    assert np.max(closed_pitch.real) < 0
    pitch_feedback = np.array([[0, 0, kp, kd]])

    # Separate full-state example demonstrates stabilizability, not available sensing.
    # Wheel position/speed feedback is hypothetical; hardware encoders are unconfirmed.
    full_gain = place_poles(A, B, [-2, -3, -8 + 8j, -8 - 8j]).gain_matrix
    closed_full = np.linalg.eigvals(A - B @ full_gain)
    assert np.max(closed_full.real) < 0

    # Exact ZOH discretization of pitch plant at 200 Hz; delayed perfect state samples.
    # This is a pure-delay experiment, not an exact MPU digital-filter model.
    dt = 0.005
    aug = np.zeros((3, 3))
    aug[:2, :2], aug[:2, 2:] = Ap, Bp
    discrete = expm(aug * dt)
    Ad, Bd = discrete[:2, :2], discrete[:2, 2:]
    delay_results = []
    for delay_steps in (0, 1, 2, 4, 8, 12):
        size = 2 * (delay_steps + 1)
        delayed = np.zeros((size, size))
        delayed[:2, :2] = Ad
        delayed[:2, -2:] += Bd @ Kp
        for k in range(1, delay_steps + 1):
            delayed[2*k:2*k+2, 2*k-2:2*k] = np.eye(2)
        spectral_radius = float(np.max(np.abs(np.linalg.eigvals(delayed))))
        delay_results.append({"delay_ms": delay_steps * dt * 1000,
                              "max_pole_magnitude": spectral_radius,
                              "pitch_stable": spectral_radius < 1})

    # Illustrative static-current/torque comparison. This omits friction and dynamics.
    # Not a claim that the L293D automatically limits current.
    stall_torque_each = 0.8 * 9.80665 / 100  # kgf cm -> N m.
    stall_current_each = 1.5
    effective_kt = stall_torque_each / stall_current_each
    torque_at_current_rating = 2 * effective_kt * 0.6
    initial_requests = []
    for deg in (1, 2, 5, 10):
        total_torque = kp * np.deg2rad(deg)
        initial_requests.append({
            "tilt_deg": deg, "initial_rate_rad_s": 0,
            "total_torque_Nm": float(total_torque),
            "per_motor_torque_Nm": float(total_torque / 2),
            "ideal_current_each_A": float(total_torque / (2 * effective_kt)),
        })

    # Nonlinear mechanics with the ideal pitch controller: check simultaneous
    # torque and speed requests, and expose translational drift explicitly.
    # The budget is a reference 6 V linear torque-speed approximation, not L293D output.
    no_load_speed = 200 * 2 * np.pi / 60
    response_checks = []
    for deg in (5, 10):
        response = solve_ivp(
            lambda t, z: nonlinear(z, kp * z[2] + kd * z[3]),
            (0, 3), [0, 0, np.deg2rad(deg), 0],
            t_eval=np.linspace(0, 3, 3001), rtol=1e-10, atol=1e-12,
        )
        assert response.success
        xx, vv, tt, ww = response.y
        commanded_torque_each = (kp * tt + kd * ww) / 2
        relative_output_speed = vv / radius - ww
        # Sufficient envelope check for magnitude-limited motoring in either direction;
        # deliberately conservative in braking quadrants, ignoring friction/inductance.
        voltage_fraction = (np.abs(commanded_torque_each) / stall_torque_each
                            + np.abs(relative_output_speed) / no_load_speed)
        response_checks.append({
            "initial_tilt_deg": deg,
            "duration_s": 3,
            "peak_torque_each_Nm": float(np.max(np.abs(commanded_torque_each))),
            "peak_gearbox_relative_speed_rpm": float(np.max(np.abs(relative_output_speed)) * 60 / (2*np.pi)),
            "peak_ideal_torque_current_each_A": float(np.max(np.abs(commanded_torque_each)) / effective_kt),
            "peak_reference_6V_torque_speed_budget_fraction": float(np.max(voltage_fraction)),
            "final_tilt_deg": float(np.rad2deg(tt[-1])),
            "final_axle_displacement_m": float(xx[-1]),
            "final_axle_speed_m_s": float(vv[-1]),
        })

    def poles(values: np.ndarray) -> list[dict]:
        return [{"real": float(v.real), "imag": float(v.imag)} for v in values]

    # Variation of provisional wheel mass, keeping nominal pitch gains fixed.
    mass_sensitivity = []
    for mw in (0.005, 0.010, 0.020, 0.050):
        M = mass + 3 * mw  # Two solid wheels: 2*mw translation + mw rotational equivalent.
        dd = M * axle_inertia - coupling**2
        aa = M * gravity_moment / dd
        bb = (M + coupling / radius) / dd
        ev = np.linalg.eigvals([[0, 1], [aa - bb * kp, -bb * kd]])
        mass_sensitivity.append({"wheel_mass_each_kg": mw,
                                 "pitch_poles": poles(ev),
                                 "pitch_stable": bool(np.max(ev.real) < 0)})

    return {
        "scope": "Ideal planar no-slip torque-input mechanics, linearized near upright",
        "body_mass_kg": mass, "wheel_mass_each_kg_assumed": wheel_mass,
        "com_above_axle_m_assumed": length,
        "body_bottom_below_axle_m": robot.body_bottom_below_axle_m,
        "body_pitch_inertia_kg_m2": body_inertia,
        "body_inertia_about_axle_kg_m2": axle_inertia,
        "effective_translational_mass_kg": effective_mass,
        "A": A.tolist(), "B": B.tolist(),
        "linearization_verified_by_numerical_differentiation": True,
        "full_controllability_rank": int(np.linalg.matrix_rank(controllability)),
        "open_loop_poles_per_s": poles(np.linalg.eigvals(A)),
        "unstable_mode_efolding_time_s": float(1 / np.sqrt(a_pitch)),
        "pitch_equation": {"angle_coefficient_per_s2": a_pitch,
                           "negative_total_torque_coefficient": b_pitch},
        "ideal_pitch_feedback": {"kp_Nm_per_rad": kp, "kd_Nm_s_per_rad": kd,
                                 "pitch_poles_per_s": poles(closed_pitch),
                                 "full_model_poles_per_s": poles(np.linalg.eigvals(A + B @ pitch_feedback))},
        "hypothetical_full_state_feedback": {"law": "tau = -K @ [x, x_dot, theta, theta_dot]",
                                              "K": full_gain.tolist(),
                                              "closed_loop_poles_per_s": poles(closed_full)},
        "sampled_pitch_delay_sensitivity_at_200Hz": delay_results,
        "wheel_mass_sensitivity_with_fixed_pitch_gains": mass_sensitivity,
        "effective_torque_constant_Nm_per_A_assumed": effective_kt,
        "total_torque_at_0_6A_each_Nm_illustrative": torque_at_current_rating,
        "initial_PD_requests": initial_requests,
        "nonlinear_ideal_pitch_response_checks": response_checks,
        "limitations": [
            "Instantaneous delivered torque; electrical/mechanical motor dynamics omitted",
            "No speed-dependent available torque, driver voltage model, or battery sag",
            "No backlash, friction/deadband, noise, or sensor-fusion errors",
            "Pitch-only feedback leaves translational modes uncontrolled",
            "Full-state feedback presumes unavailable/unconfirmed wheel state measurements",
            "Current calculation uses reference motor data, neglects friction, and is not a driver model",
            "Delay study uses perfect delayed pitch/rate samples and no saturation",
            "Planar analysis excludes yaw, lateral tipping, and loss of contact",
        ],
    }


if __name__ == "__main__":
    result = calculate()
    output = Path(__file__).with_name("stability-results.json")
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
