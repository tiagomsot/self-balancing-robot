"""Planar rolling-body dynamics; see docs/stability-analysis.md for derivation."""

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

GEOMETRY_PATH = (Path(__file__).resolve().parents[1] /
                 "ros2_ws/src/balancing_robot_sim/config/robot_geometry.json")
GEOMETRY = json.loads(GEOMETRY_PATH.read_text())


@dataclass(frozen=True)
class RobotParameters:
    body_mass_kg: float = GEOMETRY["body_mass_kg"]
    body_height_m: float = GEOMETRY["body_height_m"]
    body_depth_m: float = GEOMETRY["body_depth_m"]
    body_width_m: float = GEOMETRY["body_width_m"]
    body_bottom_below_axle_m: float = GEOMETRY["body_bottom_below_axle_m"]
    wheel_radius_m: float = GEOMETRY["wheel_radius_m"]
    wheel_width_m: float = GEOMETRY["wheel_width_m"]
    wheel_center_separation_m: float = GEOMETRY["wheel_center_separation_m"]
    wheel_mass_each_kg: float = GEOMETRY["wheel_mass_each_kg"]  # Still assumed.
    imu_above_axle_m: float = GEOMETRY["imu_above_axle_m"]
    gravity_m_s2: float = GEOMETRY["gravity_m_s2"]

    def __post_init__(self):
        if any(not np.isfinite(value) or value <= 0 for name, value in vars(self).items()
               if name != "body_bottom_below_axle_m"):
            raise ValueError("Robot parameters must be finite and positive.")
        if not np.isfinite(self.body_bottom_below_axle_m) or not 0 <= self.body_bottom_below_axle_m < self.body_height_m:
            raise ValueError("The axle must be within the box height.")

    @property
    def com_height_m(self):
        return self.body_height_m / 2 - self.body_bottom_below_axle_m

    @property
    def body_pitch_inertia(self):
        return self.body_mass_kg * (self.body_height_m**2 + self.body_depth_m**2) / 12

    @property
    def effective_mass(self):
        # Two translating wheels + their equivalent rotational inertia / r^2.
        return self.body_mass_kg + 3 * self.wheel_mass_each_kg

    @property
    def coupling(self):
        return self.body_mass_kg * self.com_height_m

    @property
    def axle_inertia(self):
        return self.body_pitch_inertia + self.body_mass_kg * self.com_height_m**2


def dynamics(state, total_torque_Nm, robot):
    """Return derivatives of [axle x, x velocity, forward tilt, tilt rate].

    Positive torque moves both wheels forward; equal opposite reaction on the
    body is included. Sensors and actuators are ideal in this first experiment.
    """
    _, velocity, angle, rate = state
    coupling = robot.coupling
    c = coupling * np.cos(angle)
    mass_matrix = np.array([[robot.effective_mass, c], [c, robot.axle_inertia]])
    rhs = np.array([
        total_torque_Nm / robot.wheel_radius_m + coupling * np.sin(angle) * rate**2,
        coupling * robot.gravity_m_s2 * np.sin(angle) - total_torque_Nm,
    ])
    x_acceleration, angular_acceleration = np.linalg.solve(mass_matrix, rhs)
    return np.array([velocity, x_acceleration, rate, angular_acceleration])


def rk4_step(state, torque, dt, robot):
    """Fourth-order Runge-Kutta step with fixed torque over this physics step."""
    k1 = dynamics(state, torque, robot)
    k2 = dynamics(state + dt * k1 / 2, torque, robot)
    k3 = dynamics(state + dt * k2 / 2, torque, robot)
    k4 = dynamics(state + dt * k3, torque, robot)
    return state + dt * (k1 + 2*k2 + 2*k3 + k4) / 6
