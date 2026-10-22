"""Sampled controller, held actuator command, physics integration, and metrics."""

import csv
from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np

from .controllers import REFERENCE_KD, REFERENCE_KP, make_controller
from .evaluation import evaluate
from .model import RobotParameters, rk4_step


@dataclass(frozen=True)
class Experiment:
    controller: str = "reference"
    kp: float = REFERENCE_KP
    kd: float = REFERENCE_KD
    initial_tilt_deg: float = 5.0
    duration_s: float = 5.0
    control_period_s: float = 0.005
    physics_step_s: float = 0.001
    total_torque_limit_Nm: float = 0.060  # Explicit scenario limit, not L293D regulation.
    fall_angle_deg: float = 35.0  # Terminate experiment; no floor-impact model.

    def __post_init__(self):
        numeric = [value for value in vars(self).values() if not isinstance(value, str)]
        if not all(np.isfinite(value) for value in numeric):
            raise ValueError("Experiment settings must be finite numbers.")
        if self.kp < 0 or self.kd < 0:
            raise ValueError("Use nonnegative P and D gains for the documented signs.")
        if not 0 < self.duration_s <= 60:
            raise ValueError("Duration must be greater than 0 and no more than 60 s.")
        if self.control_period_s <= 0 or self.physics_step_s <= 0:
            raise ValueError("Control period and physics step must be positive.")
        if self.total_torque_limit_Nm <= 0:
            raise ValueError("Torque limit must be positive.")
        if not 0 < self.fall_angle_deg < 90:
            raise ValueError("Fall cutoff must be between 0 and 90 degrees.")
        if abs(self.initial_tilt_deg) >= self.fall_angle_deg:
            raise ValueError("Starting tilt must be smaller than the fall cutoff.")
        for ratio in (self.duration_s / self.control_period_s,
                      self.control_period_s / self.physics_step_s):
            if ratio < 1 or not np.isclose(ratio, round(ratio), rtol=0, atol=1e-8):
                raise ValueError("Duration/control period and control/physics step must be integers.")


@dataclass
class Result:
    robot: RobotParameters
    experiment: Experiment
    time_s: np.ndarray
    state: np.ndarray
    requested_torque_Nm: np.ndarray
    applied_torque_Nm: np.ndarray
    fell: bool

    def metrics(self):
        return evaluate(self.time_s, self.state, self.requested_torque_Nm,
                        self.applied_torque_Nm, self.fell, self.robot.wheel_radius_m)


def simulate(experiment=None, robot=None):
    experiment = experiment or Experiment()
    robot = robot or RobotParameters()
    controller = make_controller(experiment.controller, experiment.kp, experiment.kd)
    periods = round(experiment.duration_s / experiment.control_period_s)
    substeps = round(experiment.control_period_s / experiment.physics_step_s)
    state = np.array([0.0, 0.0, np.deg2rad(experiment.initial_tilt_deg), 0.0])
    times, states, requests, torques = [], [], [], []
    fell = False
    for i in range(periods + 1):
        request = float(controller(state[2], state[3]))
        if not np.isfinite(request):
            raise ValueError("Controller returned a non-finite torque. Return a finite number in N m.")
        torque = float(np.clip(request, -experiment.total_torque_limit_Nm, experiment.total_torque_limit_Nm))
        times.append(i * experiment.control_period_s)
        states.append(state.copy())
        requests.append(request)
        torques.append(torque)
        fell = abs(np.rad2deg(state[2])) >= experiment.fall_angle_deg
        if fell or i == periods:
            break
        for _ in range(substeps):
            state = rk4_step(state, torque, experiment.physics_step_s, robot)
        if not np.all(np.isfinite(state)):
            raise ValueError("Physics became non-finite; check the experiment settings.")
    return Result(robot, experiment, np.array(times), np.array(states),
                  np.array(requests), np.array(torques), fell)


def save_data(result, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "axle_position_m", "axle_speed_m_s", "tilt_rad",
                         "pitch_rate_rad_s", "requested_total_torque_Nm", "applied_total_torque_Nm"])
        writer.writerows(np.column_stack([result.time_s, result.state,
                                         result.requested_torque_Nm, result.applied_torque_Nm]))
    summary = {
        "model": "Ideal planar rigid body; no slip; perfect pitch/rate; bounded instantaneous torque",
        "limitations": ["No driver voltage/current model", "No motor speed/response model",
                        "No backlash/deadband or IMU noise/delay", "No speed/position or yaw controller",
                        "Torque cap is an imposed scenario limit, not L293D current regulation"],
        "robot": asdict(result.robot), "experiment": asdict(result.experiment),
        "metrics": result.metrics(),
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
