"""Physics and experiment checks; run with python -m unittest discover -s tests -v."""

import csv
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simulation.model import RobotParameters, dynamics
from simulation.runner import Experiment, save_data, simulate
from gpu.robot_model import model_xml
import xml.etree.ElementTree as ET


class PhysicsTests(unittest.TestCase):
    def test_measured_axle_offset_sets_uniform_com_and_ground_clearance(self):
        robot = RobotParameters()
        self.assertAlmostEqual(robot.com_height_m, 0.048)
        self.assertAlmostEqual(robot.wheel_radius_m-robot.body_bottom_below_axle_m, 0.013)
        self.assertAlmostEqual(robot.body_pitch_inertia, 0.000830375)
        self.assertAlmostEqual(robot.body_mass_kg+2*robot.wheel_mass_each_kg, 0.460)

    def test_gpu_template_uses_the_same_measured_geometry(self):
        root = ET.fromstring(model_xml())
        body = root.find("worldbody/body[@name='base']")
        box = body.find("geom[@name='body']")
        np.testing.assert_allclose(np.fromstring(box.get("size"), sep=" ")*2, [.035, .10, .15])
        self.assertAlmostEqual(float(box.get("mass")), .420)
        self.assertAlmostEqual(float(box.get("pos").split()[2]), .048)
        left, right = [body.find(f"body[@name='{side}_wheel']") for side in ("left", "right")]
        self.assertAlmostEqual(float(left.get("pos").split()[1])-float(right.get("pos").split()[1]), .168)
        np.testing.assert_allclose(np.fromstring(left.find("geom").get("size"), sep=" "), [.04, .004])
        self.assertAlmostEqual(float(body.find("site[@name='imu']").get("pos").split()[2]), .06)

    def test_mechanical_power_matches_motor_work(self):
        """Independent work/energy identity catches sign and reaction-torque errors."""
        robot = RobotParameters()
        state = np.array([0.1, 0.2, 0.15, -0.4])
        _, v, angle, rate = state
        torque = 0.012
        C, M, J = robot.coupling, robot.effective_mass, robot.axle_inertia
        # E = .5*M*v^2 + C*cos(angle)*v*rate + .5*J*rate^2 + m*g*l*cos(angle).
        energy_gradient = np.array([
            0,
            M*v + C*np.cos(angle)*rate,
            -C*np.sin(angle)*v*rate - C*robot.gravity_m_s2*np.sin(angle),
            C*np.cos(angle)*v + J*rate,
        ])
        energy_rate = energy_gradient @ dynamics(state, torque, robot)
        input_power = torque * (v/robot.wheel_radius_m - rate)
        self.assertAlmostEqual(energy_rate, input_power, places=12)

    def test_upright_at_rest_is_an_equilibrium(self):
        result = simulate(Experiment(controller="off", initial_tilt_deg=0, duration_s=0.1))
        np.testing.assert_array_equal(result.state, np.zeros_like(result.state))

    def test_small_angle_model_matches_existing_analysis(self):
        """Keep the framework consistent with the earlier independent calculation."""
        saved = json.loads((Path(__file__).parents[1] / "analysis" / "stability-results.json").read_text())
        robot = RobotParameters()
        epsilon = 1e-6
        A = np.column_stack([
            (dynamics(epsilon*np.eye(4)[i], 0, robot) - dynamics(-epsilon*np.eye(4)[i], 0, robot)) / (2*epsilon)
            for i in range(4)
        ])
        B = (dynamics(np.zeros(4), epsilon, robot) - dynamics(np.zeros(4), -epsilon, robot)) / (2*epsilon)
        np.testing.assert_allclose(A, saved["A"], atol=1e-8)
        np.testing.assert_allclose(B, np.array(saved["B"]).ravel(), atol=1e-8)


class ExperimentTests(unittest.TestCase):
    def test_unpowered_robot_falls(self):
        result = simulate(Experiment(controller="off", duration_s=1))
        self.assertTrue(result.fell)
        self.assertGreaterEqual(abs(np.rad2deg(result.state[-1, 2])), 35)
        self.assertIsNone(result.metrics()["pitch_settling_time_s"])

    def test_reference_balances_pitch_but_does_not_hold_position(self):
        result = simulate(Experiment(duration_s=2))
        self.assertFalse(result.fell)
        self.assertLess(abs(np.rad2deg(result.state[-1, 2])), 0.01)
        self.assertIsNotNone(result.metrics()["pitch_settling_time_s"])
        self.assertGreater(abs(result.state[-1, 1]), 0.01)

    def test_weak_actuator_does_not_silently_deliver_requested_torque(self):
        result = simulate(Experiment(duration_s=1, total_torque_limit_Nm=0.001))
        self.assertTrue(result.fell)
        self.assertGreater(result.metrics()["saturated_control_fraction"], 0)
        self.assertGreater(np.max(np.abs(result.requested_torque_Nm)), 0.001)
        self.assertLessEqual(np.max(np.abs(result.applied_torque_Nm)), 0.001)

    def test_mirrored_leans_produce_mirrored_motion(self):
        positive = simulate(Experiment(duration_s=0.5, initial_tilt_deg=5))
        negative = simulate(Experiment(duration_s=0.5, initial_tilt_deg=-5))
        np.testing.assert_allclose(positive.state, -negative.state, atol=1e-12)

    def test_halving_physics_step_preserves_result(self):
        settings = Experiment(duration_s=1)
        original = simulate(settings)
        refined = simulate(replace(settings, physics_step_s=0.0005))
        np.testing.assert_allclose(original.state, refined.state, atol=1e-8, rtol=1e-7)

    def test_bad_parameters_fail_before_simulating(self):
        for changes in ({"kp": float("nan")}, {"duration_s": 0},
                        {"physics_step_s": 0.002}, {"initial_tilt_deg": 35},
                        {"total_torque_limit_Nm": -1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                Experiment(**changes)

    def test_export_preserves_settings_and_trajectory(self):
        result = simulate(Experiment(duration_s=0.1))
        with tempfile.TemporaryDirectory() as directory:
            save_data(result, directory)
            summary = json.loads((Path(directory) / "summary.json").read_text())
            with (Path(directory) / "trajectory.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), len(result.time_s))
            self.assertEqual(summary["experiment"]["controller"], "reference")
            self.assertAlmostEqual(float(rows[-1]["tilt_rad"]), result.state[-1, 2])
            self.assertIn("No driver voltage/current model", summary["limitations"])


if __name__ == "__main__":
    unittest.main()
