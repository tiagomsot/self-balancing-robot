"""Independent checks for the accelerated optimizer's plant and scoring."""

from dataclasses import replace
import ast
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from analysis.tuning_model import (BASELINE, Scenario, coast_step, mechanical_derivative,
                                   simulate)
from analysis.tune_controller import write_profile
from motor_model import MotorDriverModel, MotorParameters
from simulation.model import RobotParameters, dynamics


class TuningModelTests(unittest.TestCase):
    def test_symmetric_fast_motor_matches_original_bisection_and_lag(self):
        rng = np.random.default_rng(401)
        for lag in (0, 0.002, 0.02):
            p = replace(MotorParameters(), battery_resistance_ohm=0.5,
                        current_time_constant_s=lag, command_deadband=0.08)
            original = MotorDriverModel(p)
            fast_current = np.zeros((1, 1))
            for _ in range(70):
                request = rng.uniform(-0.05, 0.05)
                speed = rng.uniform(-50, 50)
                expected = original.step([request]*2, [speed]*2, 0.005)
                actual = coast_step(np.array([[request]]), np.array([[speed]]),
                                    fast_current, 0.005, [p])
                fast_current = actual[1]
                for got, wanted in zip(actual, (expected.left.torque_nm,
                        expected.left.current_a, expected.left.on_current_estimate_a,
                        expected.left.rms_current_estimate_a, expected.bus_voltage_v,
                        expected.driver_loss_w)):
                    self.assertAlmostEqual(float(got[0, 0]), wanted, places=8)

    def test_fast_mechanics_match_independent_mass_matrix_solution(self):
        rng = np.random.default_rng(42)
        robots = [RobotParameters(), replace(RobotParameters(), body_mass_kg=0.36)]
        state = rng.uniform(-0.2, 0.2, (3, 2, 4))
        torque = rng.uniform(-0.06, 0.06, (3, 2))
        actual = mechanical_derivative(state, torque, robots)
        for i in range(3):
            for j in range(2):
                np.testing.assert_allclose(actual[i, j], dynamics(state[i, j], torque[i, j], robots[j]))

    def test_mirrored_starts_have_mirrored_trajectories(self):
        result = simulate(BASELINE, [Scenario("positive"), Scenario("negative", -5)],
                          seconds=2, trace=True)
        np.testing.assert_allclose(result["trace"][:, 0, 0, :7],
                                   -result["trace"][:, 0, 1, :7], atol=1e-10)

    def test_zero_control_is_penalized_as_a_fall(self):
        result = simulate([0, 0, 0, 0], [Scenario("unpowered")], seconds=2)
        self.assertTrue(result["fell"][0, 0])
        self.assertFalse(result["response_passed"][0, 0])
        self.assertGreater(result["cost"][0, 0], 10000)

    def test_finer_physics_steps_agree_with_control_step_held_fixed(self):
        one = simulate(BASELINE, [Scenario("nominal")], seconds=2)
        five = simulate(BASELINE, [Scenario("nominal")], seconds=2, physics_substeps=5)
        self.assertLess(abs(one["final_position_m"][0, 0] - five["final_position_m"][0, 0]), 0.002)

    def test_optional_cap_bounds_requests_and_cannot_enlarge_the_original_limit(self):
        result = simulate([*BASELINE, 0.01], [Scenario("limited")], seconds=2, trace=True)
        self.assertLessEqual(np.max(np.abs(result["trace"][..., 4])), 0.01 + 1e-12)
        with self.assertRaises(ValueError):
            simulate([*BASELINE, 0.061], [Scenario("invalid")], seconds=2)

    def test_export_keeps_integral_valued_gains_as_ros_double_parameters(self):
        with TemporaryDirectory() as temporary:
            path = Path(temporary) / "profile.yaml"
            write_profile(path, [1.0, 0.1, 0.15, 0.35, 0.04])
            values = {line.strip().split(": ")[0]: ast.literal_eval(line.split(": ")[1])
                      for line in path.read_text().splitlines() if line.startswith("    ")}
        self.assertIs(type(values["kp"]), float)
        self.assertEqual(values["total_torque_limit_nm"], 0.04)


if __name__ == "__main__":
    unittest.main()
