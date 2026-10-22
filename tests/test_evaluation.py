"""Known-signal and saved-file checks for post-processing."""

from dataclasses import replace
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from analysis.postprocessing import comparison_notes, load_run, save_report
from simulation.evaluation import evaluate, significant_crossings
from simulation.runner import Experiment, save_data, simulate


class MetricTests(unittest.TestCase):
    def test_constant_error_has_known_rms_and_integral(self):
        time = np.array([0., 0.5, 2.])
        state = np.zeros((3, 4))
        state[:, 2] = np.deg2rad(2)
        m = evaluate(time, state, np.zeros(3), np.zeros(3), False, 0.05)
        self.assertAlmostEqual(m["rms_tilt_deg"], 2)
        self.assertAlmostEqual(m["integral_absolute_tilt_deg_s"], 4)
        self.assertEqual(m["opposite_side_overshoot_deg"], 0)
        self.assertIsNone(m["pitch_settling_time_s"])

    def test_negative_initial_lean_has_correct_overshoot_direction(self):
        state = np.zeros((4, 4))
        state[:, 2] = np.deg2rad([-5, -1, 2, 0])
        m = evaluate([0, 1, 2, 3], state, np.zeros(4), np.zeros(4), False, 0.05)
        self.assertAlmostEqual(m["opposite_side_overshoot_deg"], 2)
        self.assertAlmostEqual(m["overshoot_percent_of_initial_tilt"], 40)

    def test_zero_initial_lean_does_not_divide_by_zero(self):
        m = evaluate([0, 1], np.zeros((2, 4)), np.zeros(2), np.zeros(2), False, 0.05)
        self.assertIsNone(m["opposite_side_overshoot_deg"])
        self.assertIsNone(m["overshoot_percent_of_initial_tilt"])
        self.assertEqual(m["rms_tilt_deg"], 0)

    def test_torque_metrics_use_interval_duration_and_exclude_terminal_command(self):
        m = evaluate([0, 1, 4], np.zeros((3, 4)), [2, 0, 999], [1, 0, 999], False, 0.05)
        self.assertAlmostEqual(m["saturated_control_fraction"], 0.25)
        self.assertAlmostEqual(m["saturated_duration_s"], 1)
        self.assertAlmostEqual(m["total_torque_squared_effort_Nm2_s"], 1)

    def test_settling_requires_sustained_angle_and_rate_band(self):
        state = np.zeros((5, 4))
        state[:, 2] = np.deg2rad([5, 0.5, 0.2, 0.1, 0])
        state[1, 3] = np.deg2rad(6)  # Angle inside band, rate still outside.
        m = evaluate([0, 0.5, 1, 1.5, 2], state, np.zeros(5), np.zeros(5), False, 0.05)
        self.assertEqual(m["pitch_settling_time_s"], 1)
        short = evaluate([0, 0.1, 0.2], state[:3], np.zeros(3), np.zeros(3), False, 0.05)
        self.assertIsNone(short["pitch_settling_time_s"])

    def test_crossing_counter_ignores_tiny_sign_changes(self):
        self.assertEqual(significant_crossings([5, 0.01, -0.02, 0.03, -0.01]), 0)
        self.assertEqual(significant_crossings([5, 0.01, -2, -0.01, 1]), 2)

    def test_rejects_nonincreasing_time_or_nonfinite_recordings(self):
        with self.assertRaises(ValueError):
            evaluate([0, 0], np.zeros((2, 4)), np.zeros(2), np.zeros(2), False, 0.05)
        with self.assertRaises(ValueError):
            evaluate([0, 1], np.full((2, 4), np.nan), np.zeros(2), np.zeros(2), False, 0.05)


class SavedRunTests(unittest.TestCase):
    def test_legacy_run_retains_original_axle_location(self):
        with tempfile.TemporaryDirectory() as directory:
            save_data(simulate(Experiment(duration_s=.1)), directory)
            path = Path(directory)/"summary.json"
            summary = json.loads(path.read_text())
            summary["robot"].pop("body_bottom_below_axle_m")
            summary["robot"].pop("wheel_center_separation_m")
            summary["robot"]["body_height_m"] = .25
            path.write_text(json.dumps(summary))
            loaded = load_run(directory)
            self.assertAlmostEqual(loaded.result.robot.com_height_m, .125)
            self.assertAlmostEqual(loaded.result.robot.wheel_center_separation_m, .125)

    def test_loader_rejects_mismatched_summary_and_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            save_data(simulate(Experiment(duration_s=0.1)), directory)
            path = Path(directory) / "summary.json"
            summary = json.loads(path.read_text())
            summary["experiment"]["initial_tilt_deg"] = 10
            path.write_text(json.dumps(summary))
            with self.assertRaisesRegex(ValueError, "starting angle"):
                load_run(directory)

    def test_comparison_flags_changed_conditions_and_two_changed_gains(self):
        with tempfile.TemporaryDirectory() as directory:
            first, second = Path(directory)/"first", Path(directory)/"second"
            baseline = Experiment(duration_s=0.1)
            changed = replace(baseline, kp=0.3, kd=0.01, total_torque_limit_Nm=0.005)
            save_data(simulate(baseline), first)
            save_data(simulate(changed), second)
            notes = " ".join(comparison_notes([load_run(first), load_run(second)]))
            self.assertIn("total_torque_limit_Nm", notes)
            self.assertIn("Both P and D differ", notes)

    def test_report_preserves_inputs_and_records_their_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            source, report_dir = Path(directory)/"source", Path(directory)/"report"
            save_data(simulate(Experiment(duration_s=0.1)), source)
            original = (source/"trajectory.csv").read_bytes()
            run = load_run(source)
            save_report([run], report_dir)
            metrics = json.loads((report_dir/"metrics.json").read_text())
            self.assertEqual((source/"trajectory.csv").read_bytes(), original)
            self.assertEqual((report_dir/"input-data"/"run-1-trajectory.csv").read_bytes(), original)
            self.assertEqual(metrics["runs"][0]["csv_sha256"], hashlib.sha256(original).hexdigest())
            self.assertTrue((report_dir/"comparison.png").is_file())


if __name__ == "__main__":
    unittest.main()
