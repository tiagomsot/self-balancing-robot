"""Check that validated optimization changes only gains and preserves a backup."""
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import yaml

from analysis.apply_optimized_gains import (ACTIVE, BALANCE_CHECKS, CASES, GEOMETRY, NATIVE, GAZEBO_NATIVE,
    UpdateRejected, apply_run, digest)


class OptimizationUpdateTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.folder = self.root / "outputs/search"
        self.folder.mkdir(parents=True)
        self.active = self.root / ACTIVE
        self.active.parent.mkdir(parents=True)
        self.before = ("balance_controller:\r\n  ros__parameters:\r\n"
                       "    kp: 1.0  # N m / rad\r\n    kd: 0.1  # damping\r\n    ki: 0.0\r\n"
                       "    position_gain: 0.0\r\n    velocity_gain: 0.0\r\n"
                       "    wheel_feedback_enabled: false\r\n"
                       "    total_torque_limit_nm: 0.04\r\n    max_speed_m_s: 0.25\r\n"
                       "shuttle_demo:\r\n  ros__parameters:\r\n    settle_time_s: 2.0\r\n"
                       "motor_driver:\r\n  ros__parameters:\r\n    battery_voltage_v: 6.0\r\n")
        self.active.write_bytes(self.before.encode())
        self.baseline = yaml.safe_load(self.before)
        self.candidate = copy.deepcopy(self.baseline)
        self.gains = dict(kp=.65, kd=.05, ki=.3)
        self.candidate["balance_controller"]["ros__parameters"].update(self.gains)
        self.save_yaml(self.folder / "current.yaml", self.baseline)
        self.save_yaml(self.folder / "candidate.yaml", self.candidate)
        for relative in (GEOMETRY, NATIVE, Path("cpp/src/controller.cpp"), Path("cpp/src/RobotCore.h")):
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"test source")
        sources = {p.as_posix(): digest(self.root / p) for p in
                   (ACTIVE, GEOMETRY, NATIVE, Path("cpp/src/controller.cpp"), Path("cpp/src/RobotCore.h"))}
        self.search = dict(selected=dict(all_training_cases_pass=True,
                           gains=[.65, .05, 0, 0, .04, .3], objective=.5), feedback_mode="imu_only",
                           baseline=dict(objective=1), candidate_holdout=dict(passed=True),
                           source_sha256=sources)
        self.save_search()
        gazebo_library = self.root / GAZEBO_NATIVE
        gazebo_library.parent.mkdir(parents=True)
        gazebo_library.write_bytes(b"independently built ROS library")
        adapters = {"controller":"balance_controller", "native_binding":"shared_core.py"}
        for name in adapters.values(): (gazebo_library.parent/name).write_bytes(b"IMU-only adapter")
        records = []
        for case in CASES:
            directory = self.folder / "gazebo-validation/candidate" / case
            directory.mkdir(parents=True)
            records.append(dict(profile="candidate", scenario=case, returncode=0,
                                response_passed=True, electrical_prediction_passed=True))
            summary = dict(passed=True, requested_duration_s=12, feedback_mode="imu_only",wheel_feedback_used=False,
                           checks={key: True for key in BALANCE_CHECKS},
                           source_sha256={"geometry": sources[GEOMETRY.as_posix()],
                             "native_library": digest(gazebo_library),
                             **{k:digest(gazebo_library.parent/v) for k,v in adapters.items()},
                             **{k:v for k,v in sources.items() if k.startswith("cpp/src/")}})
            (directory / "summary.json").write_text(json.dumps(summary))
            self.save_yaml(directory / "parameters.yaml", self.candidate)
        self.validation = self.folder / "gazebo-validation/validation.json"
        self.validation.write_text(json.dumps(dict(runs=records)))

    @staticmethod
    def save_yaml(path, value):
        path.write_text(yaml.safe_dump(value))

    def save_search(self):
        (self.folder / "search.json").write_text(json.dumps(self.search))

    def apply(self):
        return apply_run(self.folder, self.root, reuse_validation=True)

    def assert_kept(self):
        self.assertEqual(self.active.read_bytes(), self.before.encode())
        self.assertFalse((self.folder / "previous-active.yaml").exists())

    def test_update_preserves_other_settings_comments_endings_and_backup(self):
        result = self.apply()
        self.assertEqual(result["status"], "applied")
        self.assertEqual(yaml.safe_load(self.active.read_text()), self.candidate)
        self.assertEqual((self.folder / "previous-active.yaml").read_bytes(), self.before.encode())
        content = self.active.read_bytes()
        self.assertIn(b"# N m / rad", content)
        self.assertIn(b"max_speed_m_s: 0.25\r\n", content)
        self.assertNotIn(b"\n", content.replace(b"\r\n", b""))
        self.assertTrue(json.loads((self.folder / "promotion.json").read_text())["active_config_modified"])
        self.assertEqual(self.apply()["status"], "already_active")

    def test_failed_holdout_or_worse_score_keeps_active_gains(self):
        self.search["candidate_holdout"]["passed"] = False
        self.save_search()
        with self.assertRaises(UpdateRejected): self.apply()
        self.search["candidate_holdout"]["passed"] = True
        self.search["selected"]["objective"] = 2
        self.save_search()
        with self.assertRaises(UpdateRejected): self.apply()
        self.assert_kept()

    def test_failed_or_missing_gazebo_case_keeps_active_gains(self):
        data = json.loads(self.validation.read_text())
        data["runs"][0]["response_passed"] = False
        self.validation.write_text(json.dumps(data))
        with self.assertRaises(UpdateRejected): self.apply()
        data["runs"].pop(0)
        self.validation.write_text(json.dumps(data))
        with self.assertRaises(UpdateRejected): self.apply()
        self.assert_kept()

    def test_validation_of_different_gains_is_rejected(self):
        wrong = copy.deepcopy(self.candidate)
        wrong["balance_controller"]["ros__parameters"]["kp"] = .8
        self.save_yaml(self.folder / "gazebo-validation/candidate" / CASES[0] / "parameters.yaml", wrong)
        with self.assertRaises(UpdateRejected): self.apply()
        self.assert_kept()

    def test_stale_control_source_is_rejected(self):
        (self.root / "cpp/src/controller.cpp").write_text("changed")
        with self.assertRaises(UpdateRejected): self.apply()
        self.assert_kept()

    def test_stale_gazebo_binary_is_rejected(self):
        (self.root / GAZEBO_NATIVE).write_bytes(b"new build")
        with self.assertRaises(UpdateRejected): self.apply()
        self.assert_kept()

    def test_non_gain_settings_cannot_be_overwritten(self):
        wrong = copy.deepcopy(self.candidate)
        wrong["motor_driver"]["ros__parameters"]["battery_voltage_v"] = 7
        self.save_yaml(self.folder / "candidate.yaml", wrong)
        with self.assertRaises(UpdateRejected): self.apply()
        self.assert_kept()

    def test_edit_during_validation_is_preserved(self):
        def edit(*args): self.active.write_text(self.before + "# manual change\n")
        with patch("analysis.apply_optimized_gains.validate_in_gazebo", side_effect=edit):
            with self.assertRaises(UpdateRejected): apply_run(self.folder, self.root)
        self.assertIn("# manual change", self.active.read_text())
        self.assertFalse((self.folder / "previous-active.yaml").exists())

    def test_validation_process_error_cannot_apply_gains(self):
        error = subprocess.CalledProcessError(1, "Gazebo")
        with patch("analysis.apply_optimized_gains.validate_in_gazebo", side_effect=error):
            with self.assertRaises(subprocess.CalledProcessError): apply_run(self.folder, self.root)
        self.assert_kept()

    def test_old_wheel_feedback_search_cannot_be_promoted(self):
        self.search["feedback_mode"]="encoder_feedback"
        self.save_search()
        with self.assertRaises(UpdateRejected):self.apply()
        self.assert_kept()

    def test_changed_ros_adapter_invalidates_validation(self):
        (self.root/GAZEBO_NATIVE.parent/"balance_controller").write_bytes(b"changed measurements")
        with self.assertRaises(UpdateRejected):self.apply()
        self.assert_kept()

    def test_integer_valued_gain_remains_a_ros_double(self):
        self.candidate["balance_controller"]["ros__parameters"]["ki"]=2.0
        self.save_yaml(self.folder/"candidate.yaml",self.candidate)
        self.search["selected"]["gains"][-1]=2.0
        self.save_search()
        for case in CASES:
            self.save_yaml(self.folder/"gazebo-validation/candidate"/case/"parameters.yaml",self.candidate)
        self.apply()
        self.assertIn(b"ki: 2.0",self.active.read_bytes())
        self.assertIsInstance(yaml.safe_load(self.active.read_text())["balance_controller"]["ros__parameters"]["ki"],float)


if __name__ == "__main__":
    unittest.main()
