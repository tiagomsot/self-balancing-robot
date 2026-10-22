"""Reference-endpoint, electrical-limit, and braking checks for the actuator."""

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "ros2_ws/src/balancing_robot_sim/scripts"))
from motor_model import MotorDriverModel, MotorParameters


class MotorModelTests(unittest.TestCase):
    def setUp(self):
        self.p = MotorParameters()

    def test_motor_alone_matches_reference_stall(self):
        model = MotorDriverModel(replace(self.p, driver_drop_at_rating_v=0.0,
                                         driver_drop_intercept_v=0.0))
        result = model.step([1.0, 0.0], [0.0, 0.0], 0.005)
        self.assertAlmostEqual(result.left.current_a, 1.5)
        self.assertAlmostEqual(result.left.torque_nm, self.p.stall_torque_nm)

    def test_motor_alone_matches_reference_no_load_speed_and_current(self):
        model = MotorDriverModel(replace(self.p, driver_drop_at_rating_v=0.0,
                                         driver_drop_intercept_v=0.0))
        result = model.step([1.0, 0.0], [self.p.no_load_speed_rad_s, 0.0], 0.005)
        self.assertAlmostEqual(result.left.current_a, 0.15)
        self.assertAlmostEqual(result.left.torque_nm, 0.0)

    def test_driver_loss_anchor_at_point_six_amps(self):
        # 5 V supply produces 0.6 A through 4 ohm motor and 2.6 V driver loss.
        model = MotorDriverModel(replace(self.p, battery_voltage_v=5.0))
        result = model.step([1.0, 0.0], [0.0, 0.0], 0.005)
        self.assertAlmostEqual(result.left.current_a, 0.6)
        self.assertAlmostEqual(result.left.driver_loss_w, 0.6 * 2.6)

    def test_torque_decreases_with_speed_at_same_pwm(self):
        model = MotorDriverModel(self.p)
        slow = model.step([1.0, 0.0], [0.0, 0.0], 0.005)
        fast = model.step([1.0, 0.0], [10.0, 0.0], 0.005)
        self.assertGreater(slow.left.torque_nm, fast.left.torque_nm)

    def test_loss_uncertainty_does_not_recalibrate_pwm(self):
        typical = MotorDriverModel(self.p)
        higher_loss = MotorDriverModel(replace(self.p, driver_drop_at_rating_v=3.6,
                                               driver_drop_intercept_v=2.6))
        first = typical.step([0.02, 0], [0, 0], 0.005)
        second = higher_loss.step([0.02, 0], [0, 0], 0.005)
        self.assertEqual(first.left.duty, second.left.duty)
        self.assertGreater(first.left.torque_nm, second.left.torque_nm)

    def test_forward_and_reverse_are_symmetric(self):
        forward = MotorDriverModel(self.p).step([0.02, 0], [2.0, 0], 0.005)
        reverse = MotorDriverModel(self.p).step([-0.02, 0], [-2.0, 0], 0.005)
        self.assertAlmostEqual(forward.left.torque_nm, -reverse.left.torque_nm)
        self.assertAlmostEqual(forward.left.current_a, -reverse.left.current_a)

    def test_driver_does_not_invent_current_regulation(self):
        result = MotorDriverModel(self.p).step([1, 0], [0, 0], 0.005)
        self.assertGreater(result.left.current_a, self.p.driver_current_rating_a)
        self.assertTrue(result.average_current_rating_exceeded)
        self.assertTrue(result.left.command_saturated)

    def test_small_coast_pwm_has_pulse_risk_even_when_average_is_low(self):
        result = MotorDriverModel(self.p).step([0.005, 0], [0, 0], 0.005)
        self.assertLess(result.left.current_a, 0.6)
        self.assertTrue(result.on_current_rating_uncertain)
        self.assertGreater(result.left.rms_current_estimate_a, abs(result.left.current_a))

    def test_zero_pwm_coasts_and_preserves_mechanical_drag(self):
        result = MotorDriverModel(self.p).step([0, 0], [3, 0], 0.005)
        self.assertEqual(result.left.current_a, 0)
        self.assertLess(result.left.torque_nm, 0)

    def test_zero_pwm_braking_opposes_shaft_motion(self):
        result = MotorDriverModel(replace(self.p, pwm_off_mode="brake")).step([0, 0], [3, 0], 0.005)
        self.assertLess(result.left.current_a, 0)
        self.assertLess(result.left.torque_nm, 0)
        self.assertEqual(result.supply_current_a, 0)

    def test_battery_sag_is_shared_by_both_motors(self):
        params = replace(self.p, battery_resistance_ohm=0.8)
        one = MotorDriverModel(params).step([0.03, 0], [0, 0], 0.005)
        two = MotorDriverModel(params).step([0.03, 0.03], [0, 0], 0.005)
        self.assertLess(two.bus_voltage_v, one.bus_voltage_v)
        self.assertLess(two.left.torque_nm, one.left.torque_nm)
        self.assertAlmostEqual(two.bus_voltage_v + 0.8 * two.supply_current_a, 6.0, places=7)

    def test_pwm_resolution_and_optional_deadband(self):
        model = MotorDriverModel(replace(self.p, command_deadband=0.1))
        full = model.p.reference_full_pwm_stall_torque_nm
        self.assertEqual(model.duty_for_torque(0.05 * full)[0], 0)
        duty = model.duty_for_torque(0.25 * full)[0]
        self.assertAlmostEqual(duty * 255, round(duty * 255))
        self.assertGreater(duty, 0.1)

    def test_pwm_on_time_is_a_diagnostic_not_a_resolved_current_waveform(self):
        slow = MotorDriverModel(self.p).step([0.02, 0], [0, 0], 0.005)
        fast = MotorDriverModel(replace(self.p, pwm_frequency_hz=980)).step([0.02, 0], [0, 0], 0.005)
        self.assertAlmostEqual(slow.left.current_a, fast.left.current_a)
        self.assertAlmostEqual(slow.left.on_time_estimate_s, 2 * fast.left.on_time_estimate_s)

    def test_optional_response_lag_is_monotone(self):
        model = MotorDriverModel(replace(self.p, current_time_constant_s=0.01))
        first = model.step([0.02, 0], [0, 0], 0.001).left.current_a
        second = model.step([0.02, 0], [0, 0], 0.001).left.current_a
        self.assertGreater(second, first)
        self.assertLess(second, MotorDriverModel(self.p).step([0.02, 0], [0, 0], 0.001).left.current_a)

    def test_supply_violation_is_reported(self):
        result = MotorDriverModel(replace(self.p, battery_voltage_v=4.5)).step([0.02, 0], [0, 0], 0.005)
        self.assertTrue(result.supply_out_of_range)

    def test_invalid_configuration_and_inputs_are_rejected(self):
        for kwargs in ({"no_load_current_a": 2}, {"pwm_levels": 0},
                       {"battery_voltage_v": math.nan}, {"command_deadband": 1},
                       {"pwm_off_mode": "unknown"}):
            with self.assertRaises(ValueError):
                MotorParameters(**kwargs)
        with self.assertRaises(ValueError):
            MotorDriverModel().step([0, math.nan], [0, 0], 0.005)


if __name__ == "__main__":
    unittest.main()
