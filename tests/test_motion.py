"""Reference limits, turn geometry and complete shuttle-sequence checks."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "ros2_ws/src/balancing_robot_sim/scripts"))
from motion_core import (MotionReference, MotionState, PlanarOdometry, Shuttle,
                         ShuttleSettings, YawController, allocate_torque, attitude, quaternion, wrap)


class MotionTests(unittest.TestCase):
    def test_steering_preserves_balance_and_wheel_request_caps(self):
        for balance in (-0.06, -0.02, 0, 0.02, 0.06):
            for steering in (-0.01, 0, 0.01):
                left, right = allocate_torque(balance, steering, 0.04)
                self.assertLessEqual(max(abs(left), abs(right)), 0.02+1e-12)
                self.assertAlmostEqual(left+right, max(-0.04, min(0.04, balance)))
        self.assertEqual(allocate_torque(0.04, 0.003, 0.04), (0.02, 0.02))

    def test_odometry_returns_after_a_half_turn_and_same_forward_distance(self):
        odom = PlanarOdometry()
        odom.update(0, 0)
        self.assertAlmostEqual(odom.update(0.5, 0)[0], 0.5)
        odom.update(0.5, math.pi)
        x, y, _ = odom.update(1.0, math.pi)
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 0)

    def test_reference_ramps_and_stops_with_bounded_lead(self):
        reference = MotionReference()
        position, speed, turn = reference.update(0, 0.1, 0.35, 0.02)
        self.assertAlmostEqual(speed, 0.003)
        self.assertAlmostEqual(turn, 0.007)
        for _ in range(200):
            position, speed, _ = reference.update(0, 0.1, 0, 0.02)
        self.assertLessEqual(position, 0.08)
        for _ in range(150):
            _, speed, turn = reference.update(0.08, 0, 0, 0.02)
        self.assertAlmostEqual(speed, 0)
        self.assertAlmostEqual(turn, 0)

    def test_quaternion_heading_wrap_and_pitch_survive_a_half_turn(self):
        for heading in (0, math.pi-0.01, -math.pi+0.01):
            roll, pitch, yaw = attitude(*quaternion(0, 0.05, heading))
            self.assertAlmostEqual(roll, 0)
            self.assertAlmostEqual(pitch, 0.05)
            self.assertAlmostEqual(wrap(yaw-heading), 0)

    def test_yaw_integral_overcomes_constant_resistance_but_cannot_take_balance_headroom(self):
        controller = YawController()
        initial = controller.update(0.35, 0, 0.02, 0, 0.04)
        for _ in range(50):
            output = controller.update(0.35, 0, 0.02, 0, 0.04)
        self.assertGreater(output, initial)
        saved_integral = controller.integral
        self.assertEqual(controller.update(0.35, 0, 0.02, 0.04, 0.04), 0)
        self.assertEqual(controller.integral, saved_integral)

    def test_two_round_trips_stop_at_each_end_and_turn_before_repeating(self):
        routine = Shuttle(ShuttleSettings(cycles=2, settle_time_s=0.3, turn_settle_time_s=0.2))
        x = y = heading = velocity = angular = 0.0
        dt = 0.02
        for step in range(10000):
            state = MotionState(step*dt, x, y, wrap(heading), 0, 0, velocity, angular)
            velocity, angular = routine.update(state)
            heading += angular*dt
            x += velocity*math.cos(heading)*dt
            y += velocity*math.sin(heading)*dt
            if routine.phase == "DONE":
                break
        self.assertEqual(routine.phase, "DONE")
        self.assertEqual(routine.legs_completed, 4)
        self.assertEqual(sum(e["phase"] == "STOP" for e in routine.events), 4)
        self.assertEqual(sum(e["phase"] == "TURN" for e in routine.events), 3)
        self.assertLess(math.hypot(x, y), 0.03)
        for leg in routine.legs:
            self.assertLess(abs(leg["projected_distance_m"]-0.5), 0.031)

    def test_failure_to_settle_times_out_without_starting_a_leg(self):
        routine = Shuttle(ShuttleSettings(phase_timeout_s=1))
        routine.update(MotionState(0, 0, 0, 0, 0.1, 0, 0, 0))
        command = routine.update(MotionState(2, 0, 0, 0, 0.1, 0, 0, 0))
        self.assertEqual(routine.phase, "FAULT")
        self.assertEqual(command, (0, 0))


if __name__ == "__main__":
    unittest.main()
