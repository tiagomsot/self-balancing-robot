"""Pause/release regressions for the actual ROS adapters (optional ROS runtime)."""
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/balancing_robot_sim/scripts"))
try:
    import rclpy
except ImportError:
    rclpy = None
else:
    from balance_controller import BalanceController
    from motor_driver import MotorDriver


@unittest.skipIf(rclpy is None, "Source ROS 2 to test simulation pause/release watchdogs")
class RosStartupTests(unittest.TestCase):
    def controller(self, running=False, fault=False):
        now = time.monotonic()
        return SimpleNamespace(
            fault=fault, resume_complete=running, last_state=[0] * 10,
            last_imu_wall=now-1, settings={"sensor_timeout_s": .25},
            last_motor_command=(-.02, -.02),
            left=Mock(**{"get_subscription_count.return_value": 0}),
            right=Mock(**{"get_subscription_count.return_value": 0}),
            publish_torque=Mock(), count_publishers=Mock(return_value=0),
            resume_process=None, resume_phase="feedback", motor_ready=False,
            bootstrap_wall=now, bootstrap_attempts=0, started_wall=now,
            last_log_wall=now)

    def motor(self, simulation_time=.025, expired_request=False):
        now = time.monotonic()
        return SimpleNamespace(
            timeout=.25, last_joint_wall=now-1,
            last_joint_message=SimpleNamespace(header=SimpleNamespace(
                stamp=SimpleNamespace(sec=0, nanosec=25_000_000))),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                nanoseconds=round(simulation_time * 1e9))),
            request_times=[now-1 if expired_request else now, now],
            left=Mock(**{"get_subscription_count.return_value": 1}),
            right=Mock(**{"get_subscription_count.return_value": 1}),
            ready=Mock(), publish=Mock())

    def test_paused_release_preserves_the_computed_balance_command(self):
        node = self.controller()
        BalanceController.on_wall_timer(node)
        self.assertEqual(node.left.publish.call_args.args[0].data, -.02)
        self.assertEqual(node.right.publish.call_args.args[0].data, -.02)
        node.publish_torque.assert_not_called()

    def test_running_world_still_disables_torque_on_stale_imu(self):
        node = self.controller(running=True)
        BalanceController.on_wall_timer(node)
        node.publish_torque.assert_called_once_with(0.0)

    def test_fault_during_bootstrap_still_disables_torque(self):
        node = self.controller(fault=True)
        BalanceController.on_wall_timer(node)
        node.publish_torque.assert_called_once_with(0.0)

    def test_frozen_plant_clock_does_not_erase_fresh_commands(self):
        node = self.motor()
        MotorDriver.watchdog(node)
        node.publish.assert_not_called()

    def test_advancing_clock_with_missing_plant_state_disables_torque(self):
        node = self.motor(simulation_time=.5)
        MotorDriver.watchdog(node)
        node.publish.assert_called_once_with(0.0, 0.0)

    def test_command_loss_disables_torque_even_when_world_is_paused(self):
        node = self.motor(expired_request=True)
        MotorDriver.watchdog(node)
        node.publish.assert_called_once_with(0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
