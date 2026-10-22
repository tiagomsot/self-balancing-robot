#!/usr/bin/env python3
"""Convert requested wheel torque to quantized PWM and model delivered torque."""

import math
import time
from dataclasses import asdict

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, MultiArrayDimension, MultiArrayLayout

from motor_model import MotorDriverModel, MotorParameters


FIELDS = [
    "time_s", "requested_left_nm", "requested_right_nm", "applied_left_nm", "applied_right_nm",
    "duty_left", "duty_right", "current_left_a", "current_right_a", "rms_left_est_a", "rms_right_est_a",
    "on_left_est_a", "on_right_est_a", "bus_voltage_v", "supply_current_a", "driver_loss_w",
    "steady_junction_est_c", "average_current_exceeded", "pulse_rating_uncertain",
    "peak_current_est_exceeded", "supply_out_of_range", "pwm_saturated", "motor_model_enabled",
    "on_time_left_est_s", "on_time_right_est_s",
]


class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        def setting(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

        self.mode = setting("mode", "motor_driver")
        if self.mode not in ("ideal", "motor_driver"):
            raise ValueError("mode must be ideal or motor_driver")
        parameters = {key: setting(key, value)
                      for key, value in asdict(MotorParameters()).items()}
        self.model = MotorDriverModel(MotorParameters(**parameters))
        self.update_period = setting("update_period_s", 0.005)
        self.timeout = setting("command_timeout_s", 0.25)
        if not math.isfinite(self.update_period) or self.update_period <= 0:
            raise ValueError("update_period_s must be positive and finite")
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("command_timeout_s must be positive and finite")
        self.left = self.create_publisher(Float64, "/left_wheel/torque", 1)
        self.right = self.create_publisher(Float64, "/right_wheel/torque", 1)
        self.ready = self.create_publisher(Bool, "/motors/ready", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.telemetry = self.create_publisher(
            Float64MultiArray, "/motors/telemetry",
            QoSProfile(depth=4000, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.requests = [0.0, 0.0]
        self.request_times = [None, None]
        self.last_joint_wall = None
        self.last_joint_message = None
        self.last_step_stamp = None
        self.warned = set()
        self.create_subscription(Float64, "/left_wheel/torque_request",
                                 lambda msg: self.on_request(0, msg), 1)
        self.create_subscription(Float64, "/right_wheel/torque_request",
                                 lambda msg: self.on_request(1, msg), 1)
        self.create_subscription(
            JointState, "/joint_states", self.on_joints,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.02, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"Actuator mode={self.mode}; PWM OFF={self.model.p.pwm_off_mode}; "
            f"supply={self.model.p.battery_voltage_v:.1f} V. Reference motor and driver estimates.")

    def publish(self, left, right):
        self.left.publish(Float64(data=left))
        self.right.publish(Float64(data=right))

    def on_request(self, side, message):
        self.requests[side] = message.data if math.isfinite(message.data) else 0.0
        self.request_times[side] = time.monotonic()
        if self.last_joint_message is not None:
            self.update_from_state(self.last_joint_message, command_refresh=True)

    def watchdog(self):
        now = time.monotonic()
        # A paused simulator has no new joint samples. Expire plant state only
        # when simulation time has advanced beyond that sample; command loss
        # still has an independent wall-time watchdog below.
        state_stamp = (self.last_joint_message.header.stamp.sec +
                       self.last_joint_message.header.stamp.nanosec * 1e-9
                       if self.last_joint_message is not None else None)
        simulation_stamp = self.get_clock().now().nanoseconds * 1e-9
        stale = (state_stamp is None or
                 (now - self.last_joint_wall > self.timeout and
                  simulation_stamp - state_stamp > self.timeout))
        if stale or any(t is None or now - t > self.timeout for t in self.request_times):
            self.publish(0.0, 0.0)
        connected = (self.left.get_subscription_count() > 0
                     and self.right.get_subscription_count() > 0
                     and self.last_joint_wall is not None
                     and all(t is not None for t in self.request_times))
        self.ready.publish(Bool(data=connected))

    def warn_once(self, key, message):
        if key not in self.warned:
            self.get_logger().warning(message)
            self.warned.add(key)

    def on_joints(self, message):
        self.update_from_state(message)

    def update_from_state(self, message, command_refresh=False):
        try:
            speeds = [message.velocity[message.name.index(name)]
                      for name in ("left_wheel_joint", "right_wheel_joint")]
        except (ValueError, IndexError):
            return
        if not all(math.isfinite(value) for value in speeds):
            self.publish(0.0, 0.0)
            return
        now = time.monotonic()
        if not command_refresh:
            self.last_joint_wall = now
            self.last_joint_message = message
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not command_refresh and self.last_step_stamp is not None:
            if stamp < self.last_step_stamp:
                self.last_step_stamp = None
                self.model.currents = [0.0, 0.0]
            elif stamp - self.last_step_stamp < self.update_period - 1e-9:
                return
        dt = 0.0 if command_refresh else (
            self.update_period if self.last_step_stamp is None else stamp - self.last_step_stamp)
        if not command_refresh:
            self.last_step_stamp = stamp
        requests = [value if t is not None and now - t <= self.timeout else 0.0
                    for value, t in zip(self.requests, self.request_times)]
        if self.mode == "ideal":
            self.publish(*requests)
            data = [stamp, *requests, *requests, *([0.0] * (len(FIELDS) - 5))]
        else:
            result = self.model.step(requests, speeds, dt)
            left, right = result.left, result.right
            self.publish(left.torque_nm, right.torque_nm)
            data = [
                stamp, *requests, left.torque_nm, right.torque_nm,
                left.duty, right.duty, left.current_a, right.current_a,
                left.rms_current_estimate_a, right.rms_current_estimate_a,
                left.on_current_estimate_a, right.on_current_estimate_a,
                result.bus_voltage_v, result.supply_current_a, result.driver_loss_w,
                result.steady_state_junction_estimate_c,
                float(result.average_current_rating_exceeded), float(result.on_current_rating_uncertain),
                float(result.peak_current_estimate_exceeded), float(result.supply_out_of_range),
                float(left.command_saturated or right.command_saturated), 1.0,
                left.on_time_estimate_s, right.on_time_estimate_s,
            ]
            if result.average_current_rating_exceeded:
                self.warn_once("average", "Estimated average current exceeds the driver rating; no current clamp is modelled.")
            if result.on_current_rating_uncertain:
                self.warn_once("pulse", "PWM ON-current estimate exceeds the continuous rating. Pulse compliance needs winding/wiring data.")
            if result.peak_current_estimate_exceeded:
                self.warn_once("peak", "ON-current estimate exceeds the absolute peak rating; the 1.2 A rating is not repetitive capability.")
            if result.supply_out_of_range:
                self.warn_once("supply", "Motor supply is below the configured driver operating range; this response is outside the validated voltage range.")
            if result.steady_state_junction_estimate_c > self.model.p.driver_max_junction_c:
                self.warn_once("thermal", "Steady-state junction-temperature proxy exceeds the rating; this is not a transient thermal simulation.")
        self.telemetry.publish(Float64MultiArray(
            layout=MultiArrayLayout(dim=[MultiArrayDimension(
                label=",".join(FIELDS), size=len(FIELDS), stride=len(FIELDS))]), data=data))


def main():
    rclpy.init()
    node = None
    try:
        node = MotorDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            try:
                if rclpy.ok():
                    node.publish(0.0, 0.0)
            except Exception:
                if rclpy.ok():
                    raise
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
