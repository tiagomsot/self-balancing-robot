#!/usr/bin/env python3
"""Publish velocity references for a repeated drive/stop/180-degree-turn demo."""

from dataclasses import asdict
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Float64MultiArray, MultiArrayDimension, MultiArrayLayout, String

from motion_core import MotionState, Shuttle, ShuttleSettings, attitude, yaw_rate

FIELDS = ("time_s", "phase", "legs_completed", "x_m", "y_m", "yaw_rad", "speed_m_s",
          "yaw_rate_rad_s", "pitch_rad", "progress_m", "remaining_m", "command_v", "command_yaw_rate")
PHASES = ("WAIT_BALANCE", "DRIVE", "STOP", "TURN", "TURN_SETTLE", "DONE", "FAULT", "WAIT")


class ShuttleDemo(Node):
    def __init__(self):
        super().__init__("shuttle_demo")
        values = {name: self.declare_parameter(name, value, ParameterDescriptor(read_only=True)).value
                  for name, value in asdict(ShuttleSettings()).items()}
        self.routine = Shuttle(ShuttleSettings(**values))
        self.command = self.create_publisher(Twist, "/cmd_vel", 1)
        latched = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, "/shuttle/status", latched)
        self.telemetry = self.create_publisher(Float64MultiArray, "/shuttle/telemetry", QoSProfile(
            depth=5000, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, "/odom", self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(Bool, "/balance/fault", self.on_fault, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state = self.last_wall = self.last_stamp = self.last_phase = None
        self.balance_faulted = False
        self.last_command = (0.0, 0.0)
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"Shuttle: {values['leg_distance_m']:.2f} m per leg, "
                               f"{values['speed_m_s']:.2f} m/s; turns at both ends.")

    def on_fault(self, message):
        self.balance_faulted = message.data

    def on_odometry(self, message):
        pose, twist = message.pose.pose, message.twist.twist
        q = pose.orientation
        roll, pitch, heading = attitude(q.x, q.y, q.z, q.w)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        self.state = MotionState(stamp, pose.position.x, pose.position.y, heading, pitch,
                                 twist.angular.y, twist.linear.x,
                                 yaw_rate(roll, pitch, twist.angular.y, twist.angular.z))
        self.last_wall = time.monotonic()

    def publish_command(self, velocity, angular):
        message = Twist()
        message.linear.x, message.angular.z = velocity, angular
        self.command.publish(message)

    def tick(self):
        if self.state is None or time.monotonic()-self.last_wall > 0.3:
            self.publish_command(0.0, 0.0)
            return
        state = self.state
        if self.balance_faulted:
            self.routine.fault("Balance controller reported a fault", state)
            self.last_command = (0.0, 0.0)
        elif self.last_stamp is None or state.time_s > self.last_stamp:
            try:
                self.last_command = self.routine.update(state)
            except ValueError as error:
                self.routine.fault(str(error), state)
                self.last_command = (0.0, 0.0)
            self.last_stamp = state.time_s
        self.publish_command(*self.last_command)
        routine = self.routine
        if routine.phase != self.last_phase:
            report = dict(phase=routine.phase, time_s=state.time_s,
                          legs_completed=routine.legs_completed,
                          cycles_completed=routine.legs_completed//2,
                          fault_reason=routine.fault_reason,
                          requested_distance_m=routine.p.leg_distance_m,
                          legs=routine.legs, events=routine.events)
            self.status.publish(String(data=json.dumps(report)))
            self.get_logger().info(f"{routine.phase}; completed legs={routine.legs_completed}")
            self.last_phase = routine.phase
        data = [state.time_s, float(PHASES.index(routine.phase)), float(routine.legs_completed),
                state.x, state.y, state.yaw, state.speed, state.yaw_rate, state.pitch,
                routine.progress, routine.remaining, *self.last_command]
        self.telemetry.publish(Float64MultiArray(layout=MultiArrayLayout(dim=[MultiArrayDimension(
            label=",".join(FIELDS), size=len(FIELDS), stride=len(FIELDS))]), data=data))


def main():
    rclpy.init()
    node = None
    try:
        node = ShuttleDemo()
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
                    node.publish_command(0.0, 0.0)
            except Exception:
                if rclpy.ok():
                    raise
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
