#!/usr/bin/env python3
"""ROS/Gazebo I/O adapter for the portable C++ controller in cpp/src."""

import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from tf2_ros import TransformBroadcaster

from motion_core import (MotionReference, PlanarOdometry, YawController, allocate_torque, attitude,
                         quaternion, yaw_rate)
from shared_core import Controller, ImuEstimator, fault_name


def clip(value, limit):
    return max(-limit, min(limit, value))


class BalanceController(Node):
    def __init__(self):
        super().__init__("balance_controller")
        defaults = {
            "kp": 0.2192027027, "kd": 0.0124864865, "ki": 0.0,
            "total_torque_limit_nm": 0.060,
            "position_gain": 0.0, "velocity_gain": 0.0,
            "wheel_feedback_enabled": False,
            "target_tilt_limit_deg": 5.0, "wheel_radius_m": 0.04,
            "fall_cutoff_deg": 35.0, "sensor_timeout_s": 0.25,
            "auto_start": True,
            "motion_enabled": False, "motion_command_timeout_s": 0.3,
            "max_speed_m_s": 0.25, "max_yaw_rate_rad_s": math.radians(50),
            "motion_acceleration_m_s2": 0.15, "yaw_acceleration_rad_s2": 0.35,
            "reference_lead_limit_m": 0.08, "yaw_rate_gain": 0.01,
            "yaw_rate_integral_gain": 0.02,
            "yaw_torque_limit_nm": 0.012, "odom_publish_period_s": 0.02,
        }
        self.settings = {key: self.declare_parameter(
                            key, default, descriptor=ParameterDescriptor(read_only=True)).value
                         for key, default in defaults.items()}
        for key, value in self.settings.items():
            if key in ("auto_start", "motion_enabled", "wheel_feedback_enabled"):
                continue
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
        for key in ("kp", "total_torque_limit_nm", "wheel_radius_m", "fall_cutoff_deg",
                    "sensor_timeout_s"):
            if self.settings[key] <= 0:
                raise ValueError(f"{key} must be positive")
        if not 0 < self.settings["fall_cutoff_deg"] < 90:
            raise ValueError("fall_cutoff_deg must be between 0 and 90")
        self.left = self.create_publisher(Float64, "/left_wheel/torque_request", 1)
        self.right = self.create_publisher(Float64, "/right_wheel/torque_request", 1)
        self.odom = self.create_publisher(Odometry, "/evaluation/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.odometry = PlanarOdometry()
        self.core = Controller(self.settings)
        if self.settings["wheel_feedback_enabled"]:
            raise ValueError("The hardware simulation is IMU-only: wheel feedback must remain disabled")
        self.imu_estimator = ImuEstimator()
        self.previous_imu_readings = None
        self.last_motor_command = (0.0, 0.0)
        self.velocity_command = self.yaw_command = 0.0
        self.last_command_wall = self.last_control_stamp = self.last_odom_stamp = None
        self.create_subscription(Twist, "/cmd_vel", self.on_command, 1)
        self.fault_status = self.create_publisher(Bool, "/balance/fault", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.fault_status.publish(Bool(data=False))
        # Keep startup history so a recorder discovered slightly later can still
        # capture the initial recovery. Commands themselves remain volatile.
        self.telemetry = self.create_publisher(
            Float64MultiArray, "/balance/telemetry",
            QoSProfile(depth=4000, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.motor_ready = False
        self.create_subscription(Bool, "/motors/ready", self.on_motor_ready, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_imu_message = None
        self.wheel_state = None
        self.position_reference = None
        self.last_imu_wall = None
        self.last_joint_wall = None
        self.last_state = None
        self.fault = False
        self.started_wall = time.monotonic()
        self.last_log_wall = self.started_wall
        self.resume_process = None
        self.resume_complete = not self.settings["auto_start"]
        self.resume_phase = "bootstrap"
        self.bootstrap_attempts = 0
        self.bootstrap_wall = None
        # A wall clock is needed while the simulation itself is paused.
        self.create_timer(0.02, self.on_wall_timer,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"IMU PID: Kp={self.settings['kp']:.5f}, Kd={self.settings['kd']:.5f}, Ki={self.settings['ki']:.5f}; "
                f"total request cap +/-{self.settings['total_torque_limit_nm']:.3f} N m")

    def publish_torque(self, total, difference=0.0):
        left, right = allocate_torque(total, difference, self.settings["total_torque_limit_nm"])
        self.left.publish(Float64(data=left))
        self.right.publish(Float64(data=right))

    def on_command(self, message):
        linear, angular = message.linear.x, message.angular.z
        if not math.isfinite(linear) or not math.isfinite(angular):
            linear = angular = 0.0
        self.velocity_command = clip(linear, self.settings["max_speed_m_s"])
        self.yaw_command = clip(angular, self.settings["max_yaw_rate_rad_s"])
        self.last_command_wall = time.monotonic()

    def on_motor_ready(self, message):
        self.motor_ready = message.data

    def stop_with_fault(self, reason):
        if not self.fault:
            self.get_logger().error(reason + "; torque disabled. Stop and relaunch to restart.")
        self.fault = True
        self.fault_status.publish(Bool(data=True))
        self.publish_torque(0.0)

    def on_joints(self, message):
        try:
            indices = [message.name.index(name) for name in
                       ("left_wheel_joint", "right_wheel_joint")]
            positions = [message.position[i] for i in indices]
            velocities = [message.velocity[i] for i in indices]
        except (ValueError, IndexError):
            return
        if not all(math.isfinite(value) for value in positions + velocities):
            # Evaluation-only channel; it cannot arm, stop or steer the controller.
            return
        self.wheel_state = (sum(positions) / 2, sum(velocities) / 2)
        self.last_joint_wall = time.monotonic()

    def on_imu(self, message):
        self.last_imu_message = message
        a, w = message.linear_acceleration, message.angular_velocity
        values = (a.x/9.80665, a.y/9.80665, a.z/9.80665, w.x, w.y, w.z)
        if not all(math.isfinite(value) for value in values):
            self.stop_with_fault("Invalid IMU state")
            return
        now = time.monotonic()
        self.last_imu_wall = now
        if self.fault:
            self.publish_torque(0.0)
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        dt = .005 if self.last_control_stamp is None else stamp-self.last_control_stamp
        self.last_control_stamp = stamp
        # Approximate the physical MPU DLPF's ~4.9 ms delay at 200 Hz.
        delayed = values if self.previous_imu_readings is None else self.previous_imu_readings
        self.previous_imu_readings = values
        pitch, rate, tilt, heading_rate, valid = self.imu_estimator.update(delayed, dt)
        if not valid:
            if self.last_state is not None:
                self.stop_with_fault("Invalid accelerometer/gyro sample")
            self.publish_torque(0.0)
            return
        requested, applied, target, left, right, enabled, fault = self.core.update(
            pitch, rate, 0.0, 0.0, heading_rate, tilt, 0.0, 0.0, False,
            0.0, 0.0, dt, False)
        if fault:
            self.stop_with_fault(fault_name(fault))
            return
        self.left.publish(Float64(data=left))
        self.right.publish(Float64(data=right))
        # Everything below is evaluation/visualization, never controller input.
        self.last_motor_command = (left, right)
        estimated_pitch = pitch
        q = message.orientation
        roll, pitch, world_yaw = attitude(q.x, q.y, q.z, q.w)
        rate = message.angular_velocity.y
        joint_position, joint_velocity = self.wheel_state or (0.0, 0.0)
        absolute_wheel_angle = joint_position + pitch
        if self.position_reference is None:
            self.position_reference = absolute_wheel_angle
        radius = self.settings["wheel_radius_m"]
        position = radius * (absolute_wheel_angle - self.position_reference)
        velocity = radius * (joint_velocity + rate)
        x, y, heading = self.odometry.update(radius*absolute_wheel_angle, world_yaw)
        data = [stamp, pitch, rate, position, velocity, target, requested, applied, estimated_pitch, tilt]
        self.last_state = data
        self.telemetry.publish(Float64MultiArray(
            layout=MultiArrayLayout(
                dim=[MultiArrayDimension(
                    label="time_s,pitch_rad,rate_rad_s,x_m,v_m_s,target_rad,requested_nm,commanded_nm,estimated_pitch_rad,estimated_tilt_rad",
                    size=10, stride=10)]),
            data=data))
        if self.last_odom_stamp is None or stamp-self.last_odom_stamp >= self.settings["odom_publish_period_s"]-1e-9:
            odom = Odometry()
            odom.header.stamp = message.header.stamp
            odom.header.frame_id, odom.child_frame_id = "odom", "base_link"
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            ox, oy, oz, ow = quaternion(roll, pitch, heading)
            odom.pose.pose.orientation = Quaternion(x=ox, y=oy, z=oz, w=ow)
            odom.twist.twist.linear.x = velocity
            odom.twist.twist.angular = message.angular_velocity
            self.odom.publish(odom)
            transform = TransformStamped()
            transform.header, transform.child_frame_id = odom.header, odom.child_frame_id
            transform.transform.translation.x, transform.transform.translation.y = x, y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(transform)
            self.last_odom_stamp = stamp

    def on_wall_timer(self):
        now = time.monotonic()
        if not self.fault and not self.resume_complete and self.last_state is not None:
            # Bootstrap deliberately pauses physics. Keep the computed command
            # alive until release instead of letting a wall watchdog erase it.
            self.left.publish(Float64(data=self.last_motor_command[0]))
            self.right.publish(Float64(data=self.last_motor_command[1]))
        elif self.fault or self.last_imu_wall is None or (
                now - self.last_imu_wall > self.settings["sensor_timeout_s"]):
            self.publish_torque(0.0)
        if not self.resume_complete:
            if self.resume_process is None:
                ready = (self.left.get_subscription_count() > 0
                         and self.right.get_subscription_count() > 0
                         and self.count_publishers("/imu/data") > 0)
                if self.resume_phase == "feedback":
                    ready = ready and self.last_state is not None and self.motor_ready
                    if not ready and now - self.bootstrap_wall > .3 and self.bootstrap_attempts < 12:
                        self.resume_phase = "bootstrap"
                    elif ready and now - self.bootstrap_wall < 0.15:
                        ready = False
                if ready and now - self.started_wall >= 0.75:
                    request = "pause: true multi_step: 5" if self.resume_phase == "bootstrap" else "pause: false"
                    self.resume_process = subprocess.Popen([
                        "gz", "service", "-s", "/world/balance/control",
                        "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                        "--timeout", "5000", "--req", request,
                    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                elif now - self.started_wall > 20:
                    raise RuntimeError("Timed out waiting for the sensor and torque bridges")
            elif self.resume_process.poll() is not None:
                stdout, stderr = self.resume_process.communicate()
                if self.resume_process.returncode or "data: true" not in stdout:
                    raise RuntimeError(f"Could not start the Gazebo world: {stdout} {stderr}")
                self.resume_process = None
                if self.resume_phase == "bootstrap":
                    self.bootstrap_attempts += 1
                    self.bootstrap_wall = now
                    self.resume_phase = "feedback"
                else:
                    self.resume_complete = True
                    self.get_logger().info("Sensors and actuator connected; simulation running.")
            elif now - self.started_wall > 20:
                self.resume_process.kill()
                raise RuntimeError("Timed out resuming the Gazebo world")
        if self.last_state and now - self.last_log_wall >= 2:
            _, pitch, _, position, _, _, _, torque = self.last_state[:8]
            position_name = "wheel_distance" if self.settings["motion_enabled"] else "x"
            self.get_logger().info(
                f"pitch={math.degrees(pitch):+.2f} deg, {position_name}={position:+.3f} m, "
                f"commanded total torque={torque:+.4f} N m")
            self.last_log_wall = now


def main():
    rclpy.init()
    node = None
    try:
        node = BalanceController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Jazzy can race DDS message-taking against external context shutdown.
        # Preserve genuine runtime errors while the ROS context is still live.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            try:
                if rclpy.ok():
                    node.publish_torque(0.0)
            except Exception:
                if rclpy.ok():
                    raise
            if node.resume_process is not None and node.resume_process.poll() is None:
                node.resume_process.terminate()
                node.resume_process.wait(timeout=5)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
