"""Spawn the unpowered robot, publish its frames, and bridge its sensors."""

import math
import json
import os
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    share = Path(get_package_share_directory("balancing_robot_sim"))
    gui = LaunchConfiguration("gui").perform(context) == "true"
    paused = LaunchConfiguration("paused").perform(context) == "true"
    if LaunchConfiguration("enable_shuttle").perform(context) == "true":
        raise ValueError("Measured-distance motion is disabled: this robot has an IMU and no wheel encoders")
    tilt_deg = float(LaunchConfiguration("initial_tilt_deg").perform(context))
    if not math.isfinite(tilt_deg) or abs(tilt_deg) > 20:
        raise ValueError("initial_tilt_deg must be finite and within +/-20 degrees")

    description = xacro.process_file(str(share / "urdf/robot.urdf.xacro")).toxml()
    geometry = json.loads((share / "config/robot_geometry.json").read_text())
    gz_args = ["-v", "2"]
    if not gui:
        gz_args.append("-s")
    if not paused:
        gz_args.append("-r")
    gz_args.append(str(share / "worlds/balance.sdf"))

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "balance", "-topic", "/robot_description",
            "-name", "balancing_robot", "-z", str(geometry["wheel_radius_m"]+0.001),
            "-P", str(math.radians(tilt_deg)),
        ],
        output="screen",
    )
    actions = [
        # Own the actual Gazebo process, rather than a shell wrapper, so ROS
        # shutdown also stops the server when a headless check finishes.
        ExecuteProcess(
            cmd=["gz", "sim", *gz_args, "--force-version", "8"],
            name="gazebo", output="screen",
            additional_env={"GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join([
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                os.environ.get("LD_LIBRARY_PATH", ""),
            ])},
            on_exit=[EmitEvent(event=Shutdown(reason="Gazebo stopped"))],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": True}],
            output="screen",
        ),
        spawn,
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gazebo_bridge",
            parameters=[{
                "config_file": str(share / "config/bridge.yaml"),
                "use_sim_time": True,
            }],
            output="screen",
        ),
    ]
    if LaunchConfiguration("enable_balance").perform(context) == "true":
        shuttle_enabled = LaunchConfiguration("enable_shuttle").perform(context) == "true"
        actuator_mode = LaunchConfiguration("actuator_mode").perform(context)
        driver_overrides = {} if actuator_mode == "configured" else {"mode": actuator_mode}
        driver = Node(
            package="balancing_robot_sim",
            executable="motor_driver",
            name="motor_driver",
            parameters=[LaunchConfiguration("controller_params").perform(context), driver_overrides],
            output="screen",
        )
        controller = Node(
            package="balancing_robot_sim",
            executable="balance_controller",
            name="balance_controller",
            parameters=[LaunchConfiguration("controller_params").perform(context),
                        {"motion_enabled": shuttle_enabled,
                         "wheel_radius_m": geometry["wheel_radius_m"]}],
            output="screen",
        )
        motion_nodes = []
        if shuttle_enabled:
            supervisor = Node(
                package="balancing_robot_sim", executable="shuttle_demo", name="shuttle_demo",
                parameters=[LaunchConfiguration("controller_params").perform(context), {
                    "use_sim_time": True,
                    "leg_distance_m": float(LaunchConfiguration("distance_m").perform(context)),
                    "speed_m_s": float(LaunchConfiguration("speed_m_s").perform(context)),
                    "cycles": int(LaunchConfiguration("cycles").perform(context)),
                }], output="screen")
            motion_nodes.append(supervisor)
            actions.insert(0, RegisterEventHandler(OnProcessExit(
                target_action=supervisor,
                on_exit=[EmitEvent(event=Shutdown(reason="Shuttle supervisor stopped"))])))

        def after_spawn(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason="Robot creation failed"))]
            return [driver, controller, *motion_nodes]

        # Register before starting the spawner so a fast creation cannot race it.
        actions.insert(0, RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=after_spawn)))
        actions.insert(0, RegisterEventHandler(OnProcessExit(
            target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(reason="Balance controller stopped"))])))
        actions.insert(0, RegisterEventHandler(OnProcessExit(
            target_action=driver,
            on_exit=[EmitEvent(event=Shutdown(reason="Motor-driver model stopped"))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"],
                              description="Open the Gazebo graphics window"),
        DeclareLaunchArgument("paused", default_value="true", choices=["true", "false"],
                              description="Wait for Play or explicit simulation steps"),
        DeclareLaunchArgument("initial_tilt_deg", default_value="5.0",
                              description="Initial forward lean in degrees, from -20 to +20"),
        DeclareLaunchArgument("enable_balance", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("enable_shuttle", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("distance_m", default_value="0.50"),
        DeclareLaunchArgument("speed_m_s", default_value="0.25"),
        DeclareLaunchArgument("cycles", default_value="0"),
        DeclareLaunchArgument("actuator_mode", default_value="configured",
                              choices=["configured", "ideal", "motor_driver"]),
        DeclareLaunchArgument("controller_params", default_value=str(
            Path(get_package_share_directory("balancing_robot_sim")) / "config/balance.yaml")),
        OpaqueFunction(function=launch_nodes),
    ])
