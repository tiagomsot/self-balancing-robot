"""Start the preset balance demo; the controller resumes the paused world."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("balancing_robot_sim"))
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("initial_tilt_deg", default_value="5.0"),
        DeclareLaunchArgument("params_file", default_value=str(share / "config/balance.yaml")),
        DeclareLaunchArgument("actuator_mode", default_value="configured",
                              choices=["configured", "ideal", "motor_driver"]),
        DeclareLaunchArgument("shuttle", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("distance_m", default_value="0.50"),
        DeclareLaunchArgument("speed_m_s", default_value="0.25"),
        DeclareLaunchArgument("cycles", default_value="0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/passive.launch.py")),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "initial_tilt_deg": LaunchConfiguration("initial_tilt_deg"),
                "controller_params": LaunchConfiguration("params_file"),
                "actuator_mode": LaunchConfiguration("actuator_mode"),
                "enable_balance": "true",
                "enable_shuttle": LaunchConfiguration("shuttle"),
                "distance_m": LaunchConfiguration("distance_m"),
                "speed_m_s": LaunchConfiguration("speed_m_s"),
                "cycles": LaunchConfiguration("cycles"),
                "paused": "true",
            }.items(),
        ),
    ])
