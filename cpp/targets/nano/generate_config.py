"""Generate Nano constants from the existing canonical YAML and geometry JSON."""
import hashlib
import json
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[3]
config = ROOT / "ros2_ws/src/balancing_robot_sim/config/balance.yaml"
geometry = ROOT / "ros2_ws/src/balancing_robot_sim/config/robot_geometry.json"
p = yaml.safe_load(config.read_text())["balance_controller"]["ros__parameters"]
if p.get("wheel_feedback_enabled", False) or p["position_gain"] or p["velocity_gain"]:
    raise ValueError("Nano and simulators must share an IMU-only profile with zero wheel gains")
m = yaml.safe_load(config.read_text())["motor_driver"]["ros__parameters"]
g = json.loads(geometry.read_text())
values = (p["kp"], p["kd"], p["total_torque_limit_nm"], 0, 0,
          p["target_tilt_limit_deg"]*3.141592653589793/180,
          p["fall_cutoff_deg"]*3.141592653589793/180, p["sensor_timeout_s"],
          p["motion_acceleration_m_s2"], p["yaw_acceleration_rad_s2"], .08,
          p["yaw_rate_gain"], p["yaw_rate_integral_gain"], p["yaw_torque_limit_nm"],
          p["max_speed_m_s"], p["max_yaw_rate_rad_s"], p.get("ki", 0.0))
out = Path(__file__).with_name("RobotFirmware") / "GeneratedConfig.h"
text = "// GENERATED: edit config/balance.yaml, then run bash build_cpp.sh nano.\n"
text += "// Owner confirmed no encoders: position/speed gains are explicitly zero.\n"
text += "#pragma once\n#include <RobotCore.h>\n"
text += "inline robot::BalanceSettings nanoSettings() {\n    robot::BalanceSettings p = {"
text += ", ".join(format(v, ".17g") for v in values)+"};\n    return p;\n}\n"
text += f"const double wheelRadius = {g['wheel_radius_m']:.17g};\n"
text += f"const double fullPwmTorque = {m['command_full_pwm_torque_nm']:.17g};\n"
text += f"const int pwmLevels = {m['pwm_levels']};\nconst double commandDeadband = {m['command_deadband']:.17g};\n"
out.write_text(text)
manifest = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in (config, geometry)}
out.with_suffix(".json").write_text(json.dumps(dict(source_sha256=manifest,
    encoder_feedback=False, mode="balance_only", feedback_mode="imu_only", hardware_verified=False), indent=2)+"\n")
print(f"Generated {out}; balance-only configuration (no wheel encoders).")
