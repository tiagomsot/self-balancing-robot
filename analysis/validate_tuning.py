"""Compare saved gain profiles with repeated, isolated real ROS/Gazebo runs."""

import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analysis.tuning_model import BASELINE, GAIN_NAMES, ROOT, score_metrics


CASES = (
    ("nominal_5deg", 5, {}),
    ("nominal_minus5deg", -5, {}),
    ("nominal_10deg", 10, {}),
    ("nominal_minus10deg", -10, {}),
    ("deadband_lag_5deg", 5, dict(command_deadband=0.08, current_time_constant_s=0.002)),
    ("deadband_lag_minus5deg", -5, dict(command_deadband=0.08, current_time_constant_s=0.002)),
    ("deadband_lag_10deg", 10, dict(command_deadband=0.08, current_time_constant_s=0.002)),
    ("lower_battery_10deg", 10, dict(battery_voltage_v=5.5, battery_resistance_ohm=0.5)),
    ("higher_drop_5deg", 5, dict(driver_drop_at_rating_v=3.6, driver_drop_intercept_v=2.6)),
    ("combined_5deg", 5, dict(command_deadband=0.08, current_time_constant_s=0.002,
                             battery_voltage_v=5.5, battery_resistance_ohm=0.5,
                             driver_drop_at_rating_v=3.6, driver_drop_intercept_v=2.6)),
)


def metrics_from_recording(folder):
    """Apply the search's cost to saved ROS samples, with time weighting.

    This is comparison evidence only: passing uses the independent recorder's
    response checks. Missing samples/gaps are reported, not filled or smoothed.
    """
    summary = json.loads((folder / "summary.json").read_text())
    motion = np.genfromtxt(folder / "telemetry.csv", delimiter=",", names=True)
    motor = np.genfromtxt(folder / "motors.csv", delimiter=",", names=True)
    def ordered_unique(data):
        # Command refreshes can publish multiple motor records at one stamp.
        _, reverse_indices = np.unique(data["time_s"][::-1], return_index=True)
        return data[np.sort(len(data)-1-reverse_indices)]
    motion, motor = ordered_unique(motion), ordered_unique(motor)
    duration = summary["requested_duration_s"]

    def average(data, values, tail=False):
        times = data["time_s"]
        if tail:
            keep = times >= duration - 1
            times, values = times[keep], values[keep]
        if len(times) < 2 or times[-1] <= times[0]:
            return float(values[-1]) if len(values) else 1e6
        return float(np.sum((values[1:] + values[:-1]) * np.diff(times) / 2)
                     / (times[-1]-times[0]))

    m = dict(duration_s=duration, fell=summary["controller_faulted"],
             maximum_pitch_deg=summary["maximum_abs_pitch_deg"],
             tail_max_pitch_deg=summary["last_second_max_abs_pitch_deg"],
             tail_max_rate_deg_s=summary["last_second_max_abs_rate_deg_s"],
             final_position_m=summary["final_position_m"],
             final_velocity_m_s=summary["final_velocity_m_s"],
             maximum_current_a=summary["actuator"]["maximum_average_motor_current_a"],
             maximum_on_estimate_a=summary["actuator"]["maximum_on_current_estimate_a"],
             minimum_bus_v=summary["actuator"]["minimum_bus_voltage_v"],
             maximum_steady_junction_estimate_c=summary["actuator"]["maximum_steady_junction_estimate_c"])
    for name, field in (("pitch", "pitch_rad"), ("position", "x_m"),
                         ("velocity", "v_m_s"), ("rate", "rate_rad_s")):
        m[f"mean_{name}_squared"] = average(motion, motion[field]**2)
        m[f"tail_mean_{name}_squared"] = average(motion, motion[field]**2, tail=True)
    m["mean_current_squared"] = average(motor, (motor["current_left_a"]**2 + motor["current_right_a"]**2)/2)
    in_band = ((np.abs(motion["pitch_rad"]) <= np.deg2rad(1)) &
               (np.abs(motion["rate_rad_s"]) <= np.deg2rad(5)) &
               (np.abs(motion["x_m"]) <= 0.02) & (np.abs(motion["v_m_s"]) <= 0.02))
    remains = np.logical_and.accumulate(in_band[::-1])[::-1]
    eligible = remains & (motion["time_s"] <= duration - 0.5)
    m["settling_time_s"] = float(motion["time_s"][np.flatnonzero(eligible)[0]]) if np.any(eligible) else duration
    if summary.get("feedback_mode") == "imu_only":
        degree=np.pi/180
        m["cost"] = float(.6*m["mean_pitch_squared"]/(5*degree)**2
                          +.05*(m["maximum_current_a"]/.6)**2
                          +1.5*m["tail_mean_pitch_squared"]/degree**2
                          +.3*m["tail_mean_rate_squared"]/(5*degree)**2)
        pitch_remains = np.logical_and.accumulate((np.abs(motion["pitch_rad"])<=degree)[::-1])[::-1]
        pitch_settled = pitch_remains & (motion["time_s"] <= duration-.5)
        m["settling_time_s"] = float(motion["time_s"][np.flatnonzero(pitch_settled)[0]]) if np.any(pitch_settled) else duration
    else:
        m["cost"] = float(score_metrics(m))
    m["largest_controller_sample_gap_s"] = float(np.max(np.diff(motion["time_s"])))
    m["largest_motor_sample_gap_s"] = float(np.max(np.diff(motor["time_s"])))
    m["response_passed"] = summary["passed"]
    m["electrical_prediction_passed"] = not any(summary["actuator"][key] for key in (
        "average_current_rating_exceeded", "peak_current_estimate_exceeded",
        "supply_out_of_range", "steady_thermal_proxy_exceeds_rating"))
    return m


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path)
    parser.add_argument("--profiles", nargs="+", choices=("baseline", "candidate"),
                        default=["baseline", "candidate"])
    parser.add_argument("--cases", nargs="+", choices=[c[0] for c in CASES])
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--output", type=Path, default=Path("outputs/controller-gazebo-validation"))
    args = parser.parse_args()
    if "candidate" in args.profiles and not args.candidate:
        parser.error("--candidate is required for candidate runs")
    args.output.mkdir(parents=True, exist_ok=True)
    defaults = yaml.safe_load((ROOT / "ros2_ws/src/balancing_robot_sim/config/balance.yaml").read_text())
    baseline = dict(zip(GAIN_NAMES, BASELINE.tolist()), total_torque_limit_nm=0.060)
    candidate = (yaml.safe_load(args.candidate.read_text())["balance_controller"]["ros__parameters"]
                 if args.candidate else {})
    records = []
    manifest = args.output / "validation.json"
    if manifest.exists():
        records = json.loads(manifest.read_text())["runs"]
    for profile in args.profiles:
        for case, tilt, motor_overrides in CASES:
            if args.cases and case not in args.cases:
                continue
            folder = args.output / profile / case
            if (folder / "summary.json").exists() or (folder / "recorder.log").exists():
                print(f"Preserving existing {profile}/{case}", flush=True)
                continue
            folder.mkdir(parents=True, exist_ok=True)
            parameters = json.loads(json.dumps(defaults))
            parameters["balance_controller"]["ros__parameters"].update(
                baseline if profile == "baseline" else candidate)
            parameters["motor_driver"]["ros__parameters"].update(motor_overrides)
            profile_file = folder / "input-parameters.yaml"
            profile_file.write_text(yaml.safe_dump(parameters, sort_keys=False))
            print(f"Running {profile}/{case}", flush=True)
            with (folder / "recorder.log").open("w") as log:
                process = subprocess.run([
                    sys.executable, str(ROOT / "analysis/check_gazebo_balance.py"),
                    "--tilt", str(tilt), "--seconds", str(args.seconds),
                    "--params-file", str(profile_file.resolve()), "--output", str(folder.resolve()),
                ], stdout=log, stderr=subprocess.STDOUT)
            record = dict(profile=profile, scenario=case, folder=str(folder), returncode=process.returncode)
            if (folder / "telemetry.csv").exists():
                record.update(metrics_from_recording(folder))
            else:
                record.update(response_passed=False, error="Incomplete recording; inspect recorder.log and launch.log")
            records.append(record)
            manifest.write_text(json.dumps(dict(
                runs=records, hardware_rating_compliance_established=False,
                note="Independent Gazebo response checks. Electrical predictions do not establish hardware ratings."
            ), indent=2) + "\n")
            print(json.dumps({key: record.get(key) for key in (
                "profile", "scenario", "response_passed", "tail_max_pitch_deg",
                "final_position_m", "final_velocity_m_s", "cost", "error")}), flush=True)


if __name__ == "__main__":
    main()
