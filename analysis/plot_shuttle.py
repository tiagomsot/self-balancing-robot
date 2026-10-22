"""Plot a recorded shuttle and check its stopped positions using world poses."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, default=Path("docs/images/shuttle-demo.png"))
    args = parser.parse_args()
    summary = json.loads((args.run / "summary.json").read_text())
    settings = json.loads((args.run / "run.json").read_text())
    data = np.genfromtxt(args.run / "telemetry.csv", delimiter=",", names=True)
    truth = np.genfromtxt(args.run / "ground-truth.csv", delimiter=",", names=True)
    _, indices = np.unique(truth["time_s"], return_index=True)
    truth = truth[np.sort(indices)]
    events = summary["final_status"]["events"]
    drives = [event for event in events if event["phase"] == "DRIVE"]
    leg_ends = [event for event in events if event["phase"] in ("TURN", "DONE")]
    turns = [event for event in events if event["phase"] == "TURN"]

    def sample(stamp):
        x = float(np.interp(stamp, truth["time_s"], truth["x_m"]))
        y = float(np.interp(stamp, truth["time_s"], truth["y_m"]))
        return np.array([x, y])

    origin = sample(drives[0]["time_s"])
    heading0 = float(np.interp(drives[0]["time_s"], truth["time_s"], np.unwrap(truth["yaw_rad"])))
    goal = origin+settings["distance_m"]*np.array([math.cos(heading0), math.sin(heading0)])
    end = sample(summary["final_status"]["time_s"])
    stop_speeds = [float(np.linalg.norm(sample(e["time_s"])-sample(e["time_s"]-0.25))/0.25)
                   for e in leg_ends]
    turn_drifts = [float(np.linalg.norm(sample(d["time_s"])-sample(t["time_s"])))
                   for t, d in zip(turns, drives[1:])]
    cross_track = (-(truth["x_m"]-origin[0])*math.sin(heading0)
                   +(truth["y_m"]-origin[1])*math.cos(heading0))
    active = truth["time_s"] >= drives[0]["time_s"]
    return_error = float(np.linalg.norm(end-origin))
    checks = dict(
        recorded_response_passed=summary["passed"],
        returned_to_start=return_error < 0.05,
        straight_path=max(abs(cross_track[active])) < 0.05,
        independently_stopped=bool(stop_speeds) and max(stop_speeds) < 0.03,
        turns_near_endpoint=bool(turn_drifts) and max(turn_drifts) < 0.05,
        truth_covers_routine=truth["time_s"][0] <= drives[0]["time_s"] and truth["time_s"][-1] >= leg_ends[-1]["time_s"],
    )
    checks = {name: bool(value) for name, value in checks.items()}
    report = dict(passed=all(checks.values()), checks=checks,
                  distance_target_m=settings["distance_m"], cruise_speed_target_m_s=settings["speed_m_s"],
                  cycles_completed=summary["final_status"]["cycles_completed"],
                  ground_truth_leg_distances_m=summary["ground_truth_leg_distances_m"],
                  ground_truth_turn_errors_deg=summary["ground_truth_turn_errors_deg"],
                  return_error_m=return_error, maximum_cross_track_m=float(max(abs(cross_track[active]))),
                  stopped_average_speeds_m_s=stop_speeds, turn_center_drift_m=turn_drifts,
                  maximum_abs_pitch_deg=float(np.max(abs(np.rad2deg(truth["pitch_rad"])))),
                  peak_observed_forward_speed_m_s=float(np.max(abs(data["speed_m_s"]))),
                  maximum_average_motor_current_a=summary["maximum_average_motor_current_a"],
                  pulse_rating_uncertain=summary["pulse_rating_uncertain"],
                  hardware_rating_compliance_established=False,
                  source_sha256=summary["source_sha256"],
                  plotter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  recording_directory=args.run.name,
                  note="Nominal simulation checks; wheel sensing/orientation are ideal. Distance and speed are targets, not exact measured motion.")
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained")
    fig.suptitle("Balancing robot | drive, stop, turn 180°, repeat", fontsize=17)
    ax = axes[0, 0]
    ax.plot(truth["x_m"], truth["y_m"], label="Gazebo world pose", lw=1.8, color="#007c91")
    ax.plot(data["x_m"], data["y_m"], "--", label="Wheel + IMU odometry", lw=1.0, color="#b35328")
    ax.scatter([origin[0], goal[0]], [origin[1], goal[1]], marker="x", s=70, color="black",
               label=f"Waypoints {settings['distance_m']:.2f} m apart")
    ax.set(title="Independent world path", xlabel="World x (m)", ylabel="World y (m)", ylim=(-0.10, 0.10))
    ax.set_aspect("equal", adjustable="box")
    ax.legend(fontsize=9)
    times = data["time_s"]
    heading_target = np.zeros_like(times)
    seen = set()
    for event in turns:
        if event["legs_completed"] not in seen:
            heading_target[times >= event["time_s"]] = 180*event["legs_completed"]
            seen.add(event["legs_completed"])
    axes[0, 1].plot(times, np.rad2deg(np.unwrap(data["yaw_rad"])), color="#007c91", label="Measured heading")
    axes[0, 1].plot(times, heading_target, "--", color="#b35328", label="Heading target")
    axes[0, 1].set(title="Half-turns at both ends", xlabel="Simulation time (s)", ylabel="Unwrapped heading (°)")
    axes[0, 1].legend(fontsize=9)
    axes[1, 0].plot(times, data["speed_m_s"], color="#007c91", label="Measured forward speed")
    axes[1, 0].plot(times, data["command_v"], "--", color="#b35328", label="Supervisor command")
    axes[1, 0].set(title="Gentle acceleration, braking and pauses", xlabel="Simulation time (s)", ylabel="Body-forward speed (m/s)")
    axes[1, 0].legend(fontsize=9)
    axes[1, 1].plot(times, np.rad2deg(data["pitch_rad"]), color="#007c91")
    axes[1, 1].axhspan(-1, 1, color="#007c91", alpha=0.08)
    axes[1, 1].set(title="The balance loop stays active throughout", xlabel="Simulation time (s)", ylabel="Body pitch (°)")
    for index, event in enumerate(events):
        end_time = events[index+1]["time_s"] if index+1 < len(events) else times[-1]
        color = {"STOP": "#54a24b", "TURN": "#4c78a8", "TURN_SETTLE": "#4c78a8"}.get(event["phase"])
        if color:
            for ax in (axes[0, 1], axes[1, 0], axes[1, 1]):
                ax.axvspan(event["time_s"], end_time, color=color, alpha=0.07)
    for ax in axes.flat:
        ax.grid(alpha=0.18)
    fig.supxlabel("Shading: green = stop, blue = turn/settle • Nominal motor model • Ideal sensing • Hardware validation pending", fontsize=10)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise RuntimeError("Post-processing checks failed; evidence was preserved")


if __name__ == "__main__":
    main()
