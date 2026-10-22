"""Load recorded runs, recompute metrics, and export traceable comparison reports."""

import csv
from dataclasses import dataclass
import hashlib
import io
import json
from pathlib import Path

import numpy as np

from simulation.model import RobotParameters
from simulation.runner import Experiment, Result

COLUMNS = ("time_s", "axle_position_m", "axle_speed_m_s", "tilt_rad", "pitch_rate_rad_s",
           "requested_total_torque_Nm", "applied_total_torque_Nm")
METRICS = [
    ("fell", "Reached fall cutoff", ""),
    ("simulated_duration_s", "Recorded duration", "s"),
    ("pitch_settling_time_s", "Pitch settling time", "s"),
    ("opposite_side_overshoot_deg", "Overshoot past upright", "deg"),
    ("overshoot_percent_of_initial_tilt", "Overshoot / initial lean", "%"),
    ("significant_upright_crossings", "Upright crossings (0.1 deg threshold)", "count"),
    ("rms_tilt_deg", "RMS tilt over recorded duration", "deg"),
    ("integral_absolute_tilt_deg_s", "Integrated absolute tilt", "deg s"),
    ("peak_abs_pitch_rate_deg_s", "Peak pitch rate", "deg/s"),
    ("final_axle_position_m", "Final axle position", "m"),
    ("final_axle_speed_m_s", "Final axle speed", "m/s"),
    ("peak_abs_axle_speed_m_s", "Peak axle speed", "m/s"),
    ("peak_gearbox_relative_speed_rpm", "Peak relative gearbox-output speed", "RPM"),
    ("peak_requested_total_torque_Nm", "Peak requested total torque", "N m"),
    ("peak_applied_total_torque_Nm", "Peak applied total torque", "N m"),
    ("saturated_duration_s", "Time clipped by torque cap", "s"),
    ("saturated_control_fraction", "Fraction of time clipped", "fraction"),
    ("total_torque_squared_effort_Nm2_s", "Squared-total-torque effort", "(N m)^2 s"),
]


@dataclass
class SavedRun:
    source: Path
    result: Result
    summary: dict
    csv_bytes: bytes
    summary_bytes: bytes

    @property
    def label(self):
        return self.source.name


def load_run(directory):
    directory = Path(directory).resolve()
    csv_bytes = (directory / "trajectory.csv").read_bytes()
    summary_bytes = (directory / "summary.json").read_bytes()
    summary = json.loads(summary_bytes.decode("utf-8"))
    reader = csv.DictReader(io.StringIO(csv_bytes.decode("utf-8-sig")))
    if reader.fieldnames is None or not set(COLUMNS).issubset(reader.fieldnames):
        raise ValueError(f"{directory}: trajectory.csv is missing expected columns.")
    try:
        values = np.array([[float(row[key]) for key in COLUMNS] for row in reader])
        robot_values = dict(summary["robot"])
        # Archived runs predate the measured axle offset; retain their geometry.
        robot_values.setdefault("body_bottom_below_axle_m", 0.0)
        robot_values.setdefault("wheel_center_separation_m", 0.125)
        robot = RobotParameters(**robot_values)
        experiment = Experiment(**summary["experiment"])
        fell = summary["metrics"]["fell"]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{directory}: invalid trajectory or settings: {error}") from error
    if values.ndim != 2 or len(values) < 2 or not isinstance(fell, bool):
        raise ValueError(f"{directory}: incomplete trajectory or invalid fall status.")
    result = Result(robot, experiment, values[:, 0], values[:, 1:5], values[:, 5], values[:, 6], fell)
    fresh = result.metrics()  # Validates finite values, shapes, and increasing time.
    if not np.isclose(result.time_s[0], 0, atol=1e-9):
        raise ValueError(f"{directory}: this report expects experiments starting at t=0.")
    if not np.allclose(np.diff(result.time_s), experiment.control_period_s, rtol=0, atol=1e-8):
        raise ValueError(f"{directory}: time samples do not match the recorded control period.")
    if not np.isclose(np.rad2deg(result.state[0, 2]), experiment.initial_tilt_deg, atol=1e-7):
        raise ValueError(f"{directory}: starting angle and summary settings disagree.")
    if abs(result.time_s[-1] - experiment.duration_s) > 1e-8 and not fell:
        raise ValueError(f"{directory}: a non-fallen run is shorter than its configured duration.")
    if result.time_s[-1] > experiment.duration_s + 1e-8:
        raise ValueError(f"{directory}: recording exceeds its configured duration.")
    if fell != (abs(np.rad2deg(result.state[-1, 2])) >= experiment.fall_angle_deg):
        raise ValueError(f"{directory}: fall flag and terminal tilt disagree.")
    for key in ("final_tilt_deg", "final_axle_position_m", "final_axle_speed_m_s"):
        if key in summary["metrics"] and not np.isclose(fresh[key], summary["metrics"][key], atol=1e-7):
            raise ValueError(f"{directory}: CSV and summary disagree on {key}; save the run again.")
    return SavedRun(directory, result, summary, csv_bytes, summary_bytes)


def comparison_notes(runs):
    notes = []
    first = runs[0]
    for run in runs[1:]:
        differing = []
        if run.summary.get("model") != first.summary.get("model") or run.summary.get("limitations") != first.summary.get("limitations"):
            differing.append("model description/limitations")
        if run.summary["robot"] != first.summary["robot"]:
            differing.append("robot parameters")
        for key in ("initial_tilt_deg", "duration_s", "control_period_s", "physics_step_s",
                    "total_torque_limit_Nm", "fall_angle_deg"):
            if getattr(run.result.experiment, key) != getattr(first.result.experiment, key):
                differing.append(key)
        if differing:
            notes.append(f"{run.label} versus {first.label}: conditions differ ({', '.join(differing)}).")
        else:
            notes.append(f"{run.label} versus {first.label}: model, starting condition, timing, duration, and torque cap match.")
        a, b = first.result.experiment, run.result.experiment
        if a.kp != b.kp and a.kd != b.kd:
            notes.append(f"Both P and D differ between {first.label} and {run.label}; do not attribute the response difference to one gain alone.")
        if len(first.result.time_s) != len(run.result.time_s):
            notes.append("Recorded windows differ; RMS and integral metrics cover different durations and should not be ranked directly.")
    return notes


def _format(value):
    if value is None:
        return "Not observed / undefined"
    if isinstance(value, bool):
        return "Yes" if value else "No"
    if isinstance(value, int):
        return str(value)
    return f"{value:.6g}"


def save_report(runs, output_dir):
    output_dir = Path(output_dir).resolve()
    if any(output_dir == run.source or output_dir in run.source.parents for run in runs):
        raise ValueError("Choose a report directory separate from, and not containing, the source runs.")
    output_dir.mkdir(parents=True, exist_ok=True)
    snapshot_dir = output_dir / "input-data"
    snapshot_dir.mkdir(exist_ok=True)
    notes = comparison_notes(runs)
    records = []
    lines = ["# Saved-run evaluation", "", "Metrics are recomputed from the saved trajectories without smoothing or rerunning the simulation.", "",
             "## Settings", "", "| Run | Controller | P (N m/rad) | D (N m s/rad) | Initial tilt (deg) | Total torque cap (N m) |",
             "| --- | --- | --- | --- | --- | --- |"]
    for index, run in enumerate(runs, 1):
        p = run.result.experiment
        label = f"{index}: {run.label}".replace("|", "\\|").replace("\n", " ")
        lines.append(f"| {label} | {p.controller} | {p.kp:.6g} | {p.kd:.6g} | {p.initial_tilt_deg:g} | {p.total_torque_limit_Nm:g} |")
        (snapshot_dir / f"run-{index}-trajectory.csv").write_bytes(run.csv_bytes)
        (snapshot_dir / f"run-{index}-summary.json").write_bytes(run.summary_bytes)
        records.append({"label": label, "source": str(run.source),
                        "csv_sha256": hashlib.sha256(run.csv_bytes).hexdigest(),
                        "summary_sha256": hashlib.sha256(run.summary_bytes).hexdigest(),
                        "metrics": run.result.metrics()})
    lines += ["", "## Results", "", "| Metric | Units | " + " | ".join(r["label"] for r in records) + " |",
              "| --- | --- | " + " | ".join("---" for _ in records) + " |"]
    for key, label, unit in METRICS:
        lines.append(f"| {label} | {unit} | " + " | ".join(_format(r["metrics"][key]) for r in records) + " |")
    lines += ["", "## Comparison conditions", ""]
    lines += [f"- {note}" for note in notes] if notes else ["One saved run evaluated; no cross-run comparison."]
    lines += ["", "## Definitions and limits", "",
              "- Settling: tilt stays within +/-1 degree and pitch rate within +/-5 degrees/s through the end, for at least 0.5 s. A fallen run has no settling time.",
              "- Overshoot: maximum tilt on the opposite side of upright from the initial lean. Percent overshoot divides by the absolute initial lean; undefined when the initial lean is zero.",
              "- Upright crossings count only excursions reaching +/-0.1 degree. This describes oscillation, not a frequency estimate.",
              "- RMS tilt and absolute-error integral use trapezoidal interpolation over the actual recorded duration.",
              "- Torque clipping duration and squared-total-torque effort use held commands over each control interval; the final logged command is excluded.",
              "- Squared torque is a control-effort proxy, not electrical energy, power, battery use, or efficiency.",
              "- Final axle speed exposes drift after pitch recovery. There is no position/speed controller in this experiment.",
              "- One recovery trajectory is evidence about that test, not a proof of global stability or physical hardware performance.",
              "- The source model uses ideal sensing and bounded instantaneous torque. Driver losses, motor dynamics, backlash, and sensor noise/delay remain absent.",
              "", "## Traceability", "",
              "Input CSV and summary snapshots are stored in input-data/. The JSON report records their SHA-256 hashes, so a later GUI save to outputs/latest does not change this report's inputs.",
              "", "![Saved-run comparison](comparison.png)", ""]
    (output_dir / "report.md").write_text("\n".join(lines), encoding="utf-8")
    (output_dir / "metrics.json").write_text(json.dumps({"runs": records, "comparison_notes": notes}, indent=2) + "\n", encoding="utf-8")
    keys = [key for key, _, _ in METRICS]
    with (output_dir / "metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["run", "source", *keys])
        writer.writeheader()
        for record in records:
            writer.writerow({"run": record["label"], "source": record["source"], **{k: record["metrics"][k] for k in keys}})
    save_comparison_plot(runs, output_dir)
    return output_dir / "report.md"


def save_comparison_plot(runs, output_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    fig.subplots_adjust(top=0.81, bottom=0.12, hspace=0.4, wspace=0.3)
    for index, run in enumerate(runs, 1):
        r = run.result
        label = f"{index}: {run.label} (P={r.experiment.kp:.4f}, D={r.experiment.kd:.5f})"
        axes[0, 0].plot(r.time_s, np.rad2deg(r.state[:, 2]), label=label)
        axes[0, 1].plot(r.time_s, np.rad2deg(r.state[:, 3]))
        axes[1, 0].plot(r.time_s, r.state[:, 0])
        axes[1, 1].plot(r.time_s, r.applied_torque_Nm)
    panels = [("Tilt and overshoot", "Tilt (deg)"), ("Angular motion", "Pitch rate (deg/s)"),
              ("Position drift", "Axle position (m)"), ("Applied actuation", "Total torque (N m)")]
    for ax, (title, ylabel) in zip(axes.flat, panels):
        ax.set(title=title, xlabel="Time (s)", ylabel=ylabel)
        ax.grid(alpha=0.2)
        ax.axhline(0, color="0.5", linewidth=0.6)
    axes[0, 0].axhspan(-1, 1, color="0.5", alpha=0.06)
    fig.suptitle("Post-processing of saved PD experiments", fontsize=13, y=0.98)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.945), ncol=1, fontsize=9, frameon=False)
    fig.text(0.5, 0.035, "Recorded ideal-model trajectories; no smoothing. Gains, assumptions, metrics, and input snapshots accompany this figure.",
             ha="center", fontsize=8)
    fig.savefig(Path(output_dir) / "comparison.png", dpi=150)
    plt.close(fig)
