"""Plot saved ROS motor-model responses without requiring ROS or rerunning them."""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load(folder, filename):
    data = np.atleast_1d(np.genfromtxt(folder / filename, delimiter=",", names=True))
    if not len(data):
        raise ValueError(f"Empty data: {folder / filename}")
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("nominal", type=Path)
    parser.add_argument("stress", type=Path)
    parser.add_argument("--ideal", type=Path)
    parser.add_argument("--output", type=Path, default=Path("outputs/motor-model-comparison.png"))
    args = parser.parse_args()
    nominal = load(args.nominal, "telemetry.csv")
    stress = load(args.stress, "telemetry.csv")
    motors = load(args.nominal, "motors.csv")
    curves = [(nominal, "Nominal TT / L293D", "#1769aa"),
              (stress, "Assumed 8% deadband + 2 ms lag", "#c74b25")]
    if args.ideal:
        curves.insert(0, (load(args.ideal, "telemetry.csv"), "Ideal actuator, same gains", "#4d9561"))
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for data, label, color in curves:
        axes[0, 0].plot(data["time_s"], np.degrees(data["pitch_rad"]), label=label, color=color)
        axes[0, 1].plot(data["time_s"], data["x_m"], label=label, color=color)
    axes[0, 0].axhspan(-1, 1, color="#dceadd", alpha=0.6)
    axes[0, 0].set(title="Body pitch", ylabel="Pitch (deg)")
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].set(title="Wheel-odometry position", ylabel="Position (m)")
    axes[1, 0].plot(nominal["time_s"], nominal["commanded_nm"],
                    label="Bounded controller request", color="#8a6b98", linestyle="--")
    axes[1, 0].plot(motors["time_s"], motors["applied_left_nm"] + motors["applied_right_nm"],
                    label="Delivered by motor model", color="#1769aa", alpha=0.85)
    axes[1, 0].set(title="Nominal combined wheel torque", ylabel="Torque (N m)")
    axes[1, 0].legend(fontsize=8)
    mean_current = np.maximum(abs(motors["current_left_a"]), abs(motors["current_right_a"]))
    on_current = np.maximum(abs(motors["on_left_est_a"]), abs(motors["on_right_est_a"]))
    axes[1, 1].plot(motors["time_s"], mean_current, label="Largest |average motor current|", color="#1769aa")
    enabled_on_current = np.where(on_current > 1e-12, on_current, np.nan)
    axes[1, 1].plot(motors["time_s"], enabled_on_current,
                    label="ON estimate when PWM is enabled", color="#bd763b",
                    linewidth=0.8, marker=".", markersize=1, alpha=0.75)
    axes[1, 1].axhline(0.6, color="#b43b3b", linestyle="--", label="0.6 A continuous rating")
    axes[1, 1].axhline(1.2, color="#777777", linestyle=":", label="1.2 A nonrepetitive peak rating")
    axes[1, 1].set(title="Nominal current estimates per motor", ylabel="Current magnitude (A)", ylim=(-0.03, 1.32))
    axes[1, 1].legend(fontsize=7.5, loc="upper right")
    for axis in axes.flat:
        axis.set_xlabel("Simulation time (s)")
        axis.set_xlim(0, 10)
        axis.grid(alpha=0.2)
    fig.suptitle("Motor / driver model: nominal recovery and sensitivity", fontsize=15)
    fig.text(0.5, 0.015,
             "Reference-component simulation. PWM ON current is an estimate; hardware rating compliance is unverified.",
             ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.045, 1, 0.95))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)
    print(args.output)


if __name__ == "__main__":
    main()
