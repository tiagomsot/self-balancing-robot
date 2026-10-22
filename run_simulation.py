"""Run the first PD experiment. No ROS installation or robot connection needed."""

import argparse
from pathlib import Path

from simulation.controllers import REFERENCE_KD, REFERENCE_KP
from simulation.runner import Experiment, save_data, simulate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=["reference", "student", "off"], default="reference")
    parser.add_argument("--kp", type=float, default=REFERENCE_KP, help="P gain in N m/rad")
    parser.add_argument("--kd", type=float, default=REFERENCE_KD, help="D gain in N m s/rad")
    parser.add_argument("--initial-tilt", type=float, default=5, help="Initial forward lean in degrees")
    parser.add_argument("--duration", type=float, default=5, help="Run duration in seconds")
    parser.add_argument("--torque-limit", type=float, default=0.06, help="Imposed TOTAL torque cap in N m")
    parser.add_argument("--headless", action="store_true", help="Save plots/data without opening a window")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "outputs" / "latest",
                        help="Output directory; files in this directory are replaced on save")
    args = parser.parse_args()
    try:
        experiment = Experiment(controller=args.controller, kp=args.kp, kd=args.kd,
                                initial_tilt_deg=args.initial_tilt, duration_s=args.duration,
                                total_torque_limit_Nm=args.torque_limit)
        result = simulate(experiment)
    except (ValueError, NotImplementedError) as error:
        parser.exit(2, f"{error}\n")

    # Select the backend before importing pyplot, so CI/headless mode needs no GUI.
    if args.headless:
        import matplotlib
        matplotlib.use("Agg")
    from simulation.plotting import ExperimentWindow, outcome_text, save_figure
    save_data(result, args.output)
    save_figure(result, args.output)
    print(outcome_text(result))
    print(f"Saved plots.png, trajectory.csv, and summary.json to {args.output.resolve()}")
    print("Model: ideal tilt/rate and torque actuation; pitch control can leave position drift.")
    if not args.headless:
        window = ExperimentWindow(result, args.output)
        window.show()


if __name__ == "__main__":
    main()
