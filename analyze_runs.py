"""Evaluate saved runs without rerunning or changing their trajectories."""

import argparse
from pathlib import Path

from analysis.postprocessing import load_run, save_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path, help="Folders containing trajectory.csv and summary.json")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "outputs" / "reports" / "latest",
                        help="Separate report folder; files there are replaced on repeated runs")
    args = parser.parse_args()
    if len(args.runs) > 4:
        parser.error("Compare up to four runs at a time so the plots and report remain readable.")
    try:
        runs = [load_run(path) for path in args.runs]
        report = save_report(runs, args.output)
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.exit(2, f"Could not evaluate saved runs: {error}\n")
    for run in runs:
        m = run.result.metrics()
        print(f"{run.label}: settling={m['pitch_settling_time_s']} s, "
              f"overshoot={m['opposite_side_overshoot_deg']} deg, "
              f"final speed={m['final_axle_speed_m_s']:.4f} m/s")
    print(f"Report: {report}")


if __name__ == "__main__":
    main()
