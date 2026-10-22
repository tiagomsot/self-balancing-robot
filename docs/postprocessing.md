# Evaluate and compare saved experiments

The simulator now calculates expanded metrics whenever a run is saved. Restart an already-open simulator to load this change. Existing CSV recordings can be evaluated without rerunning the simulation or changing their saved data.

From the repository root:

```powershell
python analyze_runs.py outputs/latest
```

To compare two runs:

```powershell
python analyze_runs.py outputs/reference-5deg outputs/latest --output outputs/reports/pd-comparison
```

Use any run folders containing trajectory.csv and summary.json. The supplied reference-5deg folder was generated during framework verification; you can create your own baseline with:

```powershell
python run_simulation.py --headless --output outputs/my-baseline
```

## Report files

- report.md: readable settings, metric comparison, definitions, and comparison conditions.
- comparison.png: overlaid tilt, pitch rate, axle position, and applied torque.
- metrics.csv and metrics.json: metrics for further analysis.
- input-data/: copies of the exact source trajectories and summaries. SHA-256 hashes are recorded in metrics.json.

The default report folder is outputs/reports/latest. Repeating an analysis into the same report folder replaces those report files. Select a new --output folder to preserve a separate report. Your source run files are not changed.

## What the metrics tell you

| Metric | Meaning |
| --- | --- |
| Pitch settling time | Earliest time after which tilt and rate remain inside the specified bands through the end, for at least 0.5 s |
| Overshoot past upright | Largest lean to the opposite side from the initial lean; degrees and percent of initial magnitude |
| Significant upright crossings | Counts side changes reaching +/-0.1 degree, ignoring tiny residual chatter |
| RMS tilt | Time-weighted overall angle error during the recorded run |
| Integrated absolute tilt | Accumulated magnitude of angle error over time |
| Peak pitch rate | How quickly the body rotates at its fastest point |
| Final axle speed and position | Whether tilt recovery leaves the robot moving or displaced |
| Peak torque and clipped duration | How much actuation was requested and when the imposed cap changed it |
| Squared-total-torque effort | Integral of squared total applied torque, a control-effort proxy |

The target tilt is upright (zero) in the current experiment. Error metrics must be adjusted if future experiments introduce moving references. These metrics describe the recording as saved; no smoothing is applied.

The settling bands are +/-1 degree and +/-5 degrees/s. The minimum observation inside those bands is 0.5 s. A run reaching the fall cutoff has no settling time. Percent overshoot is undefined for a zero initial lean. A crossing count describes oscillation; it is not an estimate of oscillation frequency or damping ratio.

State-error integrals use trapezoidal interpolation. Actuator metrics use zero-order-held commands over each interval and exclude the unused final command from time/effort integrals. Squared torque is not electrical energy or efficiency: voltage and current are not recorded in this model.

## Compare like with like

The report checks model settings, robot parameters, initial tilt, planned duration, control period, physics step, torque cap, and fall cutoff. It flags mismatches and notes when both P and D changed. Recorded windows shortened by a fall also make integral/RMS comparisons less direct.

The current latest-versus-reference example changes both P and D. It can demonstrate a response tradeoff, but cannot isolate the effect of either gain. Neither one finite recovery test nor the report alone establishes global stability or hardware performance. The current model still omits motor/driver dynamics, backlash, deadband, and IMU noise/delay.

A useful next experiment is to restore the reference D gain, change only P, save to a new folder, and compare that run with your baseline.
