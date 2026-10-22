# GPU Bayesian controller optimization

**Current workflow:** use the [IMU-only PID optimization guide](imu-only-control.md).
Kp, Kd and Ki are searched; wheel gains stay zero. The four-gain procedures and
results below are historical and relied on feedback absent from this hardware.

The current search builds and calls the shared C++ controller on the CPU;
MuJoCo Warp runs physics on CUDA. State/command transfers are included in the
timed evaluation loop. The earlier recorded 64-trial experiment below used the
pre-refactor Warp controller. A 24-trial integration check in
`outputs/shared-cpp-bayes-20261022` completed using the shared core. Its candidate
has now passed independent nominal Gazebo balance and movement checks, recorded
in [the validation results](../experiments/shared-cpp-bayes-validation-20261022.json).
At the owner's request, default runs automatically update the normal simulation
settings after validation. The later [paired BO/DE comparison](optimizer-comparison.md)
also follows this rule; the active gains always live in `config/balance.yaml`.

Run these commands in the project's Ubuntu/WSL terminal:

```bash
bash setup_gpu.sh   # once, or to update the optional environment
bash optimize.sh
```

Choose **4 → 3** (Advanced → Optimize balance gains) in the menu to run the same
search. It runs headlessly and prints progress; no graphics window is expected.
Default: 64 gain sets, eight candidates per GPU batch, eight simulated seconds
per scenario. It then builds the ROS package and checks the winner in four
12-second Gazebo recovery tests (+/-5 and +/-10 degrees). A passing winner
automatically replaces the four active gains in `config/balance.yaml`.
The normal menu's Gazebo/MuJoCo launches use those values when restarted.
A larger run is optional:

```bash
bash optimize.sh --trials 128 --batch-size 8 --seed 42
```

Each run creates a fresh timestamped folder under `outputs/`. You may specify
`--output outputs/my-search`, provided that folder does not already exist.
Closing or interrupting the search preserves completed batches in `trials.json`;
automatic resume is not implemented. Failed GPU or Gazebo checks keep the
previous gains. Successful updates preserve comments and all unrelated settings,
including movement speed, yaw gains and torque limits. They save
`previous-active.yaml`, `applied-profile.yaml` and `promotion.json` alongside the
search results. A result for an older model/configuration is rejected rather
than overwriting changes made during the search.

Automatic validation concerns balancing and does not require a particular
`my_routine.cpp` movement sequence. The round-trip recorder below is an optional
extra check for the default shuttle. To run only GPU search without validation
or active updates, explicitly use `bash optimize.sh --search-only`.

## What gets optimized

The four gains are searched within these explicit bounds:

| Parameter | Minimum | Maximum |
| --- | ---: | ---: |
| `kp` | 0.30 | 1.50 |
| `kd` | 0.020 | 0.140 |
| `position_gain` | 0.05 | 0.50 |
| `velocity_gain` | 0.15 | 0.60 |

The current gains are trial zero. The initial 16 trials include that incumbent
and a scrambled Sobol design. Later batches use a CPU Gaussian process with a
Matérn 5/2 kernel, fitted to `log(1 + cost)`. Analytic expected improvement (EI)
selects promising points from Sobol and local candidate pools. A distance
exclusion provides diversity within each batch. This is batched single-point
EI, not joint qEI. Constraints enter as penalties, rather than a separate
feasibility model. Contact physics runs on CUDA; shared C++ control and the
fixed-bus actuator adapter run on the CPU.

The fixed objective rewards small tilt, limited displacement, low speed and
lower predicted average motor current. Its final-second terms penalize residual
tilt, pitch rate, displacement and speed. In normalized units, each case costs:

```text
0.60 * mean(pitch²) / (5 deg)²
+ 0.25 * mean(distance²) / (0.10 m)²
+ 0.05 * mean(speed²) / (0.20 m/s)²
+ 0.05 * (peak average motor current / 0.60 A)²
+ 1.50 * tail_mean(pitch²) / (1 deg)²
+ 0.30 * tail_mean(pitch_rate²) / (5 deg/s)²
+ 0.50 * tail_mean(distance²) / (0.02 m)²
+ 0.50 * tail_mean(speed²) / (0.02 m/s)²
+ feasibility penalties
```

The candidate score is `mean(case costs) + 0.75 * worst case cost`; lower is
better. Weights express the design priorities, not a universal optimum.
Each failing case receives 1000 points plus 50 times the sum of squared
normalized constraint exceedances. Nonfinite cost is replaced by 1,000,000.

Training starts: tilt ±5° and ±10°, forward velocity ±0.08 m/s, or pitch rate
±10°/s. Each velocity/rate case starts upright. Every candidate sees every case.
Selection uses only these training cases, preferring candidates that pass all
cases. If none pass, a diagnostic candidate is still saved with a failure flag.

That is **eight simulations per parameter set**, each lasting eight simulated
seconds by default. Each starting condition runs once; there are no deliberate
random repeats or sampled changes to mass, battery voltage, friction or sensor
noise in this GPU search. A batch of eight candidates therefore uses 64 robot
worlds. Optimizer seeds change the proposed gains, not these test conditions.
Increasing the trial budget explores more gains under the same nominal model;
it does not add hardware uncertainty or optimize the movement routine.

Only after selection, the current and selected gains are evaluated for at least
12 seconds at held-out tilt ±7°, velocity ±0.12 m/s and pitch rate ±15°/s.
These six cases do not choose the winner or fit the surrogate.

Feasibility requires finite state/metrics, no latched fall fault, maximum total
body tilt ≤20°, final-second pitch ≤1° and pitch rate ≤5°/s, final distance
≤0.05 m, final speed ≤0.03 m/s, and peak predicted average motor current ≤0.60 A.
The existing total torque request cap is fixed (currently 0.040 N m).

A 16-trial run contains only the initial design; use more than 16 trials to
include Bayesian proposals. The default 64-trial run includes both stages.

## Inspect and validate the result

The printed output path contains:

- `search.json`: incumbent, winner, scores, all feasibility/holdout results,
  surrogate fits and package/device/source metadata.
- `trials.json`: every completed trial, its gains and per-case measurements.
- `candidate.yaml`: complete ROS parameter profile for the selected candidate.
- `current.yaml`: complete profile of the original settings.
- `run.json` and `sources/`: settings, source snapshots and SHA-256 hashes.
- `gazebo-validation/`: automatic nominal Gazebo response recordings.
- `promotion.json`: final update status; `search.json` describes the GPU stage.
- `previous-active.yaml` and `applied-profile.yaml`: profiles before/after an update.

After a successful default run, launch `bash run_menu.sh` normally; no special
profile argument is needed. Replace `outputs/my-search` below with the printed
folder for optional comparisons or `--search-only` results. View exactly the
candidate gains in every GPU world:

```bash
bash run_gpu.sh --view --params-file outputs/my-search/candidate.yaml
```

For Gazebo's ROS demo, launch with an absolute profile path:

```bash
bash run_gazebo.sh "params_file:=$(realpath outputs/my-search/candidate.yaml)"
```

The default workflow already records nominal recovery checks. To reproduce
those separately or add the default shuttle's movement check:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select balancing_robot_sim
source install/setup.bash
cd ..
python3 analysis/validate_tuning.py --profiles candidate \
  --candidate outputs/my-search/candidate.yaml \
  --cases nominal_5deg nominal_minus5deg nominal_10deg nominal_minus10deg \
  --output outputs/my-search/gazebo
python3 analysis/check_shuttle.py --cycles 1 \
  --params-file outputs/my-search/candidate.yaml \
  --output outputs/my-search/shuttle
```

The validator's historical `baseline` option means the original controller,
not this search's current settings. To compare the current profile in Gazebo,
use `--profiles candidate --candidate outputs/my-search/current.yaml` with a
different output folder. `validation.json` records each run's response verdict;
the validator preserves failed records. The standalone validation tools only
record results; `apply_optimized_gains.py`, called by the optimizer, performs
the gain update after verifying the evidence.

## First recorded search, 2026-10-22

The 64-trial run on the RTX 4060 Laptop took about 29 seconds for search/setup
and held-out evaluation after imports. It found 59 training-feasible trials.
Trial 49 scored 0.066975 versus the current profile's 0.090156 (25.7% lower).
The held-out score was 0.041756 versus 0.051546 (19.0% lower); both passed all
six held-out cases. These percentages concern the defined objective.

The selected gains were `kp=0.522365`, `kd=0.0241934`,
`position_gain=0.467018`, `velocity_gain=0.411320`, with the 0.040 N m cap.
Independent Gazebo checks passed +5°, -5° and +10°, but **failed -10°** with a
latched fall fault. The candidate completed the shuttle with measured legs
0.49684 m and 0.49652 m, and a 0.341° half-turn error. A better nominal GPU
score therefore did not establish reliable recovery in Gazebo.

This historical candidate was **not promoted at that stage**; the then-active
gains were retained. The later shared C++ results are recorded below.
The failed recording is preserved alongside the successful ones in
`outputs/gpu-bayes-20261022/`. Selected evidence is retained for GitHub in
[the result JSON](../experiments/gpu-bayes-20261022.json) and
[the candidate profile](../experiments/gpu-bayes-20261022-candidate.yaml).
The search's `gazebo_validation_passed` field stays null because that file
describes the GPU search; `validation-summary.json` records the subsequent
Gazebo failure. Future searches also need these independent checks.

## Shared C++ candidate validation, 2026-10-22

The existing 24-trial candidate scored 19.58% lower on the weighted training
objective and passed all six GPU holdout cases. This is a score improvement,
not a percentage improvement in speed. Its gains are `kp=1.23464709`,
`kd=0.09620997`, `position_gain=0.30410390`, `velocity_gain=0.57996656`,
with the same combined request cap of 0.040 N m.

Independent Gazebo tests passed all four initial tilts (+5, -5, +10 and -10
degrees) over 12 simulated seconds each. The largest final-second pitch
magnitude across these cases was 0.01942 degrees. A complete drive/stop/turn/
return/stop check also passed, finishing at 36.645 simulated seconds. Independent
world poses measured straight legs of 0.49803 m and 0.49803 m, with 0.3386 degrees
of half-turn error.

The full logs are under `outputs/shared-cpp-bayes-20261022/gazebo-validation/`
and `shuttle-validation/`. Selected measurements and source hashes are saved in
[the validation JSON](../experiments/shared-cpp-bayes-validation-20261022.json),
with [a separate candidate profile](../experiments/shared-cpp-bayes-20261022-candidate.yaml)
for reproducing it. Search metadata remains historical; the later checks are in
`validation-summary.json`. At that stage the active profile hash still matched
the pre-search profile. These nominal checks do not establish robustness to component/sensor
uncertainty or transfer to the Nano, whose position/speed feedback is unavailable.

## 64-trial shared C++ search, 2026-10-22

The owner's default `bash optimize.sh` run is recorded in
`outputs/gpu-optimization-20261022-132737-9729`. It completed 64 trials in
26.37 seconds for the timed search/evaluation stage, with 59 training-feasible
trials. Trial 49 scored 0.0670059 versus the then-active profile's 0.0901352,
a 25.66% lower training objective. Both profiles passed all six holdout cases;
the candidate holdout cost was 0.0417988 versus 0.0516059.

This candidate uses `kp=0.52236481`, `kd=0.02419345`,
`position_gain=0.46701845` and `velocity_gain=0.41131975`, with the same
0.040 N m combined request cap. These match the earlier search's gains, whose
historical -10-degree failure is retained above. That older Gazebo recording
used a different controller revision. Current checks use the shared C++ core;
the results do not establish which implementation/timing difference caused
the changed outcome.

All four nominal Gazebo starts (+/-5 and +/-10 degrees) passed 12-second
checks, and a separate repeat of -10 degrees also passed. The movement routine
passed, finishing its round trip at 32.985 simulated seconds. Independent world
poses measured legs of 0.496817 m and 0.496661 m, with 0.3658 degrees of half-turn
error. These finite nominal checks do not establish general robustness.

[Selected results and source hashes](../experiments/shared-cpp-bayes64-validation-20261022.json)
and [the candidate profile](../experiments/shared-cpp-bayes64-20261022-candidate.yaml)
are saved for reproduction. These gains were subsequently made active at the
owner's request, using the same verified recordings. The update preserved all
other settings and saved a backup of the previous profile. See
[the automatic-update record](../experiments/automatic-gain-update-20261022.json).
Launch the regular menu to use them:

```bash
bash run_menu.sh
```

## Limits

This optimizes **balancing**, not waypoint speed, turning or acceleration.
Geometry, nominal battery/driver/motor assumptions and sensing are fixed.
Varying starting states does not establish robustness to mass, backlash,
battery sag, sensor noise or ROS transport delays. The GPU backend supports
fixed-bus, quasi-static coast PWM; it rejects unsupported actuator settings.

Gazebo and hardware may rank gains differently. Current and thermal estimates
do not establish L293D pulse-rating compliance. The best observed controller
within these bounds and this budget is not proof of global optimality. Automatic
updates require the independent nominal response checks; customized movement
and physical hardware still need their own evaluation.

Primary references: [scikit-learn Gaussian processes](https://scikit-learn.org/stable/modules/gaussian_process.html),
[Matérn kernel](https://scikit-learn.org/stable/modules/generated/sklearn.gaussian_process.kernels.Matern.html).
