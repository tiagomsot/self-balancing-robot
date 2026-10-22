# Your first experiment: understand and implement PD

The framework is ready. Your contribution is the small function in [student_controller.py](../student_controller.py), followed by experiments and your interpretation of the results.

This is a Python model before ROS/Gazebo. It uses the documented body and wheel geometry, nonlinear pitch/translation dynamics, a 200 Hz controller, and 1 ms physics integration. It is a local program and never connects to the real robot.

## 1. Run the working example

Open PowerShell in the repository folder:

```powershell
cd D:\Work\Portofolio\self-balancing-robot
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe run_simulation.py
```

If your current Python already has the dependencies, `python run_simulation.py` is sufficient. The explicit .venv Python commands work without activating PowerShell scripts.

The window contains:

- A side-view replay: the green point is the IMU, and the cross is the centre of mass. Dashed outlines mark the starting pose.
- Tilt over time, with a 1-degree band.
- Axle position, to make drift visible.
- Requested and applied combined wheel torque, with the imposed limit.

Change P, D, or starting tilt, then press **Run with gains**. **Replay** replays the current result. **Save plots + CSV** saves the current run. Changing a slider alone does not change the current run. By default the program saves its initial run to outputs/latest; saving again replaces the files there.

The reference run starts at 5 degrees. With the updated 420 g geometry, the
model-informed defaults are P = 0.1558734 and D = 0.00742505. They place the
ideal continuous pitch poles at -8 +/- 8i. The older 0.219203 / 0.0124865 gains
and preview describe the initial geometry. These delivered-torque gains are
separate from the ROS motor-model tuning, and are not hardware tuning.

## 2. Write your PD function

Open [student_controller.py](../student_controller.py). Its inputs are:

| Input | Meaning | Units |
| --- | --- | --- |
| angle_rad | Estimated body tilt; positive means leaning forward | rad |
| angular_rate_rad_s | How quickly that tilt is changing | rad/s |
| kp | Proportional gain | N m/rad |
| kd | Rate-feedback gain | N m s/rad |
| target_angle_rad | Desired tilt, initially upright | rad |

The angle error is already calculated. Complete the three TODOs: form a contribution from the angle error, form a contribution from angular rate, and return the combined requested torque.

Work out the signs using this convention: a forward lean should ask the wheels to move forward. If the robot is leaning forward but rotating back toward upright, the rate term should reduce the forward correction.

Then run:

```powershell
python run_simulation.py --controller student
```

The unfinished function deliberately stops with a message rather than silently using the reference controller. A complete example is available in simulation/controllers.py if you need it. Restart the program after editing your function.

The return value is TOTAL torque for both motors, in N m. It is not PWM and is not torque per motor. Timing, saturation, and the physics integrator are already handled by the framework.

## 3. Run controlled comparisons

Keep the starting tilt at 5 degrees and keep the torque cap unchanged. Compare against the baseline, changing only one gain at a time:

1. Raise P by 25%, keeping D fixed.
2. Restore P; raise D by 25%.
3. Restore the baseline; set D to zero.
4. Restore both gains; reduce the torque cap in a separate experiment.

Use separate output folders to preserve each result. For example:

```powershell
python run_simulation.py --controller reference --headless --output outputs/baseline
python run_simulation.py --controller reference --kp 0.2740034 --headless --output outputs/higher-p
python run_simulation.py --controller reference --kd 0.0156081 --headless --output outputs/higher-d
python run_simulation.py --controller reference --kd 0 --headless --output outputs/no-d
python run_simulation.py --controller reference --torque-limit 0.005 --headless --output outputs/weak-actuator
```

These commands use the reference to make the framework immediately usable. Change `reference` to `student` once your function works. Pass `--controller off` to see the unpowered model fall.

Each run saves plots.png, trajectory.csv, and summary.json. The summary includes settings, metrics, and assumptions. The reported pitch settling time is the earliest sampled time after which tilt remains within 1 degree and pitch rate within 5 degrees/s through the end, for at least 0.5 s. A fall cutoff is not a ground-contact simulation or a proven physical recovery limit.

For expanded metrics and overlaid comparisons of saved runs, use [the post-processing guide](postprocessing.md). For example, `python analyze_runs.py outputs/baseline outputs/higher-p` evaluates the saved data and preserves input snapshots in its report folder.

## 4. Explain what happened

Write your observations in [experiments/first-pd.md](../experiments/first-pd.md). Useful questions:

- Did the tilt recover faster? Did it overshoot or oscillate?
- Did the controller request more torque than the imposed cap allowed?
- Did the robot keep moving after its tilt recovered?
- What tradeoff would you choose, and what evidence supports it?

Avoid concluding that a larger gain is always better from one experiment. Preserve the settings that produced each plot.

## Limits of this first model

The controller receives perfect tilt and rate directly from the model. The IMU is drawn in its documented location, but its electrical output and accelerometer offset effects are not simulated yet. Motors deliver instantaneous torque up to a chosen scenario cap of +/-0.060 N m combined. This cap is not a verified motor rating or an automatic L293D current limit.

Driver voltage loss, torque-speed dependence, motor response, deadband/backlash, sensor noise/delay, yaw, and speed/position control are later steps. The simple pitch controller can settle while the robot keeps drifting; the plots intentionally preserve that result.

## Check the framework

```powershell
python -m unittest discover -s tests -v
```

The checks cover mechanical power balance, upright equilibrium, the earlier model's linearization, free fall, pitch recovery, actuator saturation, symmetry, physics-step convergence, and data export.
