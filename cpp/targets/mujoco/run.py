"""MuJoCo I/O adapter: behaviour and control execute in the shared C++ library."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "ros2_ws/src/balancing_robot_sim/scripts"))
sys.path.insert(0, str(ROOT / "gpu"))
from motor_model import MotorDriverModel, MotorParameters
from shared_core import Controller, ImuEstimator, LIBRARY_PATH, PlanarOdometry, attitude, yaw_rate
from motion_core import MotionState, Shuttle, ShuttleSettings
from robot_model import GEOMETRY_PATH, MODEL_PATH, load_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--shuttle", action="store_true")
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--seconds", type=float, default=0, help="0: GUI until closed; headless defaults to 12/120 seconds")
    parser.add_argument("--tilt", type=float, default=5)
    parser.add_argument("--params-file", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.shuttle:
        parser.error("The measured-distance routine needs wheel feedback; this robot is IMU-only")
    if not math.isfinite(args.seconds) or args.seconds < 0 or not math.isfinite(args.tilt) or abs(args.tilt)>10:
        parser.error("Need nonnegative seconds and finite tilt within +/-10 degrees")
    if args.output.exists():
        parser.error("Choose a fresh recording folder")
    config_path = args.params_file or ROOT/"ros2_ws/src/balancing_robot_sim/config/balance.yaml"
    config = yaml.safe_load(config_path.read_text())
    p = config["balance_controller"]["ros__parameters"]
    p.setdefault("reference_lead_limit_m", .08)
    g = json.loads(GEOMETRY_PATH.read_text())
    radius = g["wheel_radius_m"]
    fields = MotorParameters.__dataclass_fields__
    motor = MotorDriverModel(MotorParameters(**{k:v for k,v in config["motor_driver"]["ros__parameters"].items() if k in fields}))
    controller, odometry, estimator = Controller(p), PlanarOdometry(), ImuEstimator()
    routine = Shuttle(ShuttleSettings(cycles=args.cycles, **{
        k:v for k,v in config["shuttle_demo"]["ros__parameters"].items() if k in ShuttleSettings.__dataclass_fields__ and k!="cycles"}))
    model, rows = load_model(), []
    data = mujoco.MjData(model)
    initial = math.radians(args.tilt)
    delayed_readings = np.array([-math.sin(initial), 0, math.cos(initial), 0, 0, 0])
    estimator.update(delayed_readings)
    data.qpos[3], data.qpos[5] = math.cos(initial/2), math.sin(initial/2)
    mujoco.mj_forward(model, data)
    args.output.mkdir(parents=True)
    sources = [Path(__file__), LIBRARY_PATH, config_path.resolve(), GEOMETRY_PATH, MODEL_PATH,
               ROOT/"gpu/robot_model.py", ROOT/"ros2_ws/src/balancing_robot_sim/scripts/shared_core.py",
               ROOT/"ros2_ws/src/balancing_robot_sim/scripts/motor_model.py"]
    sources += [f for f in (ROOT/"cpp/src").rglob("*") if f.suffix in (".cpp", ".h")]
    hashes = {}
    for source in sources:
        relative = source.relative_to(ROOT)
        copied = args.output/"sources"/relative
        copied.parent.mkdir(parents=True, exist_ok=True)
        copied.write_bytes(source.read_bytes())
        hashes[relative.as_posix()] = hashlib.sha256(source.read_bytes()).hexdigest()
    (args.output/"run.json").write_text(json.dumps(dict(
        controller="shared C++", physics="CPU MuJoCo", shuttle=args.shuttle,
        initial_tilt_deg=args.tilt, source_sha256=hashes, hardware_validated=False), indent=2)+"\n")
    maximum = args.seconds or ((120 if args.shuttle else 12) if args.headless else math.inf)
    target_v = target_w = 0.
    frame = 0
    next_behaviour = 0.
    fault = 0
    done_time = None

    def tick():
        nonlocal target_v, target_w, next_behaviour, fault, done_time, delayed_readings
        q, v = data.qpos, data.qvel
        roll, pitch, heading = attitude(q[4], q[5], q[6], q[3])
        rate = v[4]
        distance = radius*((q[7]+q[8])/2+pitch-initial)
        speed = radius*((v[6]+v[7])/2+rate)
        heading_rate = yaw_rate(roll, pitch, rate, v[5])
        x, y, yaw = odometry.update(radius*((q[7]+q[8])/2+pitch), heading)
        if args.shuttle and data.time >= next_behaviour-1e-9:
            target_v, target_w = routine.update(MotionState(data.time,x,y,yaw,pitch,rate,speed,heading_rate))
            next_behaviour += .05
            if routine.phase == "DONE" and done_time is None:
                done_time = data.time
        tilt = math.acos(np.clip(1-2*(q[4]*q[4]+q[5]*q[5]),-1,1))
        readings = np.array(data.sensordata, copy=True)
        readings[:3] /= 9.80665
        estimated_pitch, measured_rate, estimated_tilt, measured_yaw, valid = estimator.update(delayed_readings)
        delayed_readings = readings
        output = controller.update(estimated_pitch,measured_rate,0,0,measured_yaw,estimated_tilt,
                                   0 if valid else 1,0,False,0,0,.005,False)
        requested, bounded, target, left, right, enabled, fault = output
        sample = motor.step([left,right], v[6:8], .005)
        data.ctrl[:] = [sample.left.torque_nm,sample.right.torque_nm]
        rows.append(dict(time_s=data.time,pitch_rad=pitch,rate_rad_s=rate,x_m=x,y_m=y,yaw_rad=yaw,
                         wheel_distance_m=distance,speed_m_s=speed,world_x_m=q[0],world_y_m=q[1],
                         requested_nm=requested,commanded_nm=bounded,target_rad=target,
                         estimated_pitch_rad=estimated_pitch,estimated_tilt_rad=estimated_tilt,
                         left_current_a=sample.left.current_a,right_current_a=sample.right.current_a,
                         phase=routine.phase if args.shuttle else "BALANCE",fault=int(fault)))
        for _ in range(5):
            mujoco.mj_step(model,data)

    def continuing():
        return data.time < maximum and (not args.headless or done_time is None or data.time<done_time+1)

    if args.headless:
        while continuing():
            tick()
    else:
        from mujoco import viewer as native_viewer
        with native_viewer.launch_passive(model,data) as viewer:
            viewer.cam.distance=.8
            viewer.cam.elevation=-20
            while viewer.is_running() and continuing():
                started=time.perf_counter()
                for _ in range(4):
                    tick()
                viewer.sync()
                frame+=1
                time.sleep(max(0,.02-(time.perf_counter()-started)))
    if rows:
        with (args.output/"telemetry.csv").open("w",newline="") as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
            writer.writeheader();writer.writerows(rows)
    tail=[r for r in rows if r["time_s"]>=data.time-1]
    balance_pass=bool(tail) and not any(r["fault"] for r in rows) and max(abs(r["pitch_rad"]) for r in tail)<math.radians(1)
    passed=balance_pass and (not args.shuttle or routine.phase=="DONE")
    report=dict(passed=passed,controller="shared C++ IMU-only",physics="CPU MuJoCo",simulated_seconds=data.time,
                feedback="accelerometer + gyro; shared complementary filter", wheel_feedback_used=False,
                final_drift_m=rows[-1]["wheel_distance_m"] if rows else None,
                final_phase=routine.phase if args.shuttle else "BALANCE",legs_completed=routine.legs_completed,
                legs=routine.legs,events=routine.events,all_states_finite=bool(np.isfinite(data.qpos).all()),
                tail_max_pitch_deg=math.degrees(max((abs(r["pitch_rad"]) for r in tail),default=math.inf)),
                source_sha256=hashes,hardware_validated=False)
    (args.output/"summary.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:report[k] for k in ("passed","controller","physics","simulated_seconds","final_phase","legs_completed","tail_max_pitch_deg")},indent=2))
    if args.headless and not passed:
        raise SystemExit(1)


if __name__=="__main__":
    main()
