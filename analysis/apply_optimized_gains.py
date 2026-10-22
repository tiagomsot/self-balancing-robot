"""Validate a GPU search winner, then update the gains used by normal launches."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gpu.optimization_core import BOUNDS, NAMES, search_values

ACTIVE = Path("ros2_ws/src/balancing_robot_sim/config/balance.yaml")
GEOMETRY = Path("ros2_ws/src/balancing_robot_sim/config/robot_geometry.json")
NATIVE = Path("cpp/build/librobot_core.so")
GAZEBO_NATIVE = Path("ros2_ws/install/balancing_robot_sim/lib/balancing_robot_sim/librobot_core.so")
CASES = ("nominal_5deg", "nominal_minus5deg", "nominal_10deg", "nominal_minus10deg")
BALANCE_CHECKS = ("completed_duration", "no_controller_fault", "initial_condition", "stays_upright",
                  "settles_pitch", "settles_rate",
                  "respects_command_limit")


class UpdateRejected(RuntimeError):
    pass


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):
    return json.loads(path.read_text())


def require(condition, message):
    if not condition:
        raise UpdateRejected(message)


def check_search(folder, root):
    search = read_json(folder / "search.json")
    baseline = yaml.safe_load((folder / "current.yaml").read_text())
    candidate = yaml.safe_load((folder / "candidate.yaml").read_text())
    selected = search["selected"]
    require(selected["all_training_cases_pass"] and search["candidate_holdout"]["passed"],
            "Candidate did not pass all GPU training and holdout cases.")
    costs = (search["baseline"]["objective"], selected["objective"])
    require(all(math.isfinite(x) for x in costs) and costs[1] <= costs[0] + 1e-12,
            "Candidate score is non-finite or worse than the incumbent.")
    gains = selected["gains"]
    require(search.get("feedback_mode") == "imu_only" and gains[2:4] == [0, 0],
            "Search must use IMU-only feedback and keep both wheel gains zero.")
    require(baseline["balance_controller"]["ros__parameters"].get("wheel_feedback_enabled") is False,
            "Active hardware profile must explicitly disable wheel feedback.")
    require(len(gains) == 6 and all(math.isfinite(x) for x in gains), "Invalid gain vector.")
    require(all(low <= value <= high for value, (low, high) in zip(search_values(gains), BOUNDS)),
            "Candidate gains are outside the search bounds.")
    expected = json.loads(json.dumps(baseline))
    expected["balance_controller"]["ros__parameters"].update(dict(zip(NAMES, search_values(gains))))
    cap = baseline["balance_controller"]["ros__parameters"]["total_torque_limit_nm"]
    require(0 < cap <= .040000001 and gains[4] == cap,
            "This workflow must preserve the fixed torque request cap.")
    require(candidate == expected, "Candidate changed settings outside the three IMU PID gains.")
    sources = search["source_sha256"]
    require(ACTIVE.as_posix() in sources, "Search has no incumbent configuration hash.")
    # Reject results for an older robot/control model. Workflow/docs edits do
    # not invalidate measurements; physical model and control edits do.
    required = [GEOMETRY, NATIVE, *sorted((root / "cpp/src").rglob("*.cpp")),
                *sorted((root / "cpp/src").rglob("*.h"))]
    required += list((root/"cpp/targets/host").glob("*.cpp")) + list((root/"cpp/targets/mujoco").glob("*.cpp"))
    for item in required:
        relative = item.relative_to(root) if item.is_absolute() else item
        require(sources.get(relative.as_posix()) == digest(root / relative),
                f"Search is stale for {relative}; run optimization again.")
    for name, recorded in sources.items():
        if name in ("gpu/shared_sim.py", "gpu/robot_model.py", "gpu/optimization_core.py",
                    "ros2_ws/src/balancing_robot_sim/scripts/shared_core.py",
                    "ros2_ws/src/balancing_robot_sim/scripts/motor_model.py") or (
                        name.startswith("gpu/") and name.endswith(".xml")):
            require(recorded == digest(root / name), f"Search is stale for {name}.")
    return search, baseline, candidate


def check_record_sources(summary, root):
    hashes = summary["source_sha256"]
    for key,filename in (("controller","balance_controller"),("native_binding","shared_core.py")):
        require(hashes.get(key) == digest(root/GAZEBO_NATIVE.parent/filename),
                f"The recorded Gazebo {key} is stale.")
    for source in sorted((root / "cpp/src").rglob("*")):
        if source.suffix in (".h", ".cpp"):
            name = source.relative_to(root).as_posix()
            require(hashes.get(name) == digest(source), f"Validation is stale for {name}.")
    require(hashes.get("geometry") == digest(root / GEOMETRY), "Validation geometry is stale.")
    # ROS and standalone builds contain different build paths/debug metadata.
    # Compare each binary with its own recording, and compare shared source
    # hashes above; the two independently built libraries need not be identical.
    require(hashes.get("native_library") == digest(root / GAZEBO_NATIVE),
            "The recorded Gazebo C++ binary is stale.")


def check_validation(folder, root, candidate):
    records = read_json(folder / "gazebo-validation/validation.json")["runs"]
    parameters = candidate["balance_controller"]["ros__parameters"]
    for case in CASES:
        matching = [r for r in records if r["profile"] == "candidate" and r["scenario"] == case]
        require(len(matching) == 1 and matching[0].get("returncode") == 0 and
                matching[0].get("response_passed") is True and
                matching[0].get("electrical_prediction_passed") is True,
                f"Gazebo validation did not pass {case}.")
        case_dir = folder / "gazebo-validation/candidate" / case
        summary = read_json(case_dir / "summary.json")
        require(summary.get("feedback_mode") == "imu_only" and summary.get("wheel_feedback_used") is False,
                "Gazebo must validate the actual IMU-only controller.")
        require(summary.get("passed") is True and summary.get("requested_duration_s", 0) >= 12 and
                all(summary.get("checks", {}).get(key) is True for key in BALANCE_CHECKS),
                f"Incomplete Gazebo checks for {case}.")
        measured = yaml.safe_load((case_dir / "parameters.yaml").read_text())
        require(measured["balance_controller"]["ros__parameters"] == parameters and
                measured["motor_driver"] == candidate["motor_driver"],
                f"Gazebo {case} checked a different parameter profile.")
        check_record_sources(summary, root)


def validate_in_gazebo(folder, root):
    print("Validating winner in Gazebo: recovery from +/-5 and +/-10 degrees...", flush=True)
    subprocess.run(["bash", "-c", 'set -e; source /opt/ros/jazzy/setup.bash; '
                    'cd "$1/ros2_ws"; colcon build --symlink-install --packages-select balancing_robot_sim',
                    "optimization-build", str(root)], check=True)
    prefix = ["bash", "-c", 'set -e; source /opt/ros/jazzy/setup.bash; '
              'source "$1/ros2_ws/install/setup.bash"; cd "$1"; shift; exec /usr/bin/python3 "$@"',
              "optimization-validation", str(root)]
    subprocess.run(prefix + [str(root / "analysis/validate_tuning.py"), "--profiles", "candidate",
                   "--candidate", str(folder / "candidate.yaml"), "--cases", *CASES,
                   "--seconds", "12", "--output", str(folder / "gazebo-validation")], check=True)


def replace_gains(text, values):
    """Keep comments, units, unrelated settings and line endings intact."""
    lines, seen, in_controller = [], set(), False
    for line in text.splitlines(keepends=True):
        body = line.rstrip("\r\n")
        ending = line[len(body):]
        if re.match(r"^[^\s#][^:]*:", body):
            in_controller = body.startswith("balance_controller:")
        match = re.match(r"^    (kp|kd|ki):[^#]*(#.*)?$", body)
        if in_controller and match:
            key, comment = match.groups()
            require(key not in seen, f"Duplicate gain key: {key}.")
            seen.add(key)
            literal = f"{values[key]:.17g}"
            if "." not in literal and "e" not in literal.lower():
                literal += ".0" # ROS distinguishes DOUBLE 2.0 from INTEGER 2.
            body = f"    {key}: {literal}"
            if comment:
                body = body.ljust(max(32, len(body) + 1)) + comment
            line = body + ending
        lines.append(line)
    require(seen == set(NAMES), "Cannot locate all three PID gain lines in active YAML.")
    return "".join(lines)


def atomic_write(path, data):
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.",
                                         suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            os.chmod(temporary, path.stat().st_mode)
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def apply_run(folder, root=ROOT, reuse_validation=False):
    folder, root = Path(folder).resolve(), Path(root).resolve()
    search, baseline, candidate = check_search(folder, root)
    active = root / ACTIVE
    expected_hash = search["source_sha256"][ACTIVE.as_posix()]
    current = yaml.safe_load(active.read_text())
    if current == candidate:
        result = dict(status="already_active", active_config_modified=False, active_config=str(active))
        audit = folder / "promotion.json"
        if not audit.exists():
            audit.write_text(json.dumps(result, indent=2) + "\n")
        return result
    require(digest(active) == expected_hash and current == baseline,
            "Active settings changed after the search; keeping them. Run optimization again.")
    if not reuse_validation:
        validate_in_gazebo(folder, root)
    check_validation(folder, root, candidate)
    # Only the brief write is locked. Validation does not block manual editing;
    # a second hash check below refuses to overwrite changes made meanwhile.
    lock = active.with_name(".balance-optimization.lock")
    try:
        descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as error:
        raise UpdateRejected(f"Another update holds {lock}; active gains were kept.") from error
    try:
        os.close(descriptor)
        require(digest(active) == expected_hash, "Active settings changed during validation; keeping them.")
        before = active.read_bytes()
        values = {key: candidate["balance_controller"]["ros__parameters"][key] for key in NAMES}
        after = replace_gains(before.decode("utf-8"), values).encode("utf-8")
        require(yaml.safe_load(after) == candidate, "Gain-only update would change other settings.")
        backup = folder / "previous-active.yaml"
        require(not backup.exists(), "Backup already exists; keeping it and the active settings.")
        backup.write_bytes(before)
        (folder / "applied-profile.yaml").write_bytes(after)
        atomic_write(active, after)
        result = dict(status="applied", active_config_modified=True, active_config=str(active),
                      previous_sha256=expected_hash, active_sha256=digest(active),
                      candidate_sha256=digest(folder / "candidate.yaml"), gains=values,
                      backup=str(backup), gazebo_balance_passed=True,
                      hardware_validated=False)
        try:
            (folder / "promotion.json").write_text(json.dumps(result, indent=2) + "\n")
        except OSError as error:
            # The active replacement already succeeded. Never report that the
            # old gains were kept just because writing the audit failed.
            print(f"Gains updated; could not write promotion.json: {error}. Backup: {backup}", file=sys.stderr)
        return result
    finally:
        lock.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="Optimization output folder")
    parser.add_argument("--reuse-validation", action="store_true",
                        help="Verify existing recordings instead of running Gazebo again")
    args = parser.parse_args()
    try:
        print(json.dumps(apply_run(args.run, reuse_validation=args.reuse_validation), indent=2))
    except (UpdateRejected, OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Active gains were kept: {error}\n")


if __name__ == "__main__":
    main()
