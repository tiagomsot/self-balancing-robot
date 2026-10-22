"""View independent GPU balancing worlds in a spaced-out display grid.

Offsets affect rendering only. All worlds still simulate locally and remain
isolated. A single CPU pose copy drives the native viewer; no CPU integration
replaces the GPU simulation. The remaining robots are display-only geoms.
"""

import argparse
import json
import math
from pathlib import Path
import time

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from parallel_sim import GAIN_NAMES, GpuBatch, configuration, load_model
from shared_sim import SharedGpuBatch


def grid_offsets(count, spacing):
    columns = math.ceil(math.sqrt(count))
    rows = math.ceil(count/columns)
    return np.array([((i % columns-(columns-1)/2)*spacing,
                      (i//columns-(rows-1)/2)*spacing, 0) for i in range(count)])


def camera_for_grid(count, spacing):
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.lookat[:] = [0, 0, .12]
    camera.distance = max(1.2, math.ceil(math.sqrt(count))*spacing*1.5)
    camera.azimuth, camera.elevation = 135, -35
    return camera


def display_poses(batch, model, data, offsets):
    # GPU -> CPU copies are intentional in this visible mode. They are absent
    # from the benchmark's timed loop, and make the viewer slower than a batch.
    positions = batch.data.geom_xpos.numpy()
    rotations = batch.data.geom_xmat.numpy()
    q = batch.data.qpos.numpy()
    data.qpos[:] = q[0]
    data.qpos[:3] += offsets[0]
    mujoco.mj_forward(model, data)
    return positions, rotations, batch.diagnostic.numpy()


def append_grid(scene, model, positions, rotations, diagnostics, offsets, tilts):
    # The renderer already contains world zero. Append the other worlds and
    # one tile/label per world. Never feed these offsets back into GPU qpos.
    count = len(offsets)
    required = (count-1)*(model.ngeom-1)+count*2
    if scene.ngeom+required > scene.maxgeom:
        raise ValueError("Too many display geoms; reduce --show")
    colors = ((.15, .50, .85, 1), (.12, .65, .50, 1),
              (.90, .55, .15, 1), (.58, .36, .77, 1))

    def add(kind, size, position, rotation, rgba, label=""):
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, type=kind, size=np.asarray(size, dtype=float),
                           pos=np.asarray(position, dtype=float),
                           mat=np.asarray(rotation, dtype=float).reshape(9),
                           rgba=np.asarray(rgba, dtype=np.float32))
        geom.label = label
        scene.ngeom += 1

    for i, offset in enumerate(offsets):
        failed = diagnostics[i, 3] > .5
        color = (.9, .15, .15, 1) if failed else colors[i % 4]
        if i:
            for geom_id in range(1, model.ngeom):
                add(model.geom_type[geom_id], model.geom_size[geom_id],
                    positions[i, geom_id]+offset, rotations[i, geom_id],
                    color if geom_id == 1 else model.geom_rgba[geom_id])
        add(mujoco.mjtGeom.mjGEOM_BOX, [.27, .27, .0001], offset+[0, 0, .0001],
            np.eye(3), [.90, .93, .96, 1] if i % 2 else [.81, .86, .91, 1])
        add(mujoco.mjtGeom.mjGEOM_LABEL, [0, 0, 0], offset+[0, -.27, .02],
            np.eye(3), [0, 0, 0, 1], f"W{i+1}  {math.degrees(tilts[i]):+g} deg")


def snapshot(batch, model, offsets, tilts, spacing, output):
    from PIL import Image
    data = mujoco.MjData(model)
    positions, rotations, diagnostic = display_poses(batch, model, data, offsets)
    with mujoco.Renderer(model, width=1280, height=900) as renderer:
        renderer.update_scene(data, camera=camera_for_grid(len(offsets), spacing))
        append_grid(renderer.scene, model, positions, rotations, diagnostic, offsets, tilts)
        pixels = renderer.render().copy()
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels).save(output)
    print(f"Grid snapshot: {output.resolve()}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worlds", type=int, default=16, help="Independent GPU worlds")
    parser.add_argument("--show", type=int, default=64, help="Maximum worlds to draw")
    parser.add_argument("--spacing", type=float, default=.75, help="Display-only spacing in metres")
    parser.add_argument("--period", type=float, default=6, help="Simulated seconds before repeating")
    parser.add_argument("--seed", type=int, default=20261021)
    parser.add_argument("--frames", type=int, default=0, help="Optional bounded GUI check; 0 runs until closed")
    parser.add_argument("--snapshot", type=Path, help="Save one grid image instead of opening a window")
    parser.add_argument("--params-file", type=Path, help="Full YAML profile; use exactly these gains in every world")
    args = parser.parse_args()
    if not 1 <= args.worlds <= 4096 or not 1 <= args.show <= 256:
        parser.error("Need 1..4096 simulated worlds and 1..256 displayed worlds")
    if not math.isfinite(args.spacing) or args.spacing < .6 or not math.isfinite(args.period) or args.period < 2 or args.frames < 0:
        parser.error("Need spacing >=0.6 m, period >=2 s and nonnegative frames")
    wp.init()
    if not wp.is_cuda_available():
        raise RuntimeError("Live GPU view requires CUDA; no CPU physics fallback")
    device = wp.get_device("cuda:0")
    _, balance, _, parameters = configuration(args.params_file)
    model = load_model()
    # A larger offscreen buffer supports reproducible visual checks.
    model.vis.global_.offwidth, model.vis.global_.offheight = 1280, 900
    gains = np.tile([balance[k] for k in GAIN_NAMES], (args.worlds, 1))
    rng = np.random.default_rng(args.seed)
    # Show the active hardware controller in every world; vary initial tilt only.
    tilts = np.radians(np.resize([5., -5., 10., -10.], args.worlds))
    visible = min(args.show, args.worlds)
    offsets = grid_offsets(visible, args.spacing)
    state = dict(paused=False, reset=False)

    def key_callback(key):
        if key == 32:
            state["paused"] = not state["paused"]
        elif key in (ord("R"), ord("r")):
            state["reset"] = True

    with wp.ScopedDevice(device):
        batch = SharedGpuBatch(model, gains, tilts, parameters, device, balance)
        mjw.forward(batch.model, batch.data)
        if args.snapshot:
            snapshot(batch, model, offsets, tilts, args.spacing, args.snapshot)
            return
        from mujoco import viewer as native_viewer
        display_data = mujoco.MjData(model)
        simulated = 0.0
        frame = resets = 0
        print(f"LIVE: {args.worlds} GPU worlds on {device.name}; displaying {visible} independent robots.", flush=True)
        print("Space: pause/resume | R: restart trial | mouse: orbit/zoom | close window or Ctrl+C: exit", flush=True)
        try:
            with native_viewer.launch_passive(model, display_data, key_callback=key_callback) as viewer:
                with viewer.lock():
                    camera = camera_for_grid(visible, args.spacing)
                    viewer.cam.lookat[:] = camera.lookat
                    viewer.cam.distance = camera.distance
                    viewer.cam.azimuth, viewer.cam.elevation = camera.azimuth, camera.elevation
                while viewer.is_running() and (not args.frames or frame < args.frames):
                    started = time.perf_counter()
                    if state["reset"] or simulated >= args.period:
                        batch.reset(tilts)
                        mjw.forward(batch.model, batch.data)
                        simulated = 0.0
                        resets += 1
                        state["reset"] = False
                    if not state["paused"]:
                        for _ in range(4):
                            batch.update_control()
                            wp.capture_launch(batch.graph)
                        simulated += .020
                    positions, rotations, diagnostic = display_poses(batch, model, display_data, offsets)
                    with viewer.lock():
                        viewer.user_scn.ngeom = 0
                        append_grid(viewer.user_scn, model, positions, rotations,
                                    diagnostic, offsets, tilts)
                    viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,
                                      mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                      "GPU worlds\nTrial time\nControls\nMode",
                                      f"{args.worlds} ({visible} visible)\n{simulated:.2f} / {args.period:g} s\nSpace: pause | R: reset\n"
                                      + ("Paused" if state["paused"] else "Balancing; automatic repeat")))
                    viewer.sync()
                    frame += 1
                    time.sleep(max(0, .020-(time.perf_counter()-started)))
        except KeyboardInterrupt:
            print("\nLive GPU viewer stopped.", flush=True)
        overflow = batch.data.overflow.numpy()
        if np.any(overflow):
            raise RuntimeError("GPU constraint/contact buffer overflow in live viewer")
        print(json.dumps(dict(viewer_frames=frame, automatic_or_manual_resets=resets,
                              simulated_time_in_trial_s=simulated,
                              all_states_finite=bool(np.isfinite(batch.data.qpos.numpy()).all()),
                              physics_device=str(device))), flush=True)


if __name__ == "__main__":
    main()
