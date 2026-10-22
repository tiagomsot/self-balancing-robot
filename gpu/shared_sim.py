"""CUDA physics with the shared C++ controller on the CPU.

State/torque transfers occur at 200 Hz. No separate controller GPU kernel is used.
This favors one implementation; transfers are included in recorded loop timing.
"""
import time
import numpy as np
import mujoco_warp as mjw
import warp as wp
from parallel_sim import initial_state, record_metrics
from shared_core import NativeBatch


class SharedGpuBatch:
    def __init__(self, model, gains, tilts, parameters, device, settings):
        self.cpu_model, self.count = model, len(gains)
        self.parameters_host, self.settings = parameters, settings
        self.gains_host = np.array(gains)
        self.initial_host = np.array(tilts)
        self.model = mjw.put_model(model)
        self.data = mjw.make_data(model, nworld=self.count, nconmax=16, njmax=64)
        self.gains = wp.array(gains.astype(np.float32))
        self.initial = wp.array(tilts.astype(np.float32))
        self.stats = wp.zeros((self.count, 8), dtype=float)
        self.diagnostic = wp.zeros((self.count, 4), dtype=float)
        self.reset(tilts)
        self.physics_step(0)
        wp.synchronize()
        self.reset(tilts)
        with wp.ScopedCapture() as capture:
            self.physics_step(0)
        self.graph = capture.graph
        with wp.ScopedCapture() as capture:
            self.physics_step(1)
        self.tail_graph = capture.graph
        self.reset(tilts)

    def reset(self, tilts):
        mjw.reset_data(self.model, self.data)
        q, v = initial_state(self.cpu_model, self.count, tilts)
        self.data.qpos.assign(q)
        self.data.qvel.assign(v)
        self.initial_host = np.array(tilts)
        self.stats.zero_()
        self.diagnostic.zero_()
        self.native = NativeBatch(self.settings, self.gains_host)
        # A stationary, calibrated IMU before release, like holding the robot
        # still on hardware. Only synthesized accelerometer/gyro channels enter
        # the estimator; subsequent readings come from the physical sensor site.
        held = np.zeros((self.count, 6))
        held[:, 0], held[:, 2] = -np.sin(tilts), np.cos(tilts)
        self.native.prime_imu(held)
        # DLPF mode 3 has about 4.8-4.9 ms group delay. One 200 Hz sample
        # approximates that lag; it does not reproduce the full filter response.
        self.delayed_readings = held.copy()
        self.refresh_sensors()

    def refresh_sensors(self):
        mjw.forward(self.model, self.data)

    def update_control(self):
        readings = self.data.sensordata.numpy().copy()
        readings[:, :3] /= 9.80665
        delayed, self.delayed_readings = self.delayed_readings, readings
        torques, diagnostic = self.native.update(self.data.qvel.numpy(), delayed, self.parameters_host)
        self.data.ctrl.assign(torques.astype(np.float32))
        self.diagnostic.assign(diagnostic.astype(np.float32))

    def physics_step(self, tail):
        for _ in range(5):
            mjw.step(self.model, self.data)
            wp.launch(record_metrics, dim=self.count, inputs=[self.data.qpos, self.data.qvel,
                self.initial, self.stats, float(self.parameters_host[0]), .001, tail])

    def run(self, seconds):
        wp.synchronize()
        started = time.perf_counter()
        for step in range(round(seconds/.005)):
            self.update_control()
            wp.capture_launch(self.tail_graph if (step+1)*.005>seconds-1 else self.graph)
        wp.synchronize()
        return (time.perf_counter()-started, self.data.qpos.numpy(), self.data.qvel.numpy(),
                self.stats.numpy(), self.diagnostic.numpy())
