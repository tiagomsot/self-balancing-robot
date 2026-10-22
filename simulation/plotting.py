"""Robot replay, gain controls, and static figures for the first PD experiment."""

from dataclasses import replace
from pathlib import Path
import time

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
from matplotlib.widgets import Button, Slider
import numpy as np

from .runner import save_data, simulate

BLUE = "#1768ac"
ORANGE = "#c35d12"
GREEN = "#26734d"


def body_vertices(state, robot):
    x, _, angle, _ = state
    bottom = -robot.body_bottom_below_axle_m
    top = bottom+robot.body_height_m
    local = np.array([[-robot.body_depth_m/2, bottom], [robot.body_depth_m/2, bottom],
                      [robot.body_depth_m/2, top], [-robot.body_depth_m/2, top]])
    rotation = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
    return local @ rotation.T + [x, robot.wheel_radius_m]


def body_point(state, height, robot):
    x, _, angle, _ = state
    return [x + height*np.sin(angle), robot.wheel_radius_m + height*np.cos(angle)]


def make_panels(result, interactive=False):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    if interactive:
        fig.subplots_adjust(left=0.09, right=0.96, bottom=0.39, top=0.88, hspace=0.55, wspace=0.32)
    else:
        fig.subplots_adjust(left=0.09, right=0.96, bottom=0.14, top=0.87, hspace=0.47, wspace=0.32)
    pose = draw_panels(fig, axes, result)
    return fig, axes, pose


def draw_panels(fig, axes, result):
    for ax in axes.flat:
        ax.clear()
    robot_ax, angle_ax, position_ax, torque_ax = axes.flat
    robot, settings = result.robot, result.experiment
    fig.suptitle(f"PD balancing experiment | {settings.controller} controller\n"
                 f"P = {settings.kp:.4f} N m/rad    D = {settings.kd:.5f} N m s/rad    "
                 f"initial tilt = {settings.initial_tilt_deg:g} deg", fontsize=13)

    robot_ax.axhline(0, color="0.4", linewidth=1)
    robot_ax.set_xlim(np.min(result.state[:, 0])-0.20, np.max(result.state[:, 0])+0.20)
    robot_ax.set_ylim(-0.025, robot.wheel_radius_m + robot.body_height_m
                     - robot.body_bottom_below_axle_m + 0.055)
    robot_ax.set_aspect("equal", adjustable="box")
    robot_ax.set_xlabel("Forward position (m)")
    robot_ax.set_ylabel("Height (m)")
    robot_ax.set_title("Robot side view")
    robot_ax.add_patch(Polygon(body_vertices(result.state[0], robot), closed=True,
                              facecolor="none", edgecolor="0.65", linestyle="--", linewidth=1))
    robot_ax.add_patch(Circle((result.state[0, 0], robot.wheel_radius_m), robot.wheel_radius_m,
                             facecolor="none", edgecolor="0.65", linestyle="--", linewidth=1))

    angle_ax.axhspan(-1, 1, color=GREEN, alpha=0.08, label="1 deg band")
    angle_ax.axhline(0, color="0.5", linewidth=0.8)
    angle_ax.plot(result.time_s, np.rad2deg(result.state[:, 2]), color=BLUE, label="Body tilt")
    angle_ax.set_ylabel("Tilt (deg)")
    angle_ax.set_title("Does the tilt settle?")
    angle_ax.legend(loc="best", fontsize=8)

    position_ax.plot(result.time_s, result.state[:, 0], color=BLUE)
    position_ax.set_ylabel("Axle position (m)")
    position_ax.set_title("Watch for position drift")

    torque_ax.axhline(settings.total_torque_limit_Nm, color="0.6", linewidth=0.8, linestyle=":")
    torque_ax.axhline(-settings.total_torque_limit_Nm, color="0.6", linewidth=0.8, linestyle=":")
    torque_ax.plot(result.time_s, result.requested_torque_Nm, color=ORANGE,
                   linestyle="--", label="Requested")
    torque_ax.plot(result.time_s, result.applied_torque_Nm, color=BLUE, linewidth=1.3, label="Applied")
    torque_ax.set_ylabel("Combined wheel torque (N m)")
    torque_ax.set_title(f"Imposed torque cap: +/-{settings.total_torque_limit_Nm:g} N m")
    torque_ax.legend(loc="best", fontsize=8)

    for ax in (angle_ax, position_ax, torque_ax):
        ax.set_xlabel("Time (s)")
        ax.set_xlim(0, result.time_s[-1])
        ax.grid(alpha=0.2)
    return add_robot_pose(robot_ax, result.state[-1], robot)


def add_robot_pose(ax, state, robot):
    body = Polygon(body_vertices(state, robot), closed=True,
                   facecolor=BLUE, alpha=0.18, edgecolor=BLUE, linewidth=1.6)
    wheel = Circle((state[0], robot.wheel_radius_m), robot.wheel_radius_m,
                   facecolor="0.90", edgecolor="0.25", linewidth=1.6)
    ax.add_patch(body)
    ax.add_patch(wheel)
    spoke, = ax.plot([], [], color="0.25", linewidth=1.2)
    com, = ax.plot([], [], "+", color="0.15", markersize=8, label="Centre of mass")
    imu, = ax.plot([], [], "o", color=GREEN, markersize=4, label="IMU location")
    artists = (body, wheel, spoke, com, imu)
    update_robot_pose(artists, state, robot)
    return artists


def update_robot_pose(artists, state, robot):
    body, wheel, spoke, com, imu = artists
    x = state[0]
    r = robot.wheel_radius_m
    body.set_xy(body_vertices(state, robot))
    wheel.center = (x, r)
    phase = -x/r
    spoke.set_data([x, x + r*np.cos(phase)], [r, r + r*np.sin(phase)])
    cx, cz = body_point(state, robot.com_height_m, robot)
    ix, iz = body_point(state, robot.imu_above_axle_m, robot)
    com.set_data([cx], [cz])
    imu.set_data([ix], [iz])


def outcome_text(result):
    m = result.metrics()
    if result.fell:
        outcome = f"Fall cutoff reached at {m['simulated_duration_s']:.2f} s"
    elif m["pitch_settling_time_s"] is None:
        outcome = "Pitch did not settle within this run"
    else:
        outcome = f"Pitch settled at {m['pitch_settling_time_s']:.2f} s"
    return (f"{outcome} | final axle speed {m['final_axle_speed_m_s']:+.3f} m/s | "
            f"saturated {100*m['saturated_control_fraction']:.1f}% of control intervals")


def save_figure(result, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    fig, _, _ = make_panels(result)
    fig.text(0.5, 0.065, outcome_text(result), ha="center", fontsize=9)
    fig.text(0.5, 0.025, "Ideal pitch/rate and bounded instantaneous torque; no driver losses, "
             "backlash, motor dynamics, or position control.\n"
             "Robot drawing shows the final pose; dashed outline shows the starting pose.",
             ha="center", fontsize=8, color="0.35")
    fig.savefig(output_dir / "plots.png", dpi=150)
    plt.close(fig)


class ExperimentWindow:
    """Replay precomputed dynamics; changing a slider takes effect on Run."""

    def __init__(self, result, output_dir):
        self.result = result
        self.output_dir = Path(output_dir)
        self.fig, self.axes, self.pose = make_panels(result, interactive=True)
        self.fig.canvas.manager.set_window_title("Self-balancing robot: first PD experiment")
        p = result.experiment
        self.kp_slider = Slider(self.fig.add_axes([0.22, 0.20, 0.59, 0.025]),
                                "P (N m/rad)", 0, max(0.6, 2*p.kp), valinit=p.kp, valfmt="%.4f")
        self.kd_slider = Slider(self.fig.add_axes([0.22, 0.145, 0.59, 0.025]),
                                "D (N m s/rad)", 0, max(0.05, 2*p.kd), valinit=p.kd, valfmt="%.5f")
        tilt_limit = min(p.fall_angle_deg-0.01, max(20, abs(p.initial_tilt_deg)+1))
        self.tilt_slider = Slider(self.fig.add_axes([0.22, 0.09, 0.59, 0.025]),
                                  "Starting tilt (deg)", -tilt_limit, tilt_limit,
                                  valinit=p.initial_tilt_deg, valfmt="%.1f")
        self.run_button = Button(self.fig.add_axes([0.22, 0.255, 0.17, 0.045]), "Run with gains")
        self.replay_button = Button(self.fig.add_axes([0.43, 0.255, 0.14, 0.045]), "Replay")
        self.save_button = Button(self.fig.add_axes([0.61, 0.255, 0.20, 0.045]), "Save plots + CSV")
        self.status = self.fig.text(0.5, 0.035, "", ha="center", fontsize=9)
        self.fig.text(0.5, 0.005, "Ideal torque model | perfect tilt/rate | no position controller | "
                      "slider changes apply when you press Run", ha="center", fontsize=8, color="0.35")
        self.run_button.on_clicked(self.run)
        self.replay_button.on_clicked(self.replay)
        self.save_button.on_clicked(self.save)
        self.timer = self.fig.canvas.new_timer(interval=30)
        self.timer.add_callback(self.tick)
        self.fig.canvas.mpl_connect("close_event", lambda event: self.timer.stop())
        self._prepare_replay()

    def _prepare_replay(self):
        self.cursors = [ax.axvline(0, color="0.5", linewidth=0.8) for ax in list(self.axes.flat)[1:]]
        self.status.set_text(outcome_text(self.result))
        self.draw_sample(0)

    def draw_sample(self, index):
        update_robot_pose(self.pose, self.result.state[index], self.result.robot)
        current = self.result.time_s[index]
        self.axes.flat[0].set_title(f"Robot side view | t = {current:.2f} s")
        for cursor in self.cursors:
            cursor.set_xdata([current, current])

    def replay(self, event=None):
        self.timer.stop()
        self.started = time.perf_counter()
        self.draw_sample(0)
        self.timer.start()
        self.fig.canvas.draw_idle()

    def tick(self):
        elapsed = time.perf_counter() - self.started
        index = min(int(np.searchsorted(self.result.time_s, elapsed)), len(self.result.time_s)-1)
        self.draw_sample(index)
        if index == len(self.result.time_s)-1:
            self.timer.stop()
        self.fig.canvas.draw_idle()

    def run(self, event=None):
        self.timer.stop()
        try:
            experiment = replace(self.result.experiment, kp=float(self.kp_slider.val),
                                 kd=float(self.kd_slider.val), initial_tilt_deg=float(self.tilt_slider.val))
            updated = simulate(experiment, self.result.robot)
        except (ValueError, NotImplementedError) as error:
            self.status.set_text(str(error))
            self.fig.canvas.draw_idle()
            return
        self.result = updated
        self.pose = draw_panels(self.fig, self.axes, self.result)
        self._prepare_replay()
        self.replay()

    def save(self, event=None):
        save_data(self.result, self.output_dir)
        save_figure(self.result, self.output_dir)
        self.status.set_text(f"Saved current run to {self.output_dir}")
        self.fig.canvas.draw_idle()

    def show(self):
        self.replay()
        plt.show()
