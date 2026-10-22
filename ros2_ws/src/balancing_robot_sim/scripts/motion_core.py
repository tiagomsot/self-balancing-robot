"""ROS parameter/recording types around the shared C++ control and routine core."""
from dataclasses import dataclass
import math
from shared_core import (MotionReference, PlanarOdometry, YawController, Routine,
                         allocate_torque, attitude, quaternion, wrap, yaw_rate, fault_name)


def clip(value, limit):
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class ShuttleSettings:
    leg_distance_m: float = .50
    speed_m_s: float = .25
    acceleration_m_s2: float = .15
    turn_rate_rad_s: float = math.radians(50)
    turn_acceleration_rad_s2: float = .35
    position_gain_s_inv: float = .8
    heading_gain_s_inv: float = 1.2
    heading_damping: float = .2
    position_tolerance_m: float = .005
    heading_tolerance_deg: float = 3.
    stop_speed_m_s: float = .01
    stop_yaw_rate_rad_s: float = .05
    settle_time_s: float = 2.
    turn_settle_time_s: float = 1.
    phase_timeout_s: float = 40.
    cycles: int = 0

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value < 0 or (name != "cycles" and value == 0):
                raise ValueError(f"{name} must be finite and positive (cycles may be zero)")
        if not isinstance(self.cycles, int) or not 0 <= self.cycles <= 32767:
            raise ValueError("cycles must be an integer in 0..32767")
        if self.speed_m_s > .25 or self.turn_rate_rad_s > math.radians(50):
            raise ValueError("Demo limits: speed <=0.25 m/s and turn rate <=50 degrees/s")


@dataclass(frozen=True)
class MotionState:
    time_s: float
    x: float
    y: float
    yaw: float
    pitch: float
    pitch_rate: float
    speed: float
    yaw_rate: float


class Shuttle:
    """C++ routine adapter. Python only collects telemetry and historical events."""
    phases = ("WAIT_BALANCE", "DRIVE", "STOP", "TURN", "TURN_SETTLE", "DONE", "FAULT", "WAIT")

    def __init__(self, settings=None):
        self.p = settings or ShuttleSettings()
        self.core = Routine(self.p)
        self.phase = "WAIT_BALANCE"
        self.legs_completed = 0
        self.progress = self.remaining = 0.
        self.start_x = self.start_y = self.leg_heading = 0.
        self.events, self.legs = [], []
        self.fault_reason = ""

    def record_phase(self, state):
        self.events.append(dict(time_s=state.time_s, phase=self.phase, x_m=state.x,
                               y_m=state.y, yaw_rad=state.yaw, legs_completed=self.legs_completed))

    def fault(self, reason, state):
        self.core.fail()
        if self.phase != "FAULT":
            self.phase, self.fault_reason = "FAULT", reason
            self.record_phase(state)

    def update(self, state):
        out = self.core.update(state)
        phase, legs, fault = self.phases[int(out[2])], int(out[3]), int(out[6])
        self.progress, self.remaining = out[4:6]
        self.start_x, self.start_y, self.leg_heading = out[7:10]
        if legs > self.legs_completed:
            self.legs.append(dict(index=legs, start_x_m=self.start_x, start_y_m=self.start_y,
                end_x_m=state.x, end_y_m=state.y, projected_distance_m=self.progress,
                heading_rad=self.leg_heading))
        if self.phase == "STOP" and phase != "STOP" and self.legs:
            self.legs[-1].update(end_x_m=state.x, end_y_m=state.y,
                projected_distance_m=(state.x-self.start_x)*math.cos(self.leg_heading)
                    +(state.y-self.start_y)*math.sin(self.leg_heading))
        self.legs_completed = legs
        if phase != self.phase:
            self.phase = phase
            if fault:
                self.fault_reason = fault_name(fault)
            self.record_phase(state)
        return out[:2]
