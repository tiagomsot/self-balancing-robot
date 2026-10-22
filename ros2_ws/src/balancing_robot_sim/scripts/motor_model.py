"""Averaged TT motor / L293D surrogate; see docs/motor-driver-model.md.

The gearbox-output endpoints are reference data, not identification of the
owner's motors. PWM ripple and winding inductance are not resolved. Current
ratings produce diagnostics, never an invented L293D current regulator.
"""

import math
from dataclasses import dataclass
from shared_core import duty_for_torque as shared_duty_for_torque


def clip(value, limit):
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class MotorParameters:
    nominal_voltage_v: float = 6.0
    stall_current_a: float = 1.5
    stall_torque_nm: float = 0.0784532
    no_load_speed_rpm: float = 200.0
    no_load_current_a: float = 0.15
    battery_voltage_v: float = 6.0
    battery_resistance_ohm: float = 0.0
    driver_drop_at_rating_v: float = 2.6
    driver_drop_intercept_v: float = 1.8
    driver_current_rating_a: float = 0.6
    driver_peak_current_a: float = 1.2
    driver_min_supply_v: float = 5.0
    driver_thermal_resistance_k_w: float = 36.4
    ambient_temperature_c: float = 25.0
    driver_max_junction_c: float = 150.0
    pwm_levels: int = 255
    pwm_frequency_hz: float = 490.0
    pwm_off_mode: str = "coast"
    command_deadband: float = 0.0
    current_time_constant_s: float = 0.0
    command_full_pwm_torque_nm: float = 0.04118793

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == "pwm_off_mode":
                continue
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            if name != "ambient_temperature_c" and value < 0:
                raise ValueError(f"{name} must be nonnegative")
        for name in ("nominal_voltage_v", "stall_current_a", "stall_torque_nm",
                     "no_load_speed_rpm", "battery_voltage_v", "driver_current_rating_a",
                     "driver_peak_current_a", "driver_thermal_resistance_k_w",
                     "pwm_frequency_hz", "command_full_pwm_torque_nm"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if not isinstance(self.pwm_levels, int) or self.pwm_levels < 1:
            raise ValueError("pwm_levels must be a positive integer")
        if self.no_load_current_a >= self.stall_current_a:
            raise ValueError("no-load current must be smaller than stall current")
        if self.driver_drop_at_rating_v < self.driver_drop_intercept_v:
            raise ValueError("driver drop at the rating must exceed its intercept")
        if self.driver_drop_intercept_v >= self.nominal_voltage_v:
            raise ValueError("nominal motor voltage must exceed driver loss intercept")
        if self.pwm_off_mode not in ("coast", "brake"):
            raise ValueError("pwm_off_mode must be coast or brake")
        if not 0 <= self.command_deadband < 1:
            raise ValueError("command_deadband must be in [0, 1)")

    @property
    def resistance_ohm(self):
        return self.nominal_voltage_v / self.stall_current_a

    @property
    def torque_constant_nm_a(self):
        return self.stall_torque_nm / self.stall_current_a

    @property
    def no_load_speed_rad_s(self):
        return self.no_load_speed_rpm * 2 * math.pi / 60

    @property
    def back_emf_v_s_rad(self):
        return (self.nominal_voltage_v - self.resistance_ohm * self.no_load_current_a) / self.no_load_speed_rad_s

    @property
    def viscous_drag_nm_s_rad(self):
        return self.torque_constant_nm_a * self.no_load_current_a / self.no_load_speed_rad_s

    @property
    def driver_resistance_ohm(self):
        return (self.driver_drop_at_rating_v - self.driver_drop_intercept_v) / self.driver_current_rating_a

    @property
    def reference_full_pwm_stall_torque_nm(self):
        # Static open-loop command calibration, not a current feedback loop.
        current = (self.nominal_voltage_v - self.driver_drop_intercept_v) / (self.resistance_ohm + self.driver_resistance_ohm)
        return self.torque_constant_nm_a * current


@dataclass(frozen=True)
class MotorSample:
    duty: float
    current_a: float
    rms_current_estimate_a: float
    on_current_estimate_a: float
    torque_nm: float
    driver_loss_w: float
    command_saturated: bool
    on_time_estimate_s: float


@dataclass(frozen=True)
class DriveSample:
    left: MotorSample
    right: MotorSample
    bus_voltage_v: float
    supply_current_a: float
    driver_loss_w: float
    steady_state_junction_estimate_c: float
    average_current_rating_exceeded: bool
    on_current_rating_uncertain: bool
    peak_current_estimate_exceeded: bool
    supply_out_of_range: bool


class MotorDriverModel:
    def __init__(self, parameters=None):
        self.p = parameters or MotorParameters()
        self.currents = [0.0, 0.0]

    def duty_for_torque(self, requested_nm):
        # Controller calibration stays fixed when the actual driver/battery
        # assumptions change; uncertainty must not silently retune feedforward.
        raw = requested_nm / self.p.command_full_pwm_torque_nm
        duty = shared_duty_for_torque(requested_nm, self.p.command_full_pwm_torque_nm,
                                     self.p.pwm_levels, self.p.command_deadband)
        return duty, abs(raw) > 1

    def electrical(self, duty, speed, bus):
        p = self.p
        resistance = p.resistance_ohm + p.driver_resistance_ohm
        if p.pwm_off_mode == "coast":
            if duty == 0:
                return 0.0, 0.0, 0.0, 0.0, 0.0
            # Quasi-static ON current; OFF current is assumed to decay to zero.
            # This is an upper estimate of pulse current, not a resolved waveform.
            sign = math.copysign(1, duty)
            on_current = (sign * max(0.0, bus - p.driver_drop_intercept_v)
                          - p.back_emf_v_s_rad * speed) / resistance
            fraction = abs(duty)
            current = fraction * on_current
            rms = math.sqrt(fraction) * abs(on_current)
            loss = fraction * (p.driver_drop_intercept_v * abs(on_current)
                               + p.driver_resistance_ohm * on_current**2)
            supply = fraction * max(0.0, sign * on_current)
        else:
            # Continuous-current averaged approximation with resistive braking.
            current = (duty * max(0.0, bus - p.driver_drop_intercept_v)
                       - p.back_emf_v_s_rad * speed) / resistance
            on_current = current  # PWM ripple is unknown in this mode.
            rms = abs(current)
            loss = abs(duty * current) * p.driver_drop_intercept_v + p.driver_resistance_ohm * current**2
            supply = max(0.0, duty * current)
        return current, rms, on_current, loss, supply

    def step(self, requests_nm, speeds_rad_s, dt):
        if len(requests_nm) != 2 or len(speeds_rad_s) != 2:
            raise ValueError("Exactly two motor commands and shaft speeds are required")
        if not math.isfinite(dt) or dt < 0 or not all(
                math.isfinite(x) for x in (*requests_nm, *speeds_rad_s)):
            raise ValueError("Motor inputs and dt must be finite; dt must be nonnegative")
        p = self.p
        commands = [self.duty_for_torque(value) for value in requests_nm]

        def supply_at(bus):
            return sum(self.electrical(cmd[0], speed, bus)[4]
                       for cmd, speed in zip(commands, speeds_rad_s))

        # A shared source couples both motors. Bisection avoids an unstable
        # fixed-point iteration when an assumed battery resistance is high.
        low, high = 0.0, p.battery_voltage_v
        if p.battery_resistance_ohm:
            for _ in range(32):
                bus = (low + high) / 2
                if bus + p.battery_resistance_ohm * supply_at(bus) > p.battery_voltage_v:
                    high = bus
                else:
                    low = bus
            bus = (low + high) / 2
        else:
            bus = high
        samples = []
        for index, ((duty, saturated), speed) in enumerate(zip(commands, speeds_rad_s)):
            current, rms, on_current, loss, _ = self.electrical(duty, speed, bus)
            if p.current_time_constant_s:
                alpha = -math.expm1(-dt / p.current_time_constant_s)
                current = self.currents[index] + alpha * (current - self.currents[index])
                # Quasi-static diagnostics must not be smaller than the
                # persistent mean current in this optional response proxy.
                rms = max(rms, abs(current))
                on_current = math.copysign(max(abs(on_current), abs(current)), on_current or current)
                loss = max(loss, p.driver_resistance_ohm * current**2)
            self.currents[index] = current
            torque = p.torque_constant_nm_a * current - p.viscous_drag_nm_s_rad * speed
            samples.append(MotorSample(duty, current, rms, on_current, torque, loss, saturated,
                                       abs(duty) / p.pwm_frequency_hz))
        total_loss = sum(sample.driver_loss_w for sample in samples)
        return DriveSample(
            samples[0], samples[1], bus, supply_at(bus), total_loss,
            p.ambient_temperature_c + p.driver_thermal_resistance_k_w * total_loss,
            any(abs(s.current_a) > p.driver_current_rating_a for s in samples),
            any(abs(s.on_current_estimate_a) > p.driver_current_rating_a for s in samples),
            any(abs(s.on_current_estimate_a) > p.driver_peak_current_a for s in samples),
            bus < p.driver_min_supply_v,
        )
