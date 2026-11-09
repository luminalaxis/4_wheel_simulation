"""Powertrain: maximum tractive force at the contact patches, battery power and rotating inertia.

EV options (``powertrain.ev``):

* ``motor: emrax_228`` + ``motor_variant`` / ``motor_cooling`` / ``dc_voltage_V`` /
  ``controller_current_A`` - torque-speed envelope, efficiency and rotor inertia from the motor
  database (:mod:`aerolap.motors`), or
* ``motor_torque_curve: [[rpm, Nm], ...]`` + ``max_rpm`` - a plain torque curve.

``power_limit_kW`` is the accumulator (battery-side) limit, e.g. 80 kW in Formula Student. With a
motor model the wheel force is ``T G eta_drivetrain / r`` and the battery power
``T omega / (eta_motor(n, T) eta_inverter)``; without one a single ``efficiency`` is used.
"""

from __future__ import annotations

import math

import numpy as np

from .motors import Motor


def _curve(points, name: str):
    arr = np.asarray(points, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2 or len(arr) < 2:
        raise ValueError(f"{name} must be a list of [rpm, torque_Nm] pairs (at least two)")
    order = np.argsort(arr[:, 0])
    return arr[order, 0], arr[order, 1]


class Powertrain:
    def __init__(self, cfg: dict, tire_radius: float):
        self.type = cfg["type"]
        self.efficiency = float(cfg["efficiency"])
        self.drive = cfg["drive"]
        self.differential = cfg["differential"]
        frac = cfg.get("awd_front_fraction")
        self.awd_front_fraction = None if frac is None else float(frac)
        self.radius = tire_radius
        self.shift_time = 0.0
        self.motor: Motor | None = None
        self.n_motors = 1
        self.inverter_efficiency = 1.0
        self.power_limit = None
        self.m_rot = 0.0  # kg, rotating driveline inertia reflected to the contact patches
        if self.type == "ice":
            c = cfg["ice"]
            self.rpm, self.torque = _curve(c["torque_curve"], "powertrain.ice.torque_curve")
            self.rev_limit = float(c.get("rev_limit_rpm", self.rpm[-1]))
            primary = float(c.get("primary_ratio", 1.0))
            final = float(c["final_drive"])
            self.ratios = np.array([primary * float(g) * final for g in c["gear_ratios"]])
            self.shift_time = float(c.get("shift_time_s", 0.0))
            top = self.ratios.min()
            self.v_max = self.rev_limit * 2.0 * math.pi / 60.0 / top * tire_radius
            return
        c = cfg["ev"]
        g = float(c["reduction_ratio"])
        self.ratios = np.array([g])
        self.n_motors = int(c.get("n_motors", 1))
        pl = c.get("power_limit_kW")
        self.power_limit = None if pl is None else float(pl) * 1000.0
        if c.get("motor"):
            self.motor = Motor.from_config(c)
            self.efficiency = float(c.get("drivetrain_efficiency", 0.97))
            self.inverter_efficiency = float(c.get("inverter_efficiency", 0.97))
            self.rev_limit = self.motor.max_rpm
            inertia = self.motor.rotor_inertia
        else:
            self.rpm, self.torque = _curve(c["motor_torque_curve"], "powertrain.ev.motor_torque_curve")
            self.rev_limit = float(c.get("max_rpm", self.rpm[-1]))
            inertia = float(c.get("rotor_inertia_kgm2") or 0.0)
        self.m_rot = self.n_motors * inertia * g * g / tire_radius ** 2
        self.v_max = self.rev_limit * 2.0 * math.pi / 60.0 / g * tire_radius

    # ------------------------------------------------------------------ helpers
    def motor_rpm(self, v):
        return np.maximum(np.asarray(v, dtype=float), 0.0) / self.radius * self.ratios[0] * 60.0 / (2.0 * math.pi)

    def _motor_torque_for_force(self, f_wheel):
        return np.asarray(f_wheel, dtype=float) * self.radius / (self.ratios[0] * self.efficiency * self.n_motors)

    def max_force(self, v):
        """Maximum tractive force (N) and the gear index (0-based) that delivers it."""
        v = np.asarray(v, dtype=float)
        if self.motor is not None:
            rpm = self.motor_rpm(v)
            t = self.motor.max_torque(rpm)
            if self.power_limit is not None:
                omega = np.maximum(rpm * 2.0 * math.pi / 60.0, 1e-6)
                eta = np.full(np.shape(t), 0.95)
                for _ in range(3):  # battery limit: T omega n / (eta_m eta_inv) <= P_lim
                    t_p = self.power_limit * eta * self.inverter_efficiency / (omega * self.n_motors)
                    eta = self.motor.efficiency(rpm, np.minimum(t, t_p))
                t = np.minimum(t, t_p)
            f = t * self.n_motors * self.ratios[0] * self.efficiency / self.radius
            return f, np.zeros(np.shape(f), dtype=int)
        omega_wheel = np.maximum(v, 0.0) / self.radius
        rpm = omega_wheel[..., None] * self.ratios * 60.0 / (2.0 * math.pi)  # (..., n_gears)
        torque = np.interp(np.maximum(rpm, self.rpm[0]), self.rpm, self.torque)
        force = np.where(rpm <= self.rev_limit, torque * self.ratios * self.efficiency / self.radius, 0.0)
        gear = np.argmax(force, axis=-1)
        f = np.take_along_axis(force, gear[..., None], axis=-1)[..., 0]
        if self.type == "ev" and self.power_limit is not None:
            f = np.minimum(f, self.power_limit * self.efficiency / np.maximum(v, 0.1))
        return f, gear

    def motor_point(self, v, f_motor):
        """(rpm, torque per motor, motor efficiency) for a force ``f_motor`` at the wheels."""
        rpm = self.motor_rpm(v)
        t = np.maximum(self._motor_torque_for_force(f_motor), 0.0)
        eta = self.motor.efficiency(rpm, t) if self.motor is not None else np.full(np.shape(t), 1.0)
        return rpm, t, eta

    def battery_power(self, v, f_motor):
        """Power drawn from the battery / tank (W) to deliver ``f_motor`` (>= 0) at the wheels."""
        f = np.maximum(np.asarray(f_motor, dtype=float), 0.0)
        p_wheel = f * np.maximum(np.asarray(v, dtype=float), 0.0)
        if self.motor is None:
            return p_wheel / self.efficiency
        _, _, eta = self.motor_point(v, f)
        return p_wheel / (self.efficiency * eta * self.inverter_efficiency)

    def gear_lookup(self, v_max: float, step: float = 0.05):
        """Pre-computed best-gear table for fast scalar lookups in the speed integration."""
        grid = np.arange(0.0, v_max + step, step)
        _, gear = self.max_force(grid)
        return step, gear.tolist()
