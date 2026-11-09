"""Powertrain: maximum tractive force at the contact patches as a function of speed."""

from __future__ import annotations

import math

import numpy as np


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
        else:
            c = cfg["ev"]
            self.rpm, self.torque = _curve(c["motor_torque_curve"], "powertrain.ev.motor_torque_curve")
            self.rev_limit = float(c.get("max_rpm", self.rpm[-1]))
            self.ratios = np.array([float(c["reduction_ratio"])])
            pl = c.get("power_limit_kW")
            self.power_limit = None if pl is None else float(pl) * 1000.0
            self.v_max = self.rev_limit * 2.0 * math.pi / 60.0 / self.ratios[0] * tire_radius

    def max_force(self, v):
        """Maximum tractive force (N) and the gear index (0-based) that delivers it."""
        v = np.asarray(v, dtype=float)
        omega_wheel = np.maximum(v, 0.0) / self.radius
        rpm = omega_wheel[..., None] * self.ratios * 60.0 / (2.0 * math.pi)  # (..., n_gears)
        torque = np.interp(np.maximum(rpm, self.rpm[0]), self.rpm, self.torque)
        force = np.where(rpm <= self.rev_limit, torque * self.ratios * self.efficiency / self.radius, 0.0)
        gear = np.argmax(force, axis=-1)
        f = np.take_along_axis(force, gear[..., None], axis=-1)[..., 0]
        if self.type == "ev" and self.power_limit is not None:
            f = np.minimum(f, self.power_limit * self.efficiency / np.maximum(v, 0.1))
        return f, gear

    def gear_lookup(self, v_max: float, step: float = 0.05):
        """Pre-computed best-gear table for fast scalar lookups in the speed integration."""
        grid = np.arange(0.0, v_max + step, step)
        _, gear = self.max_force(grid)
        return step, gear.tolist()
