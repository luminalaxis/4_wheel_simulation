"""Solver interface and the lap result shared by all solvers (QSS now, transient later)."""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

import numpy as np


class Solver(Protocol):
    name: str

    def solve(self, vehicle, track, start: str = "flying") -> "LapResult":
        """Return the lap result of ``vehicle`` on ``track``.

        ``start`` is ``"standing"`` (v=0 at s=0) or ``"flying"`` (closed tracks: periodic
        lap; open tracks: start at the local speed limit).
        """
        ...


@dataclass
class LapResult:
    track_name: str
    vehicle_name: str
    level: str
    solver: str
    s: np.ndarray
    v: np.ndarray
    ax: np.ndarray
    ay: np.ndarray  # signed (positive = left turn)
    kappa: np.ndarray
    t: np.ndarray
    mode: np.ndarray
    mode_names: list[str]
    channels: dict[str, np.ndarray] = field(default_factory=dict)
    x: np.ndarray | None = None
    y: np.ndarray | None = None
    energy_J: float = 0.0
    default_mode: int = 0
    info: dict = field(default_factory=dict)  # solver-specific extras (e.g. transient iterations)

    @property
    def lap_time(self) -> float:
        return float(self.t[-1])

    def summary(self) -> dict[str, float]:
        ch = self.channels
        ds = np.diff(self.s)
        in_other = (self.mode[:-1] != self.default_mode).astype(float)
        out = {
            "lap_time_s": self.lap_time,
            "distance_m": float(self.s[-1]),
            "v_max_kmh": float(self.v.max() * 3.6),
            "v_min_kmh": float(self.v.min() * 3.6),
            "v_avg_kmh": float(self.s[-1] / max(self.lap_time, 1e-9) * 3.6),
            "ay_max_g": float(np.abs(self.ay).max() / 9.80665),
            "ax_max_g": float(self.ax.max() / 9.80665),
            "ax_min_g": float(self.ax.min() / 9.80665),
            "drive_energy_kWh": self.energy_J / 3.6e6,
            "non_default_mode_pct": float((in_other * ds).sum() / max(ds.sum(), 1e-9) * 100.0),
        }
        if "clearance_mm" in ch:
            out["min_clearance_mm"] = float(np.nanmin(ch["clearance_mm"]))
        if "roll_deg" in ch:
            out["max_roll_deg"] = float(np.nanmax(np.abs(ch["roll_deg"])))
        if "yaw_deg" in ch:
            out["max_yaw_deg"] = float(np.nanmax(np.abs(ch["yaw_deg"])))
        if "rh_front_mm" in ch:
            out["min_rh_front_mm"] = float(np.nanmin(ch["rh_front_mm"]))
            out["min_rh_rear_mm"] = float(np.nanmin(ch["rh_rear_mm"]))
        if "path_error_m" in ch:
            out["max_path_error_m"] = float(np.nanmax(np.abs(ch["path_error_m"])))
        if "bottoming" in ch:
            out["bottoming_m"] = float((ch["bottoming"][:-1] * ds).sum())
        return out

    def to_csv(self, path: str | Path) -> None:
        cols = {"s_m": self.s, "t_s": self.t, "v_mps": self.v, "v_kmh": self.v * 3.6,
                "ax_mps2": self.ax, "ay_mps2": self.ay, "kappa_1pm": self.kappa}
        cols["aero_mode"] = np.array([self.mode_names[m] for m in self.mode], dtype=object)
        for k, v in self.channels.items():
            cols[k] = v
        keys = list(cols)
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(keys)
            for i in range(len(self.s)):
                row = []
                for k in keys:
                    val = cols[k][i]
                    if isinstance(val, (float, np.floating)):
                        row.append(f"{float(val):.6g}")
                    else:
                        row.append(val)
                w.writerow(row)


def powertrain_channels(vehicle, v, ax, f_drive, dt, f_brake=None) -> tuple[dict[str, np.ndarray], float]:
    """Battery / motor channels and the energy drawn over a lap.

    ``f_drive`` is the tractive force at the tyres; the motor additionally accelerates the rotating
    driveline (``powertrain.m_rot``). ``dt`` is the time of each interval between samples.
    """
    pt = vehicle.powertrain
    f_motor = np.maximum(f_drive, 0.0) + pt.m_rot * np.maximum(ax, 0.0) * (np.asarray(f_drive) > 0.0)
    p_src = pt.battery_power(v, f_motor)
    if f_brake is not None and pt.regen_power > 0.0:
        p_regen = pt.regen_battery_power(v, f_brake)
        p_src = p_src - p_regen
    else:
        p_regen = None
    ch = {"drive_power_kW": np.maximum(f_drive, 0.0) * v / 1000.0}
    if pt.type == "ev":
        rpm, torque, eta = pt.motor_point(v, f_motor)
        ch["battery_power_kW"] = p_src / 1000.0
        if p_regen is not None:
            ch["regen_power_kW"] = p_regen / 1000.0
        ch["motor_rpm"] = rpm
        ch["motor_torque_Nm"] = torque
        if pt.motor is not None:
            ch["motor_efficiency"] = np.where(torque > 0.5, eta, np.nan)
    energy = float((0.5 * (p_src[1:] + p_src[:-1]) * dt).sum())
    return ch, energy
