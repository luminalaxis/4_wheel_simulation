"""Lumped thermal models of the accumulator cells and the motor winding over an endurance.

Both are one thermal mass ``C`` connected to a coolant / ambient temperature through a conductance ``h``::

    C dT/dt = q(t) - h (T - T_amb)          ->  T(t + dt) = T_inf + (T - T_inf) exp(-dt h / C),  T_inf = T_amb + q / h

integrated exactly for a heat rate ``q`` that is constant over each time step of the lap trace (the lap
is repeated for the laps of each SOC stage).

* Cells (``battery.thermal``): ``q = N_s R_cell f(T, SOC) / N_p I^2`` from the pack current of the lap
  (bus bars / fuse ``R_ic`` heat the pack enclosure, not the cells), ``C = N_s N_p m_cell c_p``.
  ``f(T, SOC)`` are the optional ``battery.cell.resistance_temp`` / ``resistance_soc`` tables.
* Motor winding (``powertrain.ev.motor_thermal``, motor model only): ``q`` = copper + speed losses of the
  motor model; ``h`` is calibrated so the continuous (S1) rating holds the winding at its limit with the
  coolant at ``coolant_C``; ``C`` = motor mass x ``cp`` (or ``heat_capacity_J_per_K``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

TOL_K = 0.05  # temperature limits are met within this (the power-cap search stops there)


@dataclass
class LumpedThermal:
    name: str
    heat_capacity: float  # J/K
    conductance: float  # W/K
    ambient: float  # deg C (coolant / cooling air)
    start: float  # deg C
    limit: float | None  # deg C
    derate: bool = True  # the endurance power cap also keeps this below its limit

    @property
    def time_constant(self) -> float:
        return self.heat_capacity / self.conductance if self.conductance > 0 else math.inf

    def advance(self, temp: float, q: np.ndarray, dt: np.ndarray, heat_fn=None) -> tuple[float, float]:
        """Temperature after a trace of heat rates ``q`` over steps ``dt`` -> (end, peak).

        ``heat_fn(i, T)`` (optional) recomputes the heat rate of step i at the current temperature.
        """
        c, h, amb = self.heat_capacity, self.conductance, self.ambient
        peak = temp
        if h <= 0.0:
            if heat_fn is None:
                out = temp + np.cumsum(q * dt) / c
                return float(out[-1]), float(max(temp, out.max()))
            for i in range(len(dt)):
                temp += heat_fn(i, temp) * dt[i] / c
                peak = max(peak, temp)
            return temp, peak
        decay = np.exp(-dt * h / c)
        if heat_fn is None:
            for qi, di in zip(q.tolist(), decay.tolist()):
                t_inf = amb + qi / h
                temp = t_inf + (temp - t_inf) * di
                if temp > peak:
                    peak = temp
            return temp, peak
        for i, di in enumerate(decay.tolist()):
            t_inf = amb + heat_fn(i, temp) / h
            temp = t_inf + (temp - t_inf) * di
            if temp > peak:
                peak = temp
        return temp, peak


def pack_thermal(battery: dict, pack) -> LumpedThermal | None:
    th = (battery or {}).get("thermal")
    if not th or pack is None:
        return None
    cells = pack.series * pack.parallel
    c = float(th.get("heat_capacity_J_per_K") or cells * float(th.get("cell_mass_g", 70.0)) / 1000.0
              * float(th.get("cp_J_per_kgK", 1000.0)))
    start = float(th.get("start_C", 30.0))
    lim = th.get("max_C", 60.0)
    return LumpedThermal("cells", c, float(th.get("cooling_W_per_K", 0.0)), float(th.get("ambient_C", 25.0)), start,
                         None if lim is None else float(lim), bool(th.get("derate", True)))


def motor_thermal(ev: dict, motor) -> LumpedThermal | None:
    th = (ev or {}).get("motor_thermal")
    if not th or motor is None:
        return None
    coolant = float(th.get("coolant_C", 50.0))
    lim = float(th.get("max_C", 120.0))
    # continuous rating: at the S1 torque and the speed where it gives the S1 power the winding is at its limit
    rpm = motor.cont_power / max(motor.cont_torque, 1e-9) * 60.0 / (2.0 * math.pi)
    q_cont = float(motor.losses(rpm, motor.cont_torque)) * motor_count(ev)
    h = float(th.get("conductance_W_per_K") or q_cont / max(lim - coolant, 1.0))
    c = float(th.get("heat_capacity_J_per_K") or motor.mass * float(th.get("cp_J_per_kgK", 900.0))) * motor_count(ev)
    return LumpedThermal("motor winding", c, h, coolant, float(th.get("start_C", coolant)), lim,
                         bool(th.get("derate", True)))


def motor_count(ev: dict) -> int:
    return max(int((ev or {}).get("n_motors", 1)), 1)


@dataclass
class ThermalRun:
    """Temperatures over an endurance (lap ends), peaks and limits."""

    time_s: list[float] = field(default_factory=list)
    cell_C: list[float] = field(default_factory=list)
    motor_C: list[float] = field(default_factory=list)
    cell: LumpedThermal | None = None
    motor: LumpedThermal | None = None
    cell_peak: float | None = None
    motor_peak: float | None = None

    @property
    def over_limit(self) -> list[str]:
        out = []
        if (self.cell and self.cell.limit is not None and self.cell_peak is not None
                and self.cell_peak > self.cell.limit + TOL_K):
            out.append("cells")
        if (self.motor and self.motor.limit is not None and self.motor_peak is not None
                and self.motor_peak > self.motor.limit + TOL_K):
            out.append("motor")
        return out

    def excess_K(self, derating_only: bool = True) -> float:
        """Largest exceedance of a temperature limit (K, negative = margin) of the models that derate."""
        vals = []
        for model, peak in ((self.cell, self.cell_peak), (self.motor, self.motor_peak)):
            if model is not None and model.limit is not None and peak is not None and (model.derate or not derating_only):
                vals.append(peak - model.limit)
        return max(vals) if vals else -math.inf

    def report_lines(self) -> list[str]:
        lines = []
        if self.cell is not None and self.cell_peak is not None:
            c = self.cell
            lim = "" if c.limit is None else f" (limit {c.limit:.0f} -> {'OVER' if self.cell_peak > c.limit + TOL_K else 'ok'})"
            lines.append(f"  cell temperature: {c.start:.1f} -> {self.cell_C[-1]:.1f} C, peak {self.cell_peak:.1f} C{lim}; "
                         f"{c.heat_capacity / 1000:.0f} kJ/K, cooling {c.conductance:.0f} W/K to {c.ambient:.0f} C "
                         f"(time constant {c.time_constant / 60:.0f} min)")
        if self.motor is not None and self.motor_peak is not None:
            m = self.motor
            lines.append(f"  motor winding: {m.start:.1f} -> {self.motor_C[-1]:.1f} C, peak {self.motor_peak:.1f} C "
                         f"(limit {m.limit:.0f} -> {'OVER' if self.motor_peak > m.limit + TOL_K else 'ok'}); "
                         f"R_th {1.0 / m.conductance:.4f} K/W from the S1 rating, time constant {m.time_constant / 60:.1f} min")
        return lines
