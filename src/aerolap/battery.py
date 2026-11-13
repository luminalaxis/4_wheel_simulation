"""Cell-based accumulator: open-circuit voltage vs SOC, internal resistance, current limits.

Enabled when ``battery.cell`` is present (otherwise the battery is only a fixed power limit and an
energy budget, see :mod:`aerolap.powertrain`)::

    battery:
      series: 132
      parallel: 4
      soc: 1.0                         # state of charge at the start of this lap
      soc_min: 0.05                    # lowest SOC the run may use
      interconnect_resistance_mOhm: 30 # bus bars, fuse, AIRs
      max_current_A: null              # pack limit (fuse ...); null -> cell limit x parallel
      soc_stages: 3                    # endurance: SOC steps (lap re-solved at each)
      cell:
        capacity_Ah: 4.0
        resistance_mOhm: 13            # DC internal resistance
        max_current_A: 45
        v_min: 2.8
        ocv: [[0, 3.0], [0.1, 3.35], [0.2, 3.5], [0.5, 3.68], [0.8, 3.95], [1.0, 4.18]]

Pack: ``V_oc = series * OCV(SOC)``, ``R = series * R_cell f(T, SOC) / parallel + R_ic``. For a terminal power
``P`` (positive = discharge) the current is ``I = (V_oc - sqrt(V_oc^2 - 4 R P)) / (2 R)`` and the
terminal voltage ``V_t = V_oc - R I``; the energy taken from the cells is ``V_oc I = P + R I^2``.

Optional cell resistance factors (both multiply ``resistance_mOhm``, linear interpolation, end values
outside)::

      cell:
        resistance_temp: [[0, 2.0], [25, 1.0], [45, 0.8], [60, 0.75]]   # [cell temperature C, factor]
        resistance_soc: [[0.0, 1.5], [0.2, 1.1], [0.5, 1.0], [1.0, 1.0]]  # [SOC, factor]
    temperature_C: 30                  # cell temperature for the factor (the endurance thermal model sets it)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class BatteryPack:
    series: int
    parallel: int
    capacity_Ah: float  # per cell
    r_cell: float  # ohm
    r_ic: float  # ohm
    i_cell_max: float  # A
    i_pack_max: float | None  # A
    v_cell_min: float
    ocv_soc: np.ndarray
    ocv_v: np.ndarray
    soc: float = 1.0
    soc_min: float = 0.05
    soc_stages: int = 3
    r_temp: np.ndarray | None = None  # (n, 2) [cell temperature C, resistance factor]
    r_soc: np.ndarray | None = None  # (n, 2) [SOC, resistance factor]
    temperature: float = 25.0  # cell temperature, C

    @classmethod
    def from_config(cls, b: dict) -> "BatteryPack | None":
        c = (b or {}).get("cell")
        if not c:
            return None
        ocv = np.asarray(c.get("ocv") or [[0.0, 3.0], [0.1, 3.35], [0.2, 3.5], [0.5, 3.68], [0.8, 3.95],
                                          [1.0, 4.18]], dtype=float)
        if ocv.ndim != 2 or ocv.shape[1] != 2 or len(ocv) < 2:
            raise ValueError("battery.cell.ocv must be a list of [SOC, V] pairs")
        order = np.argsort(ocv[:, 0])
        imax = b.get("max_current_A")
        th = b.get("thermal") or {}
        temp = b.get("temperature_C")
        if temp is None:
            temp = th.get("start_C", 25.0) if th else 25.0
        return cls(
            series=int(b["series"]), parallel=int(b["parallel"]), capacity_Ah=float(c["capacity_Ah"]),
            r_cell=float(c.get("resistance_mOhm", 15.0)) / 1000.0,
            r_ic=float(b.get("interconnect_resistance_mOhm") or 0.0) / 1000.0,
            i_cell_max=float(c.get("max_current_A", 30.0)), i_pack_max=None if imax is None else float(imax),
            v_cell_min=float(c.get("v_min", 2.8)), ocv_soc=ocv[order, 0], ocv_v=ocv[order, 1],
            soc=float(b.get("soc", 1.0)), soc_min=float(b.get("soc_min", 0.05)),
            soc_stages=max(1, int(b.get("soc_stages", 3))),
            r_temp=_table(c.get("resistance_temp"), "battery.cell.resistance_temp"),
            r_soc=_table(c.get("resistance_soc"), "battery.cell.resistance_soc"),
            temperature=float(temp),
        )

    # ------------------------------------------------------------------ static properties
    def r_factor(self, soc=None, temperature=None) -> float:
        """Cell resistance factor at a temperature [C] and SOC (1 without tables)."""
        f = 1.0
        if self.r_temp is not None:
            t = self.temperature if temperature is None else temperature
            f *= float(np.interp(t, self.r_temp[:, 0], self.r_temp[:, 1]))
        if self.r_soc is not None:
            s = self.soc if soc is None else soc
            f *= float(np.interp(s, self.r_soc[:, 0], self.r_soc[:, 1]))
        return f

    @property
    def resistance(self) -> float:
        if self.r_temp is None and self.r_soc is None:
            return self.series * self.r_cell / self.parallel + self.r_ic
        return self.series * self.r_cell * self.r_factor() / self.parallel + self.r_ic

    def cell_resistance(self, soc=None, temperature=None) -> float:
        """Resistance of the cells only (series / parallel, no interconnect), ohm."""
        return self.series * self.r_cell * self.r_factor(soc, temperature) / self.parallel

    @property
    def charge_C(self) -> float:
        return self.parallel * self.capacity_Ah * 3600.0

    def ocv(self, soc=None) -> np.ndarray:
        s = self.soc if soc is None else soc
        return self.series * np.interp(np.clip(s, 0.0, 1.0), self.ocv_soc, self.ocv_v)

    def energy_between(self, soc_hi: float, soc_lo: float) -> float:
        """Cell energy (J) released going from ``soc_hi`` to ``soc_lo``."""
        s = np.linspace(soc_lo, soc_hi, 201)
        v = self.ocv(s)
        return float(np.sum(0.5 * (v[1:] + v[:-1]) * np.diff(s))) * self.charge_C

    @property
    def nominal_energy(self) -> float:
        return self.energy_between(1.0, 0.0)

    @property
    def usable_energy(self) -> float:
        return self.energy_between(self.soc, self.soc_min)

    # ------------------------------------------------------------------ at the current SOC
    def max_current(self, soc=None) -> float:
        v_oc = float(self.ocv(soc))
        lim = [self.parallel * self.i_cell_max, (v_oc - self.series * self.v_cell_min) / self.resistance,
               v_oc / (2.0 * self.resistance)]  # beyond V_oc / 2R the terminal power falls again
        if self.i_pack_max is not None:
            lim.append(self.i_pack_max)
        return max(min(lim), 0.0)

    def max_power(self, soc=None) -> float:
        """Largest terminal (discharge) power, W."""
        i = self.max_current(soc)
        return float(self.ocv(soc)) * i - self.resistance * i * i

    def current(self, p_terminal, soc=None):
        """Pack current (A, + = discharge) for a terminal power (W)."""
        v = self.ocv(soc)
        r = self.resistance
        p = np.minimum(np.asarray(p_terminal, dtype=float), v * v / (4.0 * r))
        return (v - np.sqrt(np.maximum(v * v - 4.0 * r * p, 0.0))) / (2.0 * r)

    def terminal_voltage(self, p_terminal, soc=None):
        return self.ocv(soc) - self.resistance * self.current(p_terminal, soc)

    def describe(self, soc=None) -> str:
        s = self.soc if soc is None else soc
        return (f"{self.series}S{self.parallel}P, SOC {s:.2f}: OCV {float(self.ocv(s)):.0f} V, R {self.resistance * 1000:.0f} "
                f"mOhm, I_max {self.max_current(s):.0f} A, P_max {self.max_power(s) / 1000:.1f} kW")


def _table(value, path: str) -> np.ndarray | None:
    if value is None:
        return None
    try:
        arr = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        arr = np.zeros((0, 2))
    if arr.ndim != 2 or arr.shape[1] != 2 or len(arr) < 1 or np.any(arr[:, 1] <= 0):
        from .config import ConfigError

        raise ConfigError(path=path, value=value, condition="[x, 배율] 쌍의 목록 (배율 > 0)",
                          hint="예: [[0, 2.0], [25, 1.0], [60, 0.75]]")
    return arr[np.argsort(arr[:, 0])]
