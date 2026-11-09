"""Objectives and setup constraints shared by sweeps and the optimiser.

Objective spec (YAML / dict)::

    type: lap_time        # lap_time | skidpad_time | ay_max
    track: ../tracks/fsae_autocross_example.csv   # lap_time only
    start: standing       # lap_time only
    speeds_kmh: [40, 60, 80, 100]                 # ay_max only
    weights: [1, 1, 1, 1]                         # ay_max only
    level: full           # model level L0..L5 / full

Constraint spec::

    min_clearance_mm: 5           # floor clearance at the braking / cornering limits (and on the lap)
    max_roll_deg: 1.5             # at the cornering limit
    understeer_index: [1.0, 1.3]  # u_front / u_rear at the cornering limit (> 1: front saturates first)
    ride_frequency_hz: [3.0, 5.0] # static heave ride frequency, both axles
    check_speeds_kmh: [30, 50, 70, 90, 110]
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..features import features_for_level
from ..solvers.ggv import SolverOptions, ay_max_curve
from ..solvers.qss import QSSSolver
from ..track import Track
from ..vehicle import G, VehicleModel
from .cornering import braking_sweep, cornering_sweep
from .events import SKIDPAD_RADIUS


@dataclass
class Evaluation:
    value: float  # objective value (minimised)
    metrics: dict[str, float]
    violations: dict[str, float] = field(default_factory=dict)  # normalised, > 0 = violated
    penalty: float = 0.0

    @property
    def total(self) -> float:
        return self.value + self.penalty

    @property
    def feasible(self) -> bool:
        return all(v <= 1e-9 for v in self.violations.values())


class Objective:
    """Callable ``cfg -> Evaluation``."""

    def __init__(self, spec: dict, base_dir: str | Path = ".", constraints: dict | None = None,
                 opts: SolverOptions | None = None, penalty_weight: float = 1.0):
        self.spec = dict(spec)
        self.type = spec.get("type", "lap_time")
        self.level = spec.get("level", "full")
        self.features = features_for_level(self.level)
        self.opts = opts or SolverOptions()
        self.constraints = dict(constraints or {})
        self.penalty_weight = penalty_weight
        if self.type == "lap_time":
            path = Path(spec["track"])
            if not path.is_absolute():
                path = Path(base_dir) / path
            self.track = Track.from_file(path)
            self.start = spec.get("start", "flying" if self.track.closed else "standing")
            self.label, self.unit = f"lap time ({self.track.name})", "s"
        elif self.type == "skidpad_time":
            self.radius = float(spec.get("radius_m", SKIDPAD_RADIUS))
            self.label, self.unit = "skidpad time", "s"
        elif self.type == "ay_max":
            self.speeds = np.asarray(spec.get("speeds_kmh", [40, 60, 80, 100]), dtype=float) / 3.6
            w = np.asarray(spec.get("weights", np.ones(self.speeds.size)), dtype=float)
            self.weights = w / w.sum()
            self.label, self.unit = "-weighted ay_max", "g"
        else:
            raise ValueError(f"unknown objective type {self.type!r} (lap_time | skidpad_time | ay_max)")
        self.check_speeds = np.asarray(self.constraints.get("check_speeds_kmh", [30, 50, 70, 90, 110]),
                                       dtype=float) / 3.6

    # ------------------------------------------------------------------
    def __call__(self, cfg: dict) -> Evaluation:
        car = VehicleModel(cfg, self.features)
        metrics: dict[str, float] = {}
        if self.type == "lap_time":
            res = QSSSolver(self.opts).solve(car, self.track, start=self.start)
            value = res.lap_time
            sm = res.summary()
            metrics["lap_min_clearance_mm"] = sm["min_clearance_mm"]
            metrics["lap_max_roll_deg"] = sm["max_roll_deg"]
        elif self.type == "skidpad_time":
            value = self._skidpad_time(car)
        else:
            ay = ay_max_curve(car, car.default_mode, self.speeds, self.opts)
            value = -float((self.weights * ay).sum() / G)
            for v, a in zip(self.speeds, ay):
                metrics[f"ay_max_g@{v * 3.6:.0f}kmh"] = float(a / G)
        metrics.update(self.limit_metrics(car))
        ev = Evaluation(value, metrics)
        self._apply_constraints(ev)
        return ev

    def _skidpad_time(self, car: VehicleModel) -> float:
        v_grid = np.linspace(4.0, 25.0, 43)
        best = 0.0
        modes = {car.default_mode}
        strat = car.active_strategy()
        if strat.type == "ideal":
            modes |= {strat.open_mode, strat.closed_mode}
        for m in modes:
            ay = ay_max_curve(car, m, v_grid, self.opts)
            g = ay - v_grid ** 2 / self.radius
            neg = np.flatnonzero(g < 0)
            if neg.size == 0:
                v = v_grid[-1]
            else:
                j = neg[0]
                if j == 0:
                    v = float(np.sqrt(max(ay[0], 0.0) * self.radius))
                else:
                    v = v_grid[j - 1] + (v_grid[j] - v_grid[j - 1]) * g[j - 1] / (g[j - 1] - g[j])
            best = max(best, float(v))
        return 2.0 * np.pi * self.radius / max(best, 1e-6)

    def limit_metrics(self, car: VehicleModel) -> dict[str, float]:
        corner = cornering_sweep(car, self.check_speeds, opts=self.opts)
        brake = braking_sweep(car, self.check_speeds, opts=self.opts)
        usi = corner["understeer_index"]
        usi = usi[np.isfinite(usi)]
        f_front, f_rear = car.ride_frequencies()
        return {
            "min_clearance_mm": float(min(corner["clearance_mm"].min(), brake["clearance_mm"].min())),
            "max_roll_deg": float(np.abs(corner["roll_deg"]).max()),
            "understeer_index_min": float(usi.min()) if usi.size else float("nan"),
            "understeer_index_max": float(usi.max()) if usi.size else float("nan"),
            "ride_freq_front_hz": f_front,
            "ride_freq_rear_hz": f_rear,
        }

    def _apply_constraints(self, ev: Evaluation) -> None:
        c, m, viol = self.constraints, ev.metrics, ev.violations
        if "min_clearance_mm" in c:
            clear = min(m["min_clearance_mm"], m.get("lap_min_clearance_mm", np.inf))
            viol["min_clearance"] = (float(c["min_clearance_mm"]) - clear) / 1.0
        if "max_roll_deg" in c:
            roll = max(m["max_roll_deg"], m.get("lap_max_roll_deg", 0.0))
            viol["max_roll"] = (roll - float(c["max_roll_deg"])) / 0.1
        if "understeer_index" in c:
            lo, hi = c["understeer_index"]
            if np.isfinite(m["understeer_index_min"]):
                viol["understeer_low"] = (float(lo) - m["understeer_index_min"]) / 0.05
                viol["understeer_high"] = (m["understeer_index_max"] - float(hi)) / 0.05
        if "ride_frequency_hz" in c:
            lo, hi = c["ride_frequency_hz"]
            for k in ("ride_freq_front_hz", "ride_freq_rear_hz"):
                viol[k + "_low"] = (float(lo) - m[k]) / 0.1
                viol[k + "_high"] = (m[k] - float(hi)) / 0.1
        ev.penalty = self.penalty_weight * sum(max(v, 0.0) ** 2 for v in viol.values())
