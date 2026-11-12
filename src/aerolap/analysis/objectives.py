"""Objectives and setup constraints shared by sweeps and the optimiser.

Objective spec (YAML / dict)::

    type: lap_time        # lap_time | endurance_time | skidpad_time | ay_max
    track: ../tracks/fsae_autocross_example.csv   # lap_time / endurance_time
    distance_km: 22       # endurance_time only (energy-limited: power cap if the battery is short)
    start: standing       # lap_time only
    solver: qss           # lap_time only: qss | transient (ride dynamics, optional rough road)
    road: B               # transient only: null/flat, ISO 8608 class A..E, or a profile CSV
    road_seed: 1
    transient_iterations: 2
    speeds_kmh: [40, 60, 80, 100]                 # ay_max only
    weights: [1, 1, 1, 1]                         # ay_max only
    level: full           # model level L0..L5 / full

Constraint spec (violations are normalised: 1 mm clearance, 0.1 deg roll, 0.05 understeer
index, 0.1 Hz ride frequency; the penalty is ``penalty_weight * sum(violation^2)``)::

    min_clearance_mm: 5           # floor clearance at the braking / cornering limits (and on the lap)
    max_roll_deg: 1.5             # at the cornering limit
    understeer_index: [1.0, 1.3]  # u_front / u_rear at the cornering limit (> 1: front saturates first)
    ride_frequency_hz: [3.0, 5.0] # static heave ride frequency, both axles
    check_speeds_kmh: [30, 50, 70, 90, 110]
    battery_energy:               # flat-out endurance must fit the usable battery energy
      track: ../tracks/fsae_endurance_example.csv
      distance_km: 22
      margin_kWh: 0.2             # reserve kept unused (normalised per 0.1 kWh)
    cooling_margin_kW: 0          # sidepod radiators must reject the lap-average powertrain heat
                                  # + this margin (lap_time / endurance_time; normalised per 0.5 kW)
    max_stall_ratio: 0.95         # every wing element below this share of its maximum lift at the
                                  # static attitude (1 = stall; normalised per 0.02)
    min_wing_clearance_mm: 15     # lowest point of every ground-effect wing (tips with roll) on the lap
                                  # and at the braking / cornering limits (normalised per 1 mm)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..features import features_for_level
from ..solvers.ggv import SolverOptions, ay_max_curve
from ..solvers.qss import QSSSolver
from ..solvers.transient import TransientOptions, TransientSolver
from ..track import Track
from ..vehicle import G, VehicleModel
from .cornering import braking_sweep, cornering_sweep
from .energy import endurance_energy
from .events import SKIDPAD_RADIUS


def wing_stall_ratio(car: VehicleModel) -> float | None:
    """Highest stall ratio of all wings (element model: the most loaded element), static attitude."""
    ratios = []
    for c in getattr(car.aero, "components", None) or []:
        dev = getattr(c, "dev", None)
        if hasattr(dev, "stall_ratio"):
            ratios.append(float(dev.stall_ratio()))
    return max(ratios) if ratios else None


def wing_clearance_mm(car: VehicleModel, rh_f_mm, rh_r_mm, roll_deg) -> float | None:
    """Lowest point of the ground-effect wings (the lower tip in roll) over the given attitudes (mm)."""
    geo = car.aero.geometry
    lows = []
    for c in getattr(car.aero, "components", None) or []:
        dev = getattr(c, "dev", None)
        if getattr(dev, "ground_effect", False) and hasattr(dev, "span"):
            h = dev.height + geo.local_change(dev.x, rh_f_mm, rh_r_mm)
            low = h - 500.0 * dev.span * np.sin(np.radians(np.abs(np.asarray(roll_deg, dtype=float))))
            if np.any(np.isfinite(low)):
                lows.append(float(np.nanmin(low)))
    return min(lows) if lows else None


def _resolve(path: str | Path, base_dir: str | Path) -> Path:
    p = Path(path)
    return p if p.is_absolute() else Path(base_dir) / p


@dataclass
class Evaluation:
    value: float  # objective value (minimised)
    metrics: dict[str, float]
    violations: dict[str, float] = field(default_factory=dict)  # normalised, > 0 = violated
    penalty: float = 0.0
    tol: float = 0.05  # violations up to this (normalised) size still count as feasible

    @property
    def total(self) -> float:
        return self.value + self.penalty

    @property
    def feasible(self) -> bool:
        return all(v <= self.tol for v in self.violations.values())


class Objective:
    """Callable ``cfg -> Evaluation``."""

    def __init__(self, spec: dict, base_dir: str | Path = ".", constraints: dict | None = None,
                 opts: SolverOptions | None = None, penalty_weight: float = 10.0,
                 feasibility_tol: float = 0.05):
        self.spec = dict(spec)
        self.type = spec.get("type", "lap_time")
        self.level = spec.get("level", "full")
        self.features = features_for_level(self.level)
        self.opts = opts or SolverOptions()
        self.constraints = dict(constraints or {})
        self.penalty_weight = penalty_weight
        self.feasibility_tol = feasibility_tol
        if self.type == "lap_time":
            self.track = Track.from_file(_resolve(spec["track"], base_dir))
            self.start = spec.get("start", "flying" if self.track.closed else "standing")
            self.solver = spec.get("solver", "qss")
            if self.solver not in ("qss", "transient"):
                raise ValueError("objective.solver must be 'qss' or 'transient'")
            road = spec.get("road")
            if road is not None and str(road).upper() not in ("A", "B", "C", "D", "E", "FLAT", "NONE"):
                rp = Path(road)
                road = str(rp if rp.is_absolute() else Path(base_dir) / rp)
            self.transient_opts = TransientOptions(
                iterations=int(spec.get("transient_iterations", 2)), road=road,
                road_seed=int(spec.get("road_seed", 1)), road_scale=float(spec.get("road_scale", 1.0)),
                qss=self.opts)
            extra = f", transient, road {road}" if self.solver == "transient" else ""
            self.label, self.unit = f"lap time ({self.track.name}{extra})", "s"
        elif self.type == "endurance_time":
            self.track = Track.from_file(_resolve(spec["track"], base_dir), closed=True)
            self.distance_km = float(spec.get("distance_km", 22.0))
            self.label, self.unit = f"endurance time ({self.track.name}, {self.distance_km:g} km)", "s"
        elif self.type == "skidpad_time":
            self.radius = float(spec.get("radius_m", SKIDPAD_RADIUS))
            self.label, self.unit = "skidpad time", "s"
        elif self.type == "ay_max":
            self.speeds = np.asarray(spec.get("speeds_kmh", [40, 60, 80, 100]), dtype=float) / 3.6
            w = np.asarray(spec.get("weights", np.ones(self.speeds.size)), dtype=float)
            self.weights = w / w.sum()
            self.label, self.unit = "-weighted ay_max", "g"
        else:
            raise ValueError(f"unknown objective type {self.type!r} "
                             "(lap_time | endurance_time | skidpad_time | ay_max)")
        be = self.constraints.get("battery_energy")
        self.energy_track = None
        if be:
            self.energy_track = Track.from_file(_resolve(be["track"], base_dir), closed=True)
            self.energy_distance_km = float(be.get("distance_km", 22.0))
            self.energy_margin_kWh = float(be.get("margin_kWh", 0.0))
        self.check_speeds = np.asarray(self.constraints.get("check_speeds_kmh", [30, 50, 70, 90, 110]),
                                       dtype=float) / 3.6

    # ------------------------------------------------------------------
    def __call__(self, cfg: dict) -> Evaluation:
        car = VehicleModel(cfg, self.features)
        metrics: dict[str, float] = {}
        lap = None
        if self.type == "lap_time":
            if self.solver == "transient":
                res = TransientSolver(self.transient_opts).solve(car, self.track, start=self.start)
                metrics["tyre_load_std_N"] = res.info["tyre_load_std_N"]
                metrics["qss_lap_time_s"] = res.info["qss_lap_time_s"]
            else:
                res = QSSSolver(self.opts).solve(car, self.track, start=self.start)
            value = res.lap_time
            lap = res
            sm = res.summary()
            metrics["lap_min_clearance_mm"] = sm["min_clearance_mm"]
            metrics["lap_max_roll_deg"] = sm["max_roll_deg"]
        elif self.type == "endurance_time":
            en = endurance_energy(cfg, self.track, self.distance_km, self.features, self.opts)
            value = en.total_time_s
            metrics.update(self._energy_metrics(en))
            lap = en.result
            sm = en.result.summary()
            metrics["lap_min_clearance_mm"] = sm["min_clearance_mm"]
            metrics["lap_max_roll_deg"] = sm["max_roll_deg"]
        elif self.type == "skidpad_time":
            value = self._skidpad_time(car)
        else:
            ay = ay_max_curve(car, car.default_mode, self.speeds, self.opts)
            value = -float((self.weights * ay).sum() / G)
            for v, a in zip(self.speeds, ay):
                metrics[f"ay_max_g@{v * 3.6:.0f}kmh"] = float(a / G)
        if self.energy_track is not None and "endurance_flat_out_kWh" not in metrics:
            en = endurance_energy(cfg, self.energy_track, self.energy_distance_km, self.features, self.opts,
                                  derate=False)
            metrics.update(self._energy_metrics(en))
        if lap is not None:
            from .cooling import cooling_balance

            cb = cooling_balance(car, lap)
            if cb is not None:
                metrics.update({"cooling_heat_kW": cb.produced_kW, "cooling_rejected_kW": cb.rejected_kW,
                                "cooling_margin_kW": cb.margin_kW})
        metrics.update(self.limit_metrics(car))
        ch = getattr(lap, "channels", None) or {}
        if "wing_min_clearance_mm" in metrics and all(k in ch for k in ("rh_front_mm", "rh_rear_mm", "roll_deg")):
            low = wing_clearance_mm(car, ch["rh_front_mm"], ch["rh_rear_mm"], ch["roll_deg"])
            if low is not None:
                metrics["wing_min_clearance_mm"] = min(metrics["wing_min_clearance_mm"], low)
        stall = wing_stall_ratio(car)
        if stall is not None:
            metrics["wing_stall_ratio_max"] = stall
        ev = Evaluation(value, metrics, tol=self.feasibility_tol)
        self._apply_constraints(ev)
        return ev

    @staticmethod
    def _energy_metrics(en) -> dict[str, float]:
        b = en.breakdown
        return {
            "endurance_flat_out_kWh": en.unlimited_required_kWh,
            "endurance_required_kWh": en.required_kWh,
            "battery_usable_kWh": np.nan if en.usable_kWh is None else en.usable_kWh,
            "endurance_power_cap_kW": np.nan if en.power_cap_kW is None else en.power_cap_kW,
            "endurance_lap_s": en.lap_time_s,
            "aero_drag_kWh_per_lap": b.get("aero_drag_kWh", np.nan),
        }

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
        wing = [w for w in (wing_clearance_mm(car, d["rh_front_mm"], d["rh_rear_mm"], d["roll_deg"])
                            for d in (corner, brake)) if w is not None]
        return {
            **({"wing_min_clearance_mm": min(wing)} if wing else {}),
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
        if self.energy_track is not None:
            usable = m.get("battery_usable_kWh", np.nan)
            if np.isfinite(usable):
                viol["battery_energy"] = (m["endurance_flat_out_kWh"] - (usable - self.energy_margin_kWh)) / 0.1
        if "cooling_margin_kW" in c and "cooling_margin_kW" in m:
            viol["cooling"] = (float(c["cooling_margin_kW"]) - m["cooling_margin_kW"]) / 0.5
        if c.get("min_wing_clearance_mm") is not None and "wing_min_clearance_mm" in m:
            viol["wing_clearance"] = (float(c["min_wing_clearance_mm"]) - m["wing_min_clearance_mm"]) / 1.0
        if c.get("max_stall_ratio") is not None and "wing_stall_ratio_max" in m:
            viol["wing_stall"] = (m["wing_stall_ratio_max"] - float(c["max_stall_ratio"])) / 0.02
        ev.penalty = self.penalty_weight * sum(max(v, 0.0) ** 2 for v in viol.values())
