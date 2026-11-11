"""Setup optimisation (springs, ARBs, ride heights, aero balance ...) for cornering / lap time.

Spec file (YAML, paths relative to the spec file)::

    vehicle: ../vehicles/fsae_ev_aero.yaml
    objective:
      type: lap_time            # lap_time | endurance_time | skidpad_time | ay_max  (see objectives.py)
      track: ../tracks/fsae_autocross_example.csv
      start: standing
    variables:
      - {path: suspension.front.arb_rate_Nm_per_deg, bounds: [50, 500]}
      - {path: suspension.front.static_ride_height_mm, bounds: [20, 50]}
    constraints:
      min_clearance_mm: 5
      max_roll_deg: 1.5
      understeer_index: [1.0, 1.25]
      ride_frequency_hz: [3.0, 5.0]
    method: powell              # powell | differential_evolution
    max_evals: 150
    coarse: true                # coarse envelope while searching, exact re-evaluation at the end
"""

from __future__ import annotations

import copy
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution, minimize

from ..config import get_path, load_vehicle_config, load_yaml, with_overrides
from ..solvers.ggv import SolverOptions
from .objectives import Evaluation, Objective


@dataclass
class Variable:
    path: str
    lo: float
    hi: float
    integer: bool = False

    def to_unit(self, x: float) -> float:
        return (x - self.lo) / (self.hi - self.lo)

    def from_unit(self, u: float) -> float:
        x = self.lo + float(np.clip(u, 0.0, 1.0)) * (self.hi - self.lo)
        return float(round(x)) if self.integer else x


@dataclass
class OptimizationSpec:
    vehicle_cfg: dict
    objective_spec: dict
    variables: list[Variable]
    constraints: dict
    base_dir: Path
    method: str = "powell"
    max_evals: int = 150
    coarse: bool = True
    seed: int = 1
    penalty_weight: float = 10.0
    feasibility_tol: float = 0.05
    n_refine: int = 5  # best coarse candidates re-evaluated with the fine envelope

    @classmethod
    def from_file(cls, path: str | Path) -> "OptimizationSpec":
        path = Path(path)
        raw = load_yaml(path)
        base = path.resolve().parent
        veh = Path(raw["vehicle"])
        if not veh.is_absolute():
            veh = base / veh
        variables = []
        for v in raw["variables"]:
            lo, hi = (float(b) for b in v["bounds"])
            if hi <= lo:
                raise ValueError(f"variable {v['path']}: upper bound must exceed lower bound")
            variables.append(Variable(v["path"], lo, hi, bool(v.get("integer", False))))
        return cls(
            vehicle_cfg=load_vehicle_config(veh), objective_spec=raw["objective"], variables=variables,
            constraints=raw.get("constraints", {}) or {}, base_dir=base,
            method=raw.get("method", "powell"), max_evals=int(raw.get("max_evals", 150)),
            coarse=bool(raw.get("coarse", True)), seed=int(raw.get("seed", 1)),
            penalty_weight=float(raw.get("penalty_weight", 10.0)),
            feasibility_tol=float(raw.get("feasibility_tol", 0.05)),
            n_refine=int(raw.get("n_refine", 5)),
        )


@dataclass
class HistoryEntry:
    n: int
    x: dict[str, float]
    value: float
    total: float
    feasible: bool


@dataclass
class OptimizationResult:
    spec: OptimizationSpec
    initial: dict[str, float]
    best: dict[str, float]
    initial_eval: Evaluation
    best_eval: Evaluation
    history: list[HistoryEntry] = field(default_factory=list)
    best_cfg: dict | None = None
    elapsed_s: float = 0.0
    unit: str = ""
    label: str = ""

    def report(self) -> str:
        lines = [f"objective: {self.label}  [{self.unit}]   evaluations: {len(self.history)}   "
                 f"time: {self.elapsed_s:.0f} s", ""]
        lines.append(f"{'variable':45s} {'initial':>10s} {'optimum':>10s} {'bounds':>18s}")
        lines.append("-" * 86)
        for v in self.spec.variables:
            lines.append(f"{v.path:45s} {self.initial[v.path]:10.4g} {self.best[v.path]:10.4g} "
                         f"{'[' + f'{v.lo:g}, {v.hi:g}' + ']':>18s}")
        lines.append("")
        i, b = self.initial_eval, self.best_eval
        lines.append(f"{'':28s} {'initial':>12s} {'optimum':>12s} {'change':>10s}")
        lines.append(f"{'objective':28s} {i.value:12.4f} {b.value:12.4f} {b.value - i.value:+10.4f}")
        for k in sorted(set(i.metrics) | set(b.metrics)):
            vi, vb = i.metrics.get(k, math.nan), b.metrics.get(k, math.nan)
            lines.append(f"{k:28s} {vi:12.4f} {vb:12.4f} {vb - vi:+10.4f}")
        lines.append("")
        lines.append("constraints (normalised violation, <= 0 is satisfied):")
        for k in sorted(b.violations):
            lines.append(f"  {k:30s} initial {i.violations.get(k, math.nan):+8.3f}   "
                         f"optimum {b.violations[k]:+8.3f}")
        lines.append("feasible: " + ("yes" if b.feasible else "NO - widen the bounds or relax constraints"))
        return "\n".join(lines)


def run_optimization(spec: OptimizationSpec, progress=None) -> OptimizationResult:
    t0 = time.time()
    cfg0 = spec.vehicle_cfg
    vars_ = spec.variables
    search_opts = SolverOptions.coarse() if spec.coarse else SolverOptions()
    obj_search = Objective(spec.objective_spec, spec.base_dir, spec.constraints, search_opts,
                           spec.penalty_weight, spec.feasibility_tol)
    obj_final = Objective(spec.objective_spec, spec.base_dir, spec.constraints, SolverOptions(),
                          spec.penalty_weight, spec.feasibility_tol)

    initial = {v.path: float(get_path(cfg0, v.path)) for v in vars_}
    cache: dict[tuple, Evaluation] = {}
    history: list[HistoryEntry] = []

    def overrides(u) -> dict[str, float]:
        return {v.path: v.from_unit(ui) for v, ui in zip(vars_, u)}

    def f(u) -> float:
        over = overrides(u)
        key = tuple(round(x, 9) for x in over.values())
        if key not in cache:
            try:
                ev = obj_search(with_overrides(cfg0, over, "최적화 탐색"))
            except ValueError as exc:  # e.g. roll-unstable setup
                ev = Evaluation(math.inf, {}, {"invalid": 1.0}, 1e6)
                ev.metrics["error"] = 0.0
                if progress:
                    progress(len(history) + 1, over, None, str(exc))
            cache[key] = ev
            history.append(HistoryEntry(len(history) + 1, over, ev.value, ev.total, ev.feasible))
            if progress and math.isfinite(ev.value):
                progress(len(history), over, ev, None)
        ev = cache[key]
        return ev.total if math.isfinite(ev.total) else 1e9

    u0 = np.array([np.clip(v.to_unit(initial[v.path]), 0.0, 1.0) for v in vars_])
    f(u0)
    if spec.method == "powell":
        minimize(f, u0, method="Powell", bounds=[(0.0, 1.0)] * len(vars_),
                 options={"maxfev": spec.max_evals, "xtol": 1e-2, "ftol": 1e-4})
    elif spec.method == "differential_evolution":
        pop = max(5, int(spec.max_evals / max(1, 10 * len(vars_))))
        differential_evolution(f, [(0.0, 1.0)] * len(vars_), seed=spec.seed, x0=u0, polish=False,
                               popsize=pop, maxiter=max(1, spec.max_evals // max(1, pop * len(vars_)) - 1),
                               tol=1e-6, init="sobol")
    else:
        raise ValueError("method must be 'powell' or 'differential_evolution'")

    # Re-evaluate the best coarse candidates (penalised objective) with the fine envelope and
    # keep the best feasible one; the start point is always a candidate.
    ranked, seen = [], set()
    for h in sorted(history, key=lambda h: h.total):
        key = tuple(round(x, 6) for x in h.x.values())
        if key not in seen and math.isfinite(h.total):
            seen.add(key)
            ranked.append(h.x)
        if len(ranked) >= max(1, spec.n_refine):
            break
    initial_eval = obj_final(copy.deepcopy(cfg0))
    finals = [(dict(initial), initial_eval)]
    finals += [(x, obj_final(with_overrides(cfg0, x, "최적화 후보 정밀 재평가"))) for x in ranked if x != initial]
    best, best_eval = min(finals, key=lambda c: (not c[1].feasible, c[1].value if c[1].feasible
                                                 else c[1].total))
    cfg_best = with_overrides(cfg0, best, "최적 셋업 적용")
    return OptimizationResult(spec, initial, best, initial_eval, best_eval, history, cfg_best,
                              time.time() - t0, obj_final.unit, obj_final.label)
