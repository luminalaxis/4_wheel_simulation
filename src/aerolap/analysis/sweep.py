"""Parameter sweeps (1-D / 2-D) and finite-difference sensitivities."""

from __future__ import annotations

import itertools
from dataclasses import dataclass

import numpy as np

from ..config import get_path, with_overrides
from .objectives import Evaluation, Objective


def parse_values(text: str) -> np.ndarray:
    """``"a:b:step"`` (inclusive) or ``"v1,v2,v3"``."""
    if ":" in text:
        a, b, step = (float(x) for x in text.split(":"))
        n = int(round((b - a) / step))
        return a + step * np.arange(n + 1)
    return np.array([float(x) for x in text.split(",")])


@dataclass
class SweepResult:
    paths: list[str]
    values: list[np.ndarray]
    evaluations: np.ndarray  # object array of Evaluation, shape = lengths of values

    def objective(self) -> np.ndarray:
        return np.vectorize(lambda e: e.value, otypes=[float])(self.evaluations)

    def metric(self, key: str) -> np.ndarray:
        return np.vectorize(lambda e: e.metrics.get(key, np.nan), otypes=[float])(self.evaluations)

    def feasible(self) -> np.ndarray:
        return np.vectorize(lambda e: e.feasible, otypes=[bool])(self.evaluations)

    def table(self) -> str:
        keys = ["min_clearance_mm", "lap_min_clearance_mm", "max_roll_deg", "understeer_index_min",
                "understeer_index_max"]
        head = "  ".join([f"{p.split('.')[-1][:22]:>22s}" for p in self.paths] + [f"{'objective':>11s}"]
                         + [f"{k[:14]:>14s}" for k in keys] + ["ok"])
        lines = [head, "-" * len(head)]
        for idx in itertools.product(*(range(len(v)) for v in self.values)):
            ev: Evaluation = self.evaluations[idx]
            cells = [f"{self.values[d][i]:22.4g}" for d, i in enumerate(idx)] + [f"{ev.value:11.4f}"]
            cells += [f"{ev.metrics.get(k, np.nan):14.3f}" for k in keys]
            cells.append("yes" if ev.feasible else "NO")
            lines.append("  ".join(cells))
        return "\n".join(lines)


def sweep(cfg: dict, paths: list[str], values: list[np.ndarray], objective: Objective,
          progress=None) -> SweepResult:
    shape = tuple(len(v) for v in values)
    evals = np.empty(shape, dtype=object)
    total = int(np.prod(shape))
    for k, idx in enumerate(itertools.product(*(range(n) for n in shape))):
        over = {p: float(values[d][i]) for d, (p, i) in enumerate(zip(paths, idx))}
        evals[idx] = objective(with_overrides(cfg, over, "파라미터 스윕"))
        if progress:
            progress(k + 1, total, over, evals[idx])
    return SweepResult(list(paths), [np.asarray(v) for v in values], evals)


@dataclass
class Sensitivity:
    path: str
    base: float
    step: float
    d_objective: float  # objective change for +step

    @property
    def per_unit(self) -> float:
        return self.d_objective / self.step


def sensitivities(cfg: dict, paths: list[str], objective: Objective, rel_step: float = 0.1,
                  abs_steps: dict[str, float] | None = None) -> tuple[Evaluation, list[Sensitivity]]:
    """Central differences of the objective w.r.t. each parameter (+/- step)."""
    base_eval = objective(cfg)
    out = []
    for p in paths:
        x0 = float(get_path(cfg, p))
        step = (abs_steps or {}).get(p) or (abs(x0) * rel_step if x0 != 0 else rel_step)
        up = objective(with_overrides(cfg, {p: x0 + step}, f"민감도 {p} +{step:g}")).value
        dn = objective(with_overrides(cfg, {p: x0 - step}, f"민감도 {p} -{step:g}")).value
        out.append(Sensitivity(p, x0, step, 0.5 * (up - dn)))
    return base_eval, out


def sensitivity_table(base: Evaluation, rows: list[Sensitivity], unit: str) -> str:
    lines = [f"base objective: {base.value:.4f} {unit}",
             f"{'parameter':45s} {'value':>10s} {'step':>9s} {'d obj/step':>12s}",
             "-" * 80]
    for r in sorted(rows, key=lambda r: -abs(r.d_objective)):
        lines.append(f"{r.path:45s} {r.base:10.4g} {r.step:9.3g} {r.d_objective:+12.4f}")
    return "\n".join(lines)
