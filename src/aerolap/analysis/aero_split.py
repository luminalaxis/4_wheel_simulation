"""Aero package split: best size of each device (front wing, sidepods / undertray, rear wing).

Each device in ``aero.components`` has a ``scale`` (size / angle; downforce ~ scale, drag ~
scale ** drag_exponent). Two studies:

* **individual** - sweep one device's scale with the others at their current value: shows how lap
  time (or energy-limited endurance time) depends on each device on its own;
* **joint** - optimise all scales together (Powell, coarse envelope, best candidates re-checked with
  the fine envelope) under the usual setup constraints, e.g. an understeer-index window so the
  aero balance stays stable. The devices interact through the balance (front wing vs rear wing),
  so the joint optimum is the one to build; the individual curves explain it.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..aero.maps import component_table, parse_components
from ..config import get_path, with_overrides
from .objectives import Evaluation, Objective
from .optimize import OptimizationResult, OptimizationSpec, Variable, run_optimization


def scale_path(name: str) -> str:
    return f"aero.components.{name}.scale"


@dataclass
class AeroSplitResult:
    names: list[str]
    values: np.ndarray  # sweep grid of scales
    sweeps: dict[str, list[Evaluation]] = field(default_factory=dict)  # search accuracy (coarse envelope)
    base: Evaluation | None = None  # current package, search accuracy
    base_fine: Evaluation | None = None  # current package, fine envelope (same basis as the joint optimum)
    best_fine: dict[str, tuple[float, Evaluation]] = field(default_factory=dict)  # per device, fine envelope
    joint: OptimizationResult | None = None
    warning: str = ""  # e.g. a balance window no aero change can reach
    unit: str = ""
    label: str = ""

    def best_individual(self, name: str) -> tuple[float, Evaluation]:
        evs = self.sweeps[name]
        ok = [i for i, e in enumerate(evs) if e.feasible] or list(range(len(evs)))
        i = min(ok, key=lambda k: evs[k].value)
        return float(self.values[i]), evs[i]

    def report(self, cfg: dict) -> str:
        lines = [f"objective: {self.label} [{self.unit}]"]
        if self.warning:
            lines += ["", "WARNING: " + self.warning]
        lines += ["", "current package:",
                 component_table(parse_components(cfg["aero"]["components"])), ""]
        if self.sweeps:
            head = f"{'scale':>7s} " + " ".join(f"{n:>14s}" for n in self.names)
            lines += ["individual sweeps - search accuracy (coarse envelope), other devices at their current scale;",
                      "* = constraint violated:", head, "-" * len(head)]
            for i, x in enumerate(self.values):
                cells = []
                for n in self.names:
                    e = self.sweeps[n][i]
                    cells.append(f"{e.value:13.3f}{' ' if e.feasible else '*'}")
                lines.append(f"{x:7.2f} " + " ".join(cells))
            lines.append("")
            fine = self.base_fine is not None and bool(self.best_fine)
            base = (self.base_fine if fine else self.base).value
            lines.append(f"current package: {base:.3f} {self.unit}" + (" (fine envelope, as below)" if fine else ""))
            for n in self.names:
                x, e = self.best_fine[n] if fine else self.best_individual(n)
                cur = float(get_path(cfg, scale_path(n)))
                lines.append(f"best {n:8s} alone: scale {x:5.2f} (now {cur:4.2f}) -> {e.value:.3f} {self.unit} "
                             f"({e.value - base:+.3f}){'' if e.feasible else '  * violates a constraint'}")
            lines.append("")
        if self.joint is not None:
            j = self.joint
            lines.append("joint optimum (all devices together, constraints applied):")
            for v in j.spec.variables:
                lines.append(f"  {v.path:32s} {j.initial[v.path]:6.3f} -> {j.best[v.path]:6.3f}")
            lines.append(f"  objective {j.initial_eval.value:.3f} -> {j.best_eval.value:.3f} {self.unit} "
                         f"({j.best_eval.value - j.initial_eval.value:+.3f}), feasible: "
                         f"{'yes' if j.best_eval.feasible else 'NO'}")
            m = j.best_eval.metrics
            if "understeer_index_min" in m:
                lines.append(f"  understeer index {m['understeer_index_min']:.3f} .. {m['understeer_index_max']:.3f}")
            lines += ["", "optimised package:", component_table(parse_components(j.best_cfg["aero"]["components"]))]
        return "\n".join(lines)


def aero_split(cfg: dict, objective_spec: dict, base_dir: str | Path = ".", constraints: dict | None = None,
               names: list[str] | None = None, bounds: tuple[float, float] = (0.0, 2.0), points: int = 9,
               joint: bool = True, max_evals: int = 80, opts=None, progress=None) -> AeroSplitResult:
    comps = (cfg.get("aero") or {}).get("components") or {}
    if not comps:
        raise ValueError("the vehicle has no aero.components (front / side / rear devices) to optimise")
    names = names or list(comps)
    for n in names:
        if n not in comps:
            raise ValueError(f"aero component {n!r} not in {list(comps)}")
    obj = Objective(objective_spec, base_dir, constraints, opts)
    values = np.linspace(bounds[0], bounds[1], points)
    res = AeroSplitResult(names, values, unit=obj.unit, label=obj.label)
    res.base = obj(cfg)
    total = len(names) * points
    k = 0
    for n in names:
        res.sweeps[n] = []
        for x in values:
            ev = obj(with_overrides(cfg, {scale_path(n): float(x)}))
            res.sweeps[n].append(ev)
            k += 1
            if progress:
                progress(f"sweep {k}/{total}: {n} = {x:.2f} -> {ev.value:.3f}")
    # the current package and each device's best point again with the fine envelope, so that these
    # numbers and the joint optimum (also re-evaluated fine) share one basis
    obj_fine = Objective(objective_spec, base_dir, constraints)
    res.base_fine = obj_fine(cfg)
    for n in names:
        x, _ = res.best_individual(n)
        cur = float(get_path(cfg, scale_path(n)))
        res.best_fine[n] = (x, res.base_fine if abs(x - cur) < 1e-9 else obj_fine(with_overrides(cfg, {scale_path(n): x})))
        if progress:
            progress(f"fine check: {n} = {x:.2f} -> {res.best_fine[n][1].value:.3f}")
    win = (constraints or {}).get("understeer_index")
    if win and not res.base_fine.feasible and not any(e.feasible for evs in res.sweeps.values() for e in evs):
        m = res.base_fine.metrics
        res.warning = (f"no device size meets the understeer-index window {win[0]:g} .. {win[1]:g}: the current car "
                       f"is at {m['understeer_index_min']:.3f} .. {m['understeer_index_max']:.3f} (lowest value at "
                       "low speed, where the balance is mechanical - springs / ARBs / weight distribution - and aero "
                       "cannot move it). Use a window that contains the low-speed value, or change the mechanical "
                       "setup first; the result below is NOT a valid optimum.")
        if progress:
            progress("WARNING: " + res.warning)
    if joint:
        spec = OptimizationSpec(vehicle_cfg=copy.deepcopy(cfg), objective_spec=objective_spec,
                                variables=[Variable(scale_path(n), bounds[0], bounds[1]) for n in names],
                                constraints=dict(constraints or {}), base_dir=Path(base_dir), max_evals=max_evals)

        def cb(n, x, ev, err):
            if progress and ev is not None:
                progress(f"joint {n}: " + ", ".join(f"{p.split('.')[-2]}={v:.2f}" for p, v in x.items())
                         + f" -> {ev.value:.3f}{'' if ev.feasible else ' (violates)'}")

        res.joint = run_optimization(spec, cb)
    return res
