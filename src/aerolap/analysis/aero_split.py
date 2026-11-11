"""Aero package split: best setting of each device (front wing, side wing / sidepods, rear wing).

Design variables (see :func:`design_variables`):

* a wing (``type: wing``): the main-element angle ``aoa_deg`` (name ``front``), and with per-element
  geometry also each flap's angle to the main chord (names ``front.flap1``, ``front.flap2`` ...),
* a sidepod (``type: sidepod``): the radiator exit area ``exit_area_m2`` (cooling vs drag),
* a floor or a simple component: its ``scale``.

Two studies:

* **individual** - sweep one variable with all others at their current value: shows how lap
  time (or energy-limited endurance time) depends on each one on its own;
* **joint** - optimise all variables together (Powell, coarse envelope, best candidates re-checked with
  the fine envelope) under the usual setup constraints, e.g. an understeer-index window so the
  aero balance stays stable. The devices interact through the balance (front wing vs rear wing),
  so the joint optimum is the one to build; the individual curves explain it.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..aero.devices import geometry_from_cfg
from ..aero.maps import component_table, parse_components
from ..config import get_path, with_overrides
from .objectives import Evaluation, Objective
from .optimize import OptimizationResult, OptimizationSpec, Variable, run_optimization


def scale_path(name: str) -> str:
    return f"aero.components.{name}.scale"


UNIT_TEXT = {"deg": "angle", "m2": "area", "x": "scale"}


def design_variables(cfg: dict) -> dict[str, tuple[str, tuple[float, float], str]]:
    """name -> (config path, default range, unit) of every design variable of every device.

    Names: the device name for its main variable (wing main aoa, sidepod exit area, scale), and
    ``<device>.<flap>`` for a flap angle of a wing with per-element geometry."""
    out = {}
    for comp in parse_components((cfg.get("aero") or {}).get("components"), cfg.get("_base_dir", ".")):
        for suffix, key, rng, unit, _val in comp.design_variables():
            out[comp.name + suffix] = (f"aero.components.{comp.name}.{key}", tuple(rng), unit)
    return out


def resolve_names(cfg: dict, names: list[str] | None) -> list[str]:
    """Variable names from device names (all their variables) or variable names (``front.flap1``)."""
    dv = design_variables(cfg)
    if not names:
        return list(dv)
    out = []
    for n in names:
        n = n.strip()
        hits = [v for v in dv if v == n or v.split(".")[0] == n]  # a device brings all its variables
        if not hits:
            raise ValueError(f"aero design variable {n!r} not found; have {', '.join(dv)}")
        out += [h for h in hits if h not in out]
    return out


def _table(cfg: dict) -> str:
    return component_table(parse_components(cfg["aero"]["components"], cfg.get("_base_dir", ".")),
                           geometry_from_cfg(cfg))


@dataclass
class AeroSplitResult:
    names: list[str]
    values: dict[str, np.ndarray]  # sweep grid of each device's design variable
    paths: dict[str, str] = field(default_factory=dict)  # design-variable config path per device
    units: dict[str, str] = field(default_factory=dict)
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
        return float(self.values[name][i]), evs[i]

    def report(self, cfg: dict) -> str:
        lines = [f"objective: {self.label} [{self.unit}]"]
        if self.warning:
            lines += ["", "WARNING: " + self.warning]
        lines += ["", "current package:", _table(cfg), ""]
        if self.sweeps:
            lines += ["individual sweeps - search accuracy (coarse envelope), other devices at their current value;",
                      "* = constraint violated:"]
            for n in self.names:
                u = self.units.get(n, "x")
                cells = [f"{_fmt(x, u)}: {e.value:.3f}{'' if e.feasible else '*'}"
                         for x, e in zip(self.values[n], self.sweeps[n])]
                lines.append(f"  {n:14s} " + "  ".join(cells))
            lines.append("")
            fine = self.base_fine is not None and bool(self.best_fine)
            base = (self.base_fine if fine else self.base).value
            lines.append(f"current package: {base:.3f} {self.unit}" + (" (fine envelope, as below)" if fine else ""))
            for n in self.names:
                x, e = self.best_fine[n] if fine else self.best_individual(n)
                cur = float(get_path(cfg, self.paths[n]))
                u = self.units.get(n, "x")
                lines.append(f"best {n:14s} alone: {UNIT_TEXT.get(u, u)} {_fmt(x, u)} (now {_fmt(cur, u)}) -> "
                             f"{e.value:.3f} {self.unit} "
                             f"({e.value - base:+.3f}){'' if e.feasible else '  * violates a constraint'}")
            lines.append("")
        if self.joint is not None:
            j = self.joint
            lines.append("joint optimum (all devices together, constraints applied):")
            for v in j.spec.variables:
                lines.append(f"  {v.path:50s} {j.initial[v.path]:7.3f} -> {j.best[v.path]:7.3f}")
            lines.append(f"  objective {j.initial_eval.value:.3f} -> {j.best_eval.value:.3f} {self.unit} "
                         f"({j.best_eval.value - j.initial_eval.value:+.3f}), feasible: "
                         f"{'yes' if j.best_eval.feasible else 'NO'}")
            m = j.best_eval.metrics
            if "understeer_index_min" in m:
                lines.append(f"  understeer index {m['understeer_index_min']:.3f} .. {m['understeer_index_max']:.3f}")
            lines += ["", "optimised package:", _table(j.best_cfg)]
        return "\n".join(lines)


def _fmt(x: float, unit: str) -> str:
    return f"{x:.1f}°" if unit == "deg" else (f"{x:.3f} m²" if unit == "m2" else f"{x:.2f}")


def aero_split(cfg: dict, objective_spec: dict, base_dir: str | Path = ".", constraints: dict | None = None,
               names: list[str] | None = None, bounds: tuple[float, float] | None = None, points: int = 9,
               joint: bool = True, max_evals: int = 80, opts=None, progress=None) -> AeroSplitResult:
    """``names``: devices (all their variables) or single variables (``front.flap1``); default all.
    ``bounds`` overrides the range of scale-type variables (angles and areas keep their ranges)."""
    comps = (cfg.get("aero") or {}).get("components") or {}
    if not comps:
        raise ValueError("the vehicle has no aero.components (front / side / rear devices) to optimise")
    names = resolve_names(cfg, names)
    dv = design_variables(cfg)
    ranges = {n: (bounds if bounds is not None and dv[n][2] == "x" else dv[n][1]) for n in names}
    obj = Objective(objective_spec, base_dir, constraints, opts)
    res = AeroSplitResult(names, {n: np.linspace(ranges[n][0], ranges[n][1], points) for n in names},
                          {n: dv[n][0] for n in names}, {n: dv[n][2] for n in names}, unit=obj.unit, label=obj.label)
    res.base = obj(cfg)
    total = len(names) * points
    k = 0
    for n in names:
        res.sweeps[n] = []
        for x in res.values[n]:
            ev = obj(with_overrides(cfg, {res.paths[n]: float(x)}, f"공력 장치 개별 스윕 ({n})"))
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
        cur = float(get_path(cfg, res.paths[n]))
        res.best_fine[n] = (x, res.base_fine if abs(x - cur) < 1e-9 else obj_fine(with_overrides(cfg, {res.paths[n]: x},
                                                                                      f"공력 장치 정밀 확인 ({n})")))
        if progress:
            progress(f"fine check: {n} = {x:.2f} -> {res.best_fine[n][1].value:.3f}")
    win = (constraints or {}).get("understeer_index")
    if win and not res.base_fine.feasible and not any(e.feasible for evs in res.sweeps.values() for e in evs):
        m = res.base_fine.metrics
        res.warning = (f"no device setting meets the understeer-index window {win[0]:g} .. {win[1]:g}: the current "
                       f"car is at {m['understeer_index_min']:.3f} .. {m['understeer_index_max']:.3f} (lowest value at "
                       "low speed, where the balance is mechanical - springs / ARBs / weight distribution - and aero "
                       "cannot move it). Use a window that contains the low-speed value, or change the mechanical "
                       "setup first; the result below is NOT a valid optimum.")
        if progress:
            progress("WARNING: " + res.warning)
    if joint:
        spec = OptimizationSpec(vehicle_cfg=copy.deepcopy(cfg), objective_spec=objective_spec,
                                variables=[Variable(res.paths[n], ranges[n][0], ranges[n][1]) for n in names],
                                constraints=dict(constraints or {}), base_dir=Path(base_dir), max_evals=max_evals)

        def cb(n, x, ev, err):
            if progress and ev is not None:
                short = {res.paths[k]: k for k in names}
                progress(f"joint {n}: " + ", ".join(f"{short.get(p, p)}={v:.3g}" for p, v in x.items())
                         + f" -> {ev.value:.3f}{'' if ev.feasible else ' (violates)'}")

        res.joint = run_optimization(spec, cb)
    return res
