"""Aero package split: best setting of each device (front wing, side wing / sidepods, rear wing).

Design variables (see :func:`design_variables`):

* a wing (``type: wing``): the main-element angle ``aoa_deg`` (name ``front``), and with per-element
  geometry also each flap's angle to the main chord (names ``front.flap1``, ``front.flap2`` ...),
* a sidepod (``type: sidepod``): the radiator exit area ``exit_area_m2`` (cooling vs drag),
* a floor or a simple component: its ``scale``.

With ``detail=True`` (CLI ``--detail``) also the geometry of each device (see
:meth:`aerolap.aero.devices.Wing.detail_variables`): wing span (``front.span``), height in ground
effect (``front.height``), position (``front.x``), each element's chord (``front.main.chord``,
``front.flap1.chord``) and each flap's slot (``front.flap1.gap``, ``front.flap1.overlap``), the
sidepod's radiator core area (``sidepod.core_area``). Default ranges are a band around the current
value; set the real limits (rules box, packaging) with ``detail_ranges`` in the device or with
``ranges`` here - a variable that ends on a bound is marked in the report.

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
import fnmatch
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


UNIT_TEXT = {"deg": "angle", "m2": "area", "x": "scale", "m": "length", "mm": "length"}

# axis / table text per variable kind (last part of the name)
KIND_TEXT = {"span": "span", "height": "height above ground", "x": "position behind front axle",
             "chord": "chord", "gap": "slot gap", "overlap": "slot overlap", "core_area": "radiator core area"}


def design_variables(cfg: dict, detail: bool = False) -> dict[str, tuple[str, tuple[float, float], str]]:
    """name -> (config path, default range, unit) of every design variable of every device.

    Names: the device name for its main variable (wing main aoa, sidepod exit area, scale), and
    ``<device>.<flap>`` for a flap angle of a wing with per-element geometry; with ``detail`` also
    the geometry (``front.span``, ``front.flap1.gap``, ``sidepod.core_area`` ...)."""
    out = {}
    for comp in parse_components((cfg.get("aero") or {}).get("components"), cfg.get("_base_dir", ".")):
        for suffix, key, rng, unit, _val in comp.design_variables(detail):
            out[comp.name + suffix] = (f"aero.components.{comp.name}.{key}", tuple(rng), unit)
    return out


def resolve_names(cfg: dict, names: list[str] | None, detail: bool = False) -> list[str]:
    """Variable names from device names (all their variables), variable names (``front.flap1``,
    ``front.flap1.gap``) or patterns (``front.*.gap``, ``rear.flap2.*``).

    A device name brings its angle / area variables, with ``detail`` also its geometry; a detail
    variable named on its own (or by a pattern) is always used."""
    dv = design_variables(cfg, True)
    basic = set(design_variables(cfg, False))
    if not names:
        return [v for v in dv if detail or v in basic]
    out = []
    for n in names:
        n = n.strip()
        if not n:
            continue
        if any(ch in n for ch in "*?["):
            hits = [v for v in dv if fnmatch.fnmatchcase(v, n)]
        else:
            hits = [v for v in dv if v == n or (v.split(".")[0] == n and (detail or v in basic))]
        if not hits:
            raise ValueError(f"aero design variable {n!r} not found; have {', '.join(dv)}")
        out += [h for h in hits if h not in out]
    return out


def pin_ground_effect(cfg: dict, names: list[str]) -> tuple[dict, list[str]]:
    """Copy of ``cfg`` with ``ground_effect`` written into every wing whose geometry is optimised and
    whose file leaves it to the automatic choice (height < chord). A chord or height change would
    otherwise switch ground effect on or off - a jump in lift the optimiser exploits and a saved
    optimum that no longer matches the evaluated one. Returns (config, ["side (off)", ...])."""
    geometry = {n.split(".")[0] for n in names if n not in design_variables(cfg, False)}
    if not geometry:
        return cfg, []
    out = copy.deepcopy(cfg)
    raw = out["aero"]["components"]
    pinned = []
    for comp in parse_components(raw, cfg.get("_base_dir", ".")):
        dev = getattr(comp, "dev", None)
        if (comp.name in geometry and hasattr(dev, "ground_effect") and hasattr(dev, "span") and not dev.cfd
                and raw[comp.name].get("ground_effect") is None):
            raw[comp.name]["ground_effect"] = bool(dev.ground_effect)
            pinned.append(f"{comp.name} ({'on' if dev.ground_effect else 'off'})")
    return out, pinned


def describe(name: str, unit: str) -> str:
    """Axis text of a design variable, e.g. 'front.flap1.gap' -> 'slot gap [mm]'."""
    parts = name.split(".")
    if unit == "deg":
        return "flap angle to main chord [deg]" if len(parts) > 1 else "main aoa [deg]"
    if len(parts) == 1:
        return "exit area [m^2]" if unit == "m2" else "scale [-]"
    return f"{KIND_TEXT.get(parts[-1], parts[-1])} [{'m^2' if unit == 'm2' else unit}]"


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
    ranges: dict[str, tuple[float, float]] = field(default_factory=dict)  # search range per variable
    notes: list[str] = field(default_factory=list)
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
        lines += ["note: " + n for n in self.notes]
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
                lines.append(f"  {n:20s} " + "  ".join(cells))
            lines.append("")
            fine = self.base_fine is not None and bool(self.best_fine)
            base = (self.base_fine if fine else self.base).value
            lines.append(f"current package: {base:.3f} {self.unit}" + (" (fine envelope, as below)" if fine else ""))
            for n in self.names:
                x, e = self.best_fine[n] if fine else self.best_individual(n)
                cur = float(get_path(cfg, self.paths[n]))
                u = self.units.get(n, "x")
                lines.append(f"best {n:20s} alone: {UNIT_TEXT.get(u, u)} {_fmt(x, u)} (now {_fmt(cur, u)}) -> "
                             f"{e.value:.3f} {self.unit} "
                             f"({e.value - base:+.3f}){'' if e.feasible else '  * violates a constraint'}")
            lines.append("")
        if self.joint is not None:
            j = self.joint
            lines.append("joint optimum (all devices together, constraints applied; "
                         "<< / >> = on the lower / upper bound - widen it if the rules allow):")
            short = {p: n for n, p in self.paths.items()}
            at_bound = []
            for v in j.spec.variables:
                n = short.get(v.path, v.path)
                u = self.units.get(n, "x")
                b = j.best[v.path]
                tol = 0.01 * (v.hi - v.lo)
                mark = " <<" if b <= v.lo + tol else (" >>" if b >= v.hi - tol else "")
                if mark:
                    at_bound.append(n)
                lines.append(f"  {n:22s} {_fmt(j.initial[v.path], u):>10s} -> {_fmt(b, u):>10s}   "
                             f"[{_fmt(v.lo, u)} .. {_fmt(v.hi, u)}]{mark}")
            lines.append(f"  objective {j.initial_eval.value:.3f} -> {j.best_eval.value:.3f} {self.unit} "
                         f"({j.best_eval.value - j.initial_eval.value:+.3f}), feasible: "
                         f"{'yes' if j.best_eval.feasible else 'NO'}")
            m = j.best_eval.metrics
            if "understeer_index_min" in m:
                lines.append(f"  understeer index {m['understeer_index_min']:.3f} .. {m['understeer_index_max']:.3f}")
            mi = j.initial_eval.metrics
            if "wing_stall_ratio_max" in m:
                lines.append(f"  most loaded wing element {mi.get('wing_stall_ratio_max', float('nan')):.0%} -> "
                             f"{m['wing_stall_ratio_max']:.0%} of its maximum lift")
            if "wing_min_clearance_mm" in m:
                lines.append(f"  lowest ground-effect wing point {mi.get('wing_min_clearance_mm', float('nan')):.1f} -> "
                             f"{m['wing_min_clearance_mm']:.1f} mm (lap, braking / cornering limits)")
            if at_bound:
                lines.append(f"  on a bound: {', '.join(at_bound)} - the optimum lies at or beyond the range; "
                             "check the limit (rules, packaging, structure) before building it")
            lines += ["", "optimised package:", _table(j.best_cfg)]
        return "\n".join(lines)


def _fmt(x: float, unit: str) -> str:
    if unit == "deg":
        return f"{x:.1f}°"
    if unit == "m2":
        return f"{x:.3f} m²" if x >= 0.01 else f"{x:.4f} m²"
    if unit == "mm":
        return f"{x:.1f} mm"
    if unit == "m":
        return f"{x:.3f} m"
    return f"{x:.2f}"


def auto_max_evals(n_vars: int) -> int:
    """Joint-optimisation budget when none is given: Powell needs ~10-15 evaluations per variable and sweep."""
    return max(80, 15 * n_vars)


def aero_split(cfg: dict, objective_spec: dict, base_dir: str | Path = ".", constraints: dict | None = None,
               names: list[str] | None = None, bounds: tuple[float, float] | None = None, points: int = 9,
               joint: bool = True, max_evals: int | None = None, opts=None, progress=None, detail: bool = False,
               ranges: dict[str, tuple[float, float]] | None = None) -> AeroSplitResult:
    """``names``: devices (all their variables), single variables (``front.flap1``, ``front.flap1.gap``)
    or patterns (``*.gap``); default all. ``detail`` adds each device's geometry (span, chords, gaps,
    overlaps, height, position, radiator core). ``bounds`` overrides the range of scale-type variables,
    ``ranges`` (name -> (lo, hi)) that of any variable. ``points`` = 0 skips the individual sweeps.
    ``max_evals`` None -> :func:`auto_max_evals`."""
    comps = (cfg.get("aero") or {}).get("components") or {}
    if not comps:
        raise ValueError("the vehicle has no aero.components (front / side / rear devices) to optimise")
    names = resolve_names(cfg, names, detail)
    dv = design_variables(cfg, True)
    rng_over = dict(ranges or {})
    unknown = [k for k in rng_over if k not in names]
    if unknown:
        raise ValueError(f"range given for {', '.join(unknown)}, which is not among the optimised variables "
                         f"({', '.join(names)}); name it in the components or use detail (--detail) for geometry")
    for k, (lo, hi) in rng_over.items():
        if not hi > lo:
            raise ValueError(f"range of {k}: upper bound {hi:g} must exceed lower bound {lo:g}")
    ranges = {n: tuple(rng_over[n]) if n in rng_over else (bounds if bounds is not None and dv[n][2] == "x" else dv[n][1])
              for n in names}
    if max_evals is None:
        max_evals = auto_max_evals(len(names))
    obj = Objective(objective_spec, base_dir, constraints, opts)
    points = max(int(points), 0)
    res = AeroSplitResult(names, {n: np.linspace(ranges[n][0], ranges[n][1], points) for n in names},
                          {n: dv[n][0] for n in names}, {n: dv[n][2] for n in names}, unit=obj.unit, label=obj.label)
    res.ranges = ranges
    cfg, pinned = pin_ground_effect(cfg, names)
    if pinned:
        res.notes.append("ground effect kept as at the start while the geometry changes (written to the optimised "
                         "file as ground_effect): " + ", ".join(pinned))
    if progress:
        for note in res.notes:
            progress("note: " + note)
        progress(f"{len(names)} design variables: {', '.join(names)}"
                 + (f"; joint optimisation up to {max_evals} evaluations" if joint else ""))
    res.base = obj(cfg)
    total = len(names) * points
    k = 0
    for n in names if points else []:
        res.sweeps[n] = []
        for x in res.values[n]:
            ev = obj(with_overrides(cfg, {res.paths[n]: float(x)}, f"공력 장치 개별 스윕 ({n})"))
            res.sweeps[n].append(ev)
            k += 1
            if progress:
                progress(f"sweep {k}/{total}: {n} = {x:.4g} -> {ev.value:.3f}")
    # the current package and each device's best point again with the fine envelope, so that these
    # numbers and the joint optimum (also re-evaluated fine) share one basis
    obj_fine = Objective(objective_spec, base_dir, constraints)
    res.base_fine = obj_fine(cfg) if points else None
    for n in names if points else []:
        x, _ = res.best_individual(n)
        cur = float(get_path(cfg, res.paths[n]))
        res.best_fine[n] = (x, res.base_fine if abs(x - cur) < 1e-9 else obj_fine(with_overrides(cfg, {res.paths[n]: x},
                                                                                      f"공력 장치 정밀 확인 ({n})")))
        if progress:
            progress(f"fine check: {n} = {x:.4g} -> {res.best_fine[n][1].value:.3f}")
    win = (constraints or {}).get("understeer_index")
    if win and points and not res.base_fine.feasible and not any(e.feasible for evs in res.sweeps.values() for e in evs):
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
