"""No aero vs current aero vs optimised aero: the same car through the FSAE dynamic events.

Three versions of the car:

* **no aero** - every downforce device switched off (wings, floor, simple components:
  ``enabled: false``); the sidepods stay (the radiators are needed) but lose their own downforce.
  A car without components (constant / table aero map) gets ``cla_scale: 0`` - its drag stays, as
  the map does not say how much of it the wings make. The mass of the removed package comes off the
  car: the components' ``mass_kg``, or ``aero_mass_kg`` for the whole package when given.
* **current** - the car as it is.
* **optimised** - a given optimised vehicle (e.g. ``optimized_vehicle.yaml`` from ``aero-split`` /
  ``auto-aero``), or the result of a device optimisation run by the caller.

Each one runs acceleration, skidpad, autocross (the given track, standing start) and the
energy-limited endurance, so the comparison includes the battery: drag costs energy, and a car
whose endurance does not fit the battery is slowed by the power cap.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field

import numpy as np

from ..aero.devices import _enabled
from ..config import set_path
from ..features import features_for_level
from ..solvers.ggv import SolverOptions
from ..track import Track
from ..vehicle import VehicleModel
from .events import EventResult, run_events

DOWNFORCE_TYPES = ("wing", "floor", "")  # "" = simple cla x scale component


def no_aero_cfg(cfg: dict, aero_mass_kg: float | None = None) -> tuple[dict, list[str]]:
    """The car without its downforce devices; returns (config, notes)."""
    out = copy.deepcopy(cfg)
    aero = out.setdefault("aero", {})
    comps = aero.get("components") or {}
    notes = []
    removed = []
    for name, c in comps.items():
        if not isinstance(c, dict) or not _enabled(c):
            continue
        kind = str(c.get("type", "")).lower()
        if kind in DOWNFORCE_TYPES:
            c["enabled"] = False
            removed.append(name)
            if aero_mass_kg is not None:
                c.pop("mass_kg", None)  # the package mass below replaces the per-device masses
        elif kind == "sidepod" and c.get("cla"):
            c["cla"] = 0.0
            notes.append(f"{name}: 라디에이터·냉각 항력은 유지, 다운포스(cla)만 0")
    if removed:
        notes.insert(0, "꺼진 장치: " + ", ".join(removed))
    base_cla = aero.get("type", "constant") != "constant" or float(aero.get("cla", 0.0) or 0.0) != 0.0
    if base_cla:
        aero["cla_scale"] = 0.0
        notes.append("공력 맵/상수 ClA → 0 (cla_scale 0). 맵의 항력은 그대로 - 윙 항력이 맵에 들어 있으면 "
                     "에어로 없음의 항력이 실제보다 큼")
    if not removed and not base_cla:
        notes.append("다운포스 장치가 없어 현재 차와 같음")
    if aero_mass_kg:
        set_path(out, "mass.total_kg", float(out["mass"]["total_kg"]) - float(aero_mass_kg))
        notes.append(f"에어로 패키지 무게 {float(aero_mass_kg):g} kg 제외")
    elif not any(float((comps.get(n) or {}).get("mass_kg") or 0.0) for n in removed):
        notes.append("장치 무게(mass_kg) 정보가 없어 질량은 그대로 - 에어로 패키지 무게를 넣으면 가벼워진 차로 계산")
    return out, notes


@dataclass
class Variant:
    name: str
    cfg: dict
    events: list[EventResult]
    cla: float
    cda: float
    front_share: float
    mass: float
    notes: list[str] = field(default_factory=list)

    def time(self, event: str) -> float:
        return next(e.time_s for e in self.events if e.name == event)

    @property
    def endurance(self):
        return next((e for e in self.events if e.name == "endurance"), None)


@dataclass
class AeroCompareResult:
    variants: list[Variant]
    distance_km: float

    def table(self) -> str:
        vs = self.variants
        base = vs[0]
        w = 20
        head = f"{'':24s}" + "".join(f"{v.name:>{w}s}" for v in vs)
        lines = ["에어로 없음 / 현재 / 최적화 비교  (괄호: 에어로 없음 대비)", "", head, "-" * len(head)]

        def row(label, fn, fmt, delta=True, pct=False):
            cells = []
            for v in vs:
                x = fn(v)
                txt = format(x, fmt)
                if delta and v is not base:
                    d = x - fn(base)
                    txt += f" ({d / fn(base) * 100:+.1f}%)" if pct else f" ({d:+{fmt}})"
                cells.append(f"{txt:>{w}s}")
            lines.append(f"{label:24s}" + "".join(cells))

        row("질량 [kg]", lambda v: v.mass, ".1f")
        row("ClA [m²]", lambda v: v.cla, ".3f")
        row("CdA [m²]", lambda v: v.cda, ".3f")
        row("앞 분담 [%]", lambda v: 100 * v.front_share, ".1f", delta=False)
        row("L/D", lambda v: v.cla / v.cda if v.cda > 0 else float("nan"), ".2f", delta=False)
        lines.append("")
        for ev in base.events:
            row(f"{ev.name} [s]", lambda v, n=ev.name: v.time(n), ".3f", pct=True)
        e = base.endurance
        if e is not None:
            row("내구 에너지 [kWh]", lambda v: v.endurance.energy_kWh, ".2f")
            row("내구 1랩 [s]", lambda v: v.endurance.energy.lap_time_s if v.endurance.energy else float("nan"),
                ".3f", pct=True)
            lines.append(f"{'내구 출력 제한 [kW]':24s}" + "".join(
                f"{(f'{v.endurance.energy.power_cap_kW:.1f}' if v.endurance.energy and v.endurance.energy.power_cap_kW else '없음'):>{w}s}"
                for v in vs))
        lines.append("")
        if len(vs) > 2:
            cur, opt = vs[1], vs[2]
            lines.append("최적화 효과 (현재 대비): " + ", ".join(
                f"{ev.name} {opt.time(ev.name) - cur.time(ev.name):+.3f} s" for ev in base.events))
        for v in vs:
            for n in v.notes:
                lines.append(f"[{v.name}] {n}")
        return "\n".join(lines)


def _variant(name, cfg, track, endu, distance_km, level, opts, notes=()):
    car = VehicleModel(cfg, features_for_level(level))
    cf, cr, cd, _ = car.static_aero()
    cf, cr, cd = (float(np.asarray(x)) for x in (cf, cr, cd))
    cla = cf + cr
    events = run_events(car, track, endu, distance_km, opts)
    return Variant(name, cfg, events, cla, cd, cf / cla if abs(cla) > 1e-9 else float("nan"), car.m, list(notes))


def aero_compare(cfg: dict, track_path: str, optimized_cfg: dict | None = None, level: str = "full",
                 distance_km: float = 22.0, opts: SolverOptions | None = None, aero_mass_kg: float | None = None,
                 log=print, optimized_notes=()) -> AeroCompareResult:
    """Run the events for no aero / current / optimised (when given) and compare."""
    opts = opts or SolverOptions()
    track = Track.from_file(track_path)
    endu = Track.from_file(track_path, closed=True)
    bare, notes = no_aero_cfg(cfg, aero_mass_kg)
    todo = [("에어로 없음", bare, notes), ("현재", cfg, [])]
    if optimized_cfg is not None:
        todo.append(("최적화", optimized_cfg, list(optimized_notes)))
    out = []
    for i, (name, c, n) in enumerate(todo):
        log(f"[{i + 1}/{len(todo)}] {name}: 가속·스키드패드·오토크로스·내구 계산 중 ...")
        out.append(_variant(name, c, track, endu, distance_km, level, opts, n))
    return AeroCompareResult(out, distance_km)


def optimize_devices(cfg: dict, track_path: str, objective: str = "endurance_time", level: str = "full",
                     distance_km: float = 22.0, understeer=(1.0, 1.3), max_evals: int = 80, progress=None):
    """Joint optimisation of every aero design variable (no per-device sweeps): the "optimised" car.

    Same search as the joint part of ``aero-split`` (Powell on the coarse envelope, the best candidates
    re-checked with the fine one). Returns the :class:`OptimizationResult` (``best_cfg``)."""
    from pathlib import Path

    from .aero_split import design_variables
    from .optimize import OptimizationSpec, Variable, run_optimization

    dv = design_variables(cfg)
    if not dv:
        raise ValueError("the vehicle has no aero.components to optimise")
    spec_obj = {"type": objective, "level": level, "track": str(Path(track_path).resolve())}
    if objective == "endurance_time":
        spec_obj["distance_km"] = distance_km
    constraints = {"understeer_index": list(understeer)} if understeer else {}
    spec = OptimizationSpec(vehicle_cfg=copy.deepcopy(cfg), objective_spec=spec_obj,
                            variables=[Variable(path, rng[0], rng[1]) for path, rng, _u in dv.values()],
                            constraints=constraints, base_dir=Path(cfg.get("_base_dir", ".")), max_evals=max_evals)
    short = {path: n for n, (path, _r, _u) in dv.items()}

    def cb(n, x, ev, err):
        if progress and ev is not None:
            progress(f"최적화 {n}/{max_evals}: " + ", ".join(f"{short.get(p, p)}={v:.3g}" for p, v in x.items())
                     + f" -> {ev.value:.3f}{'' if ev.feasible else ' (제약 위반)'}")

    return run_optimization(spec, cb)
