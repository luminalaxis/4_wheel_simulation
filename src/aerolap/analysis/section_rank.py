"""Rank 2-D CFD wing sections (AERO_AUTOMATION ``summary.csv``) by lap / endurance time.

AERO_AUTOMATION ranks its geometries by the section's maximum Cl or maximum L/D. What the car
needs depends on the event: on one lap downforce is cheap, in a battery-limited endurance drag is
expensive (see ``aero-exchange``). This puts every section of the file on the chosen wing of the
vehicle (span, position, height, end plates of the vehicle), finds its best angle of attack under
the balance constraints and ranks the sections by the objective - next to the section-only ranks.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import minimize_scalar

from ..aero.section_data import load_sections
from ..config import with_overrides
from ..solvers.ggv import SolverOptions
from .objectives import Evaluation, Objective


@dataclass
class SectionScore:
    geometry_id: str
    params: dict
    aoa: float
    evaluation: Evaluation
    cla: float
    cda: float
    max_cl_2d: float
    max_ld_2d: float
    fine: Evaluation | None = None

    @property
    def value(self) -> float:  # search accuracy - the same basis for every section
        return self.evaluation.value

    @property
    def feasible(self) -> bool:
        return self.evaluation.feasible


@dataclass
class SectionRanking:
    component: str
    source: str
    label: str
    unit: str
    current: Evaluation  # fine envelope
    current_search: Evaluation  # search accuracy (same basis as the ranking)
    scores: list[SectionScore] = field(default_factory=list)
    synthetic: bool = False
    best_cfg: dict | None = None

    def ordered(self) -> list[SectionScore]:
        return sorted(self.scores, key=lambda s: (not s.feasible, s.value))

    def report(self) -> str:
        rows = self.ordered()
        by_cl = {s.geometry_id: i + 1 for i, s in enumerate(sorted(self.scores, key=lambda s: -s.max_cl_2d))}
        by_ld = {s.geometry_id: i + 1 for i, s in enumerate(sorted(self.scores, key=lambda s: -s.max_ld_2d))}
        lines = [f"{self.component} wing sections from {Path(self.source).name}"
                 + ("  [SYNTHETIC DATA - not CFD]" if self.synthetic else ""),
                 f"objective: {self.label} [{self.unit}], current car {self.current_search.value:.3f} "
                 f"(search accuracy; fine {self.current.value:.3f})", "",
                 f"{'rank':>4s} {'objective':>10s} {'vs now':>8s} {'aoa':>6s} {'ClA':>6s} {'CdA':>6s} {'L/D':>6s} "
                 f"{'2-D maxCl':>9s} {'#':>3s} {'2-D maxL/D':>10s} {'#':>3s} ok  section"]
        for i, s in enumerate(rows, 1):
            lines.append(f"{i:4d} {s.value:10.3f} {s.value - self.current_search.value:+8.3f} {s.aoa:6.2f} {s.cla:6.3f} "
                         f"{s.cda:6.3f} {s.cla / max(s.cda, 1e-9):6.2f} {s.max_cl_2d:9.3f} {by_cl[s.geometry_id]:3d} "
                         f"{s.max_ld_2d:10.2f} {by_ld[s.geometry_id]:3d} {'yes' if s.feasible else 'NO ':3s} "
                         f"{s.geometry_id}")
        if rows:
            best = rows[0]
            lines += ["", "top sections re-checked with the fine envelope (same basis as 'current car fine'):"]
            for s in rows:
                if s.fine is not None:
                    lines.append(f"  {s.fine.value:10.3f} ({s.fine.value - self.current.value:+.3f})"
                                 f"{'' if s.fine.feasible else ' violates'}  {s.geometry_id} @ {s.aoa:.2f} deg")
            lines += ["", f"best for this car: {best.geometry_id} at {best.aoa:.2f} deg",
                      f"its rank by section max Cl: {by_cl[best.geometry_id]}, by section max L/D: "
                      f"{by_ld[best.geometry_id]} (of {len(rows)})"]
        lines.append("2-D maxCl / maxL/D: section values at the wing's static clearance (front) and the reference speed")
        return "\n".join(lines)


_EMPIRICAL = ("chord_m", "cl0", "cl_max", "cd0", "k_ground", "h_stall_mm", "ground_effect", "range", "elements")


def _section_entry(comp: dict) -> dict:
    """The wing's own keys (span, position, height, end plates ...) without the empirical-section ones."""
    return {k: v for k, v in comp.items() if k not in _EMPIRICAL}


def rank_sections(cfg: dict, summary_csv: str | Path, component: str, objective_spec: dict,
                  constraints: dict | None = None, base_dir: str | Path = ".", opts: SolverOptions | None = None,
                  max_geometries: int | None = None, refine_top: int = 3, progress=None) -> SectionRanking:
    from ..aero.devices import geometry_from_cfg
    from ..vehicle import VehicleModel

    comp = ((cfg.get("aero") or {}).get("components") or {}).get(component)
    if not comp or str(comp.get("type", "")).lower() != "wing":
        raise ValueError(f"aero.components.{component} must be a 'type: wing' device to take section data")
    if "span_m" not in comp:
        raise ValueError(f"aero.components.{component} needs span_m")
    sset = load_sections(Path(summary_csv).resolve())
    opts = opts or SolverOptions.coarse()
    obj = Objective(objective_spec, base_dir, constraints, opts)
    obj_fine = Objective(objective_spec, base_dir, constraints)
    out = SectionRanking(component, str(summary_csv), obj.label, obj.unit, obj_fine(cfg), obj(cfg),
                         synthetic=sset.synthetic)
    sections = list(sset.sections.values())[:max_geometries] if max_geometries else list(sset.sections.values())
    geo = geometry_from_cfg(cfg)
    path = f"aero.components.{component}"
    for k, sec in enumerate(sections, 1):
        new = _section_entry(comp)
        new.update({"section_data": str(Path(summary_csv).resolve()), "section": sec.geometry_id})
        base = with_overrides(cfg, {path: new}, f"CFD 단면 {sec.geometry_id}를 {component} 윙에 적용")
        cache: dict[float, Evaluation] = {}

        def f(a, base=base):
            a = round(float(a), 3)
            if a not in cache:
                cache[a] = obj(with_overrides(base, {f"{path}.aoa_deg": a}, f"CFD 단면 받음각 탐색 ({a:g} deg)"))
            ev = cache[a]
            return ev.total

        for a in sec.aoa:  # the CFD grid first, then refine around the best point
            f(a)
        a0 = min(cache, key=lambda a: cache[a].total)
        i = int(np.argmin(np.abs(sec.aoa - a0)))
        bracket = (float(sec.aoa[max(i - 1, 0)]), float(sec.aoa[min(i + 1, len(sec.aoa) - 1)]))
        if bracket[1] > bracket[0]:
            minimize_scalar(f, bounds=bracket, method="bounded", options={"xatol": 0.25, "maxiter": 6})
        best_a = min(cache, key=lambda a: (not cache[a].feasible, cache[a].total))
        car = VehicleModel(with_overrides(base, {f"{path}.aoa_deg": best_a}, "CFD 단면 최적 받음각"))
        dev = next(c for c in car.aero.components if c.name == component).dev
        cla, cda = (float(np.asarray(x)) for x in dev.coefficients(geo, geo.rh_f0, geo.rh_r0, 0.0, 0.0))
        h = dev.height if sec.has_ground else None
        out.scores.append(SectionScore(sec.geometry_id, sec.params, best_a, cache[best_a], cla, cda,
                                       sec.max_cl(h, dev.section_speed)[1], sec.max_ld(h, dev.section_speed)[1]))
        if progress:
            ev = cache[best_a]
            progress(f"[{k}/{len(sections)}] {sec.geometry_id}: aoa {best_a:.2f} -> {ev.value:.3f}"
                     f"{'' if ev.feasible else ' (violates)'}")
    for s in out.ordered()[:refine_top]:  # same basis as the current car: fine envelope
        new = _section_entry(comp)
        new.update({"section_data": str(Path(summary_csv).resolve()), "section": s.geometry_id, "aoa_deg": s.aoa})
        s.fine = obj_fine(with_overrides(cfg, {path: new}, "CFD 단면 정밀 재평가"))
    best = out.ordered()[0] if out.scores else None
    if best is not None:
        new = _section_entry(comp)
        new.update({"section_data": str(Path(summary_csv).resolve()), "section": best.geometry_id,
                    "aoa_deg": round(best.aoa, 2)})
        out.best_cfg = with_overrides(copy.deepcopy(cfg), {path: new}, "CFD 단면 1위 적용")
    return out
