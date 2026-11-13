"""Next AERO_AUTOMATION batch around a wing the optimiser chose (``aerolap cfd-plan``).

The loop between the two tools::

    aerolap auto-aero car.yaml track.csv            -> optimized_vehicle.yaml (element model)
    aerolap cfd-plan optimized_vehicle.yaml --component front --out cfd_front
        -> cfd_front/project.toml + cases.csv        (copy into an AERO_AUTOMATION project folder)
    aero-auto screen / mesh / run / collect          -> runs/summary.csv (Fluent)
    wing:  section_data: runs/summary.csv
           section_fit: true                         (element model corrected with the CFD)
    aerolap auto-aero ... again                      -> moves over the CFD data; repeat cfd-plan
                                                        around the new optimum until it stops moving

The geometries are the wing's current element values (the centre) and one step up and down of
each element value the optimiser may move (chords +-10 %, flap angles +-5 deg, gaps / overlaps
+-2 mm), one at a time - the points the CFD correction needs to know the local slopes. Rows use
the ``summary.csv`` column names of AERO_AUTOMATION, so ``section_fit`` reads the results back
without any mapping. Conditions: ``aoa`` around the wing's angle, ``ground_clearance`` around its
height (ground-effect wings only) and ``speed_kmh`` over the lap's speed range.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..aero.maps import parse_components

STEPS = {"chord": 0.10, "deflection": 5.0, "gap": 2.0, "overlap": 2.0}  # chord relative, others absolute


@dataclass
class CfdPlan:
    component: str
    ground: bool
    rows: list[dict]
    aoa: list[float]
    clearance: list[float]
    speeds: list[float]

    @property
    def n_cases(self) -> int:
        return len(self.rows) * len(self.aoa) * max(len(self.clearance), 1) * len(self.speeds)

    def project_toml(self) -> str:
        n_el = {r["n_el"] for r in self.rows}
        lines = [
            f"# aerolap cfd-plan: next AERO_AUTOMATION batch around the {self.component} wing chosen by aerolap",
            f"# {len(self.rows)} geometries x {len(self.aoa)} aoa"
            + (f" x {len(self.clearance)} ground clearances" if self.ground else " (free stream)")
            + f" x {len(self.speeds)} speeds = {self.n_cases} cases",
            "# Check against the AERO_AUTOMATION README before running (journal, base cases, mesh settings).",
            "# After 'aero-auto collect': in the aerolap vehicle file set on this wing",
            "#   section_data: <this folder>/runs/summary.csv",
            "#   section_fit: true",
            "",
            "[project]", f'name = "aerolap_{self.component}"',
            f'template = "{"journal.jou" if self.ground else "journal_freestream.jou"}"',
            'output_dir = "runs"', "max_parallel = 1", "",
            "[solver]", 'kind = "fluent"', 'mode = "2ddp"', "processes = 4", "graphics = true",
            "timeout_minutes = 60", "",
            "[constants]", "downforce = true", "iterations = 2000"]
        if self.ground:
            lines.append('ground_ref = "lowest"')
        name = "{geom}" + ("_h{ground_clearance}" if self.ground else "") + "_v{speed_kmh}_a{aoa}"
        lines += ["", "[cases]", 'csv = "cases.csv"', f'name = "{name}"', 'group = "{geom}"', "",
                  "[cases.sweep]", f"aoa = {_lst(self.aoa)}"]
        if self.ground:
            lines.append(f"ground_clearance = {_lst(self.clearance)}")
        lines += [f"speed_kmh = {_lst(self.speeds)}", "",
                  "[derived]", 'chord = "main_chord / 1000"', 'velocity = "speed_kmh / 3.6"',
                  'mach = "velocity / 340.29"', 'reynolds = "1.225 * velocity * chord / 1.7894e-5"', "",
                  "[mesh]", "y_plus = 1.0", "growth_rate = 1.15", "surface_cells = 200", "flap_surface_cells = 150"]
        if not self.ground:
            lines.append('farfield_shape = "circle"')
        lines += ["", "[screening]", 'transition = "free"',
                  'protect = ["*gap", "*overlap", "*chord", "*deflection", "ground_clearance"]',
                  "prune_aoa = false              # aerolap already chose these geometries: screen only the angles",
                  "", "[results]", 'report_files = ["*.out"]', "average_last = 100", 'x_param = "aoa"',
                  "wind_axes = true", 'objective = "max_cl"', 'split_by = ["speed_kmh"]', ""]
        if len(n_el) > 1:
            lines.insert(3, "# NOTE: geometries with different element counts - base_case is set per row in cases.csv")
        return "\n".join(lines)

    def write(self, out: str | Path) -> list[Path]:
        out = Path(out)
        out.mkdir(parents=True, exist_ok=True)
        cols = list(dict.fromkeys(k for r in self.rows for k in r if k != "n_el"))
        with open(out / "cases.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            for r in self.rows:
                w.writerow({k: r.get(k, "") for k in cols})
        (out / "project.toml").write_text(self.project_toml(), encoding="utf-8")
        return [out / "project.toml", out / "cases.csv"]

    def report(self) -> str:
        head = (f"{self.component} wing: {len(self.rows)} geometries, aoa {_lst(self.aoa)}"
                + (f", ground clearance {_lst(self.clearance)} mm" if self.ground else ", free stream")
                + f", speeds {_lst(self.speeds)} km/h -> {self.n_cases} cases (before AERO_AUTOMATION screening)")
        return head + "\n" + "\n".join(f"  {r['geom']}" for r in self.rows)


def _lst(xs) -> str:
    return "[" + ", ".join(f"{x:g}" for x in xs) + "]"


def _fmt(x: float) -> float | int:
    x = round(float(x), 1)
    return int(x) if x == int(x) else x


def cfd_plan(cfg: dict, component: str, free: list[str] | None = None, aoa_span: float = 4.0, aoa_step: float = 2.0,
             speeds: list[float] | None = None, steps: dict | None = None) -> CfdPlan:
    """Geometries around the wing ``component`` of ``cfg``; ``free``: element value names to vary
    (``flap1.deflection``, ``main.chord`` ...; default all)."""
    comps = {c.name: getattr(c, "dev", c) for c in parse_components((cfg.get("aero") or {}).get("components"),
                                                                     cfg.get("_base_dir", "."))}
    dev = comps.get(component)
    model = getattr(dev, "model", None)
    if model is None:
        raise ValueError(f"aero.components.{component} must be an enabled 'type: wing' with 'elements' "
                         f"(have {', '.join(comps) or 'no enabled devices'})")
    steps = {**STEPS, **(steps or {})}
    base = model.params()

    def row(params: dict) -> dict:
        r = {k: (_fmt(v) if isinstance(v, (int, float)) else v) for k, v in params.items() if v not in (None, "")}
        for k in ("main_airfoil",) + tuple(f"flap{i}_airfoil" for i in range(1, len(model.elements))):
            r.setdefault(k, "4412")  # AERO_AUTOMATION needs a section: the model's camber / thickness default
        n = len(model.elements)
        parts = [f"{n}el", f"{r['main_airfoil']}c{r['main_chord']:g}"]
        parts += [f"f{r[f'flap{i}_chord']:g}d{r[f'flap{i}_deflection']:g}g{r[f'flap{i}_gap']:g}o{r[f'flap{i}_overlap']:g}"
                  for i in range(1, n)]
        return {"geom": "_".join(parts), "base_case": f"base_{n}el{'' if ground else '_freestream'}.cas.h5", **r, "n_el": n}

    ground = bool(dev.ground_effect)
    rows = [row(base)]
    for key, val in base.items():
        kind = key.split("_", 1)[-1]
        if kind not in steps or not isinstance(val, (int, float)):
            continue
        name = key.replace("_", ".", 1)
        if free and not any(name == f or name.startswith(f + ".") or key == f for f in free):
            continue
        d = steps[kind] * val if kind == "chord" else steps[kind]
        for sgn in (-1.0, 1.0):
            new = val + sgn * d
            if kind in ("gap",) and new <= 0.5 or kind == "deflection" and not 0.0 <= new <= 80.0:
                continue
            r = row({**base, key: new})
            if r["geom"] not in {x["geom"] for x in rows}:
                rows.append(r)
    a0 = round(dev.aoa / aoa_step) * aoa_step
    aoa = [float(x) for x in np.arange(a0 - aoa_span, a0 + aoa_span + 1e-9, aoa_step)]
    h = float(dev.height)
    clearance = sorted({max(_fmt(h + d), 10) for d in (-20.0, 0.0, 20.0)}) if ground else []
    return CfdPlan(component, ground, rows, aoa, [float(x) for x in clearance], list(speeds or [40.0, 70.0, 100.0]))
