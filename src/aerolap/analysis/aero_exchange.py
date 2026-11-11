"""Aero exchange rates: what a step of downforce, drag, balance, mass or a device change is worth.

Central differences of the objective (one lap or the battery-limited endurance) for

* ClA +0.1 m^2 and CdA +0.1 m^2 (via ``aero.cla_scale`` / ``aero.cda_scale``, balance kept),
* aero balance +1 % to the front (``aero.cop_shift``), mass +1 kg,
* each device's design variable (+1 deg of a wing, +0.1 of a floor / simple component).

The exchange rate "ClA 0.1 = CdA x" is the extra drag that a downforce gain may cost before it
stops paying; "ClA 0.1 = y kg" compares aero with mass reduction.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .aero_split import design_variables
from .objectives import Objective
from .sweep import sensitivities


@dataclass
class ExchangeItem:
    label: str
    path: str
    step: float  # change of the config value
    per: str  # unit of the reported change ("ClA +0.1", "+1 kg" ...)
    delta: float  # objective change for that step


@dataclass
class AeroExchange:
    label: str
    unit: str
    base: float
    items: list[ExchangeItem] = field(default_factory=list)

    def get(self, label: str) -> ExchangeItem | None:
        return next((i for i in self.items if i.label == label), None)

    def report(self) -> str:
        lines = [f"objective: {self.label} [{self.unit}], current {self.base:.3f}", "",
                 f"{'change':38s} {'objective change':>18s}"]
        lines.append("-" * 58)
        for it in self.items:
            lines.append(f"{it.per:38s} {it.delta:+14.4f} {self.unit}")
        cla, cda, mass = self.get("ClA"), self.get("CdA"), self.get("mass")
        lines.append("")
        if cla and cda and cda.delta > 0 and cla.delta < 0:
            lines.append(f"exchange rate: ClA +0.1 is worth CdA +{0.1 * -cla.delta / cda.delta:.3f} "
                         f"(L/D of the extra downforce must be better than {cda.delta / -cla.delta:.2f} to pay)")
        elif cla and cda:
            lines.append("exchange rate: not defined (downforce does not help or drag does not cost here)")
        if cla and mass and mass.delta > 0 and cla.delta < 0:
            lines.append(f"                ClA +0.1 is worth {-cla.delta / mass.delta:.1f} kg less mass")
        return "\n".join(lines)


def aero_exchange(cfg: dict, objective: Objective, progress=None) -> AeroExchange:
    from ..vehicle import VehicleModel

    car = VehicleModel(cfg)
    g = car.aero.geometry
    cf, cr, cd, _ = car.aero.coefficients(car.default_mode, g.rh_f0, g.rh_r0, 0.0, 0.0)
    cla, cda = float(cf + cr), float(cd)
    items = [("ClA", "aero.cla_scale", 0.1 / max(cla, 1e-6), "ClA +0.1 m^2 (balance kept)"),
             ("CdA", "aero.cda_scale", 0.1 / max(cda, 1e-6), "CdA +0.1 m^2"),
             ("balance", "aero.cop_shift", 0.01, "aero balance +1 % to the front"),
             ("mass", "mass.total_kg", 1.0, "mass +1 kg")]
    for name, (path, _rng, unit) in design_variables(cfg).items():
        step, what = {"deg": (1.0, "angle +1 deg"), "m2": (0.005, "area +0.005 m^2")}.get(unit, (0.1, "scale +0.1"))
        items.append((name, path, step, f"{name}: {what}"))
    paths = [p for _, p, _, _ in items]
    steps = {p: s for _, p, s, _ in items}
    # cla_scale / cda_scale / cop_shift may be absent from the file: defaults 1, 1, 0
    base_cfg = dict(cfg)
    aero = dict(cfg["aero"])
    aero.setdefault("cla_scale", 1.0)
    aero.setdefault("cda_scale", 1.0)
    aero.setdefault("cop_shift", 0.0)
    base_cfg["aero"] = aero
    if progress:
        progress(f"exchange rates: {len(paths)} parameters x 2 evaluations ...")
    base, rows = sensitivities(base_cfg, paths, objective, abs_steps=steps)
    out = AeroExchange(objective.label, objective.unit, base.value)
    for (label, path, step, per), r in zip(items, rows):
        out.items.append(ExchangeItem(label, path, step, per, r.d_objective))
    if progress:
        progress(out.report())
    return out

