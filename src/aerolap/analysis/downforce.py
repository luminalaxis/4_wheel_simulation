"""Downforce of every aero device (front wing, side wing, rear wing, floor, ...) from the aero variables.

``F = 1/2 rho V^2 ClA`` per device, with ClA / CdA from the device model at the given attitude
(ride heights, roll, yaw), so the angles, flap deflections, gaps, spans and heights in
``aero.components`` directly set the numbers. The remainder to the full aero model (bare car /
aero map, ``cla_scale``, ``cop_shift``) is listed as its own row, so the rows add up to the total
the lap simulation uses.
"""

from __future__ import annotations

import numpy as np

from ..environment import density_from_config
from ..vehicle import VehicleModel


def device_downforce(cfg: dict, speeds_kmh, rh_f: float | None = None, rh_r: float | None = None,
                     roll_deg: float = 0.0, yaw_deg: float = 0.0) -> dict:
    """Per-device ClA, CdA and downforce / drag (N) at each speed. Positive = downforce.

    Returns ``{"rho", "rh_f", "rh_r", "speeds", "rows": [...], "total": {...}}``; each row has
    ``name``, ``design``, ``cla_f``, ``cla_r``, ``cla``, ``cda`` and arrays ``downforce`` /
    ``drag`` / ``front`` / ``rear`` (N, front / rear axle share) over the speeds.
    """
    car = VehicleModel(cfg)
    aero, geo = car.aero, car.aero.geometry
    rh_f = geo.rh_f0 if rh_f is None else float(rh_f)
    rh_r = geo.rh_r0 if rh_r is None else float(rh_r)
    v = np.asarray(speeds_kmh, dtype=float) / 3.6
    q = 0.5 * car.rho * v ** 2

    def row(name, design, cla_f, cla_r, cda):
        cla_f, cla_r, cda = (float(np.asarray(x)) for x in (cla_f, cla_r, cda))
        return {"name": name, "design": design, "cla_f": cla_f, "cla_r": cla_r, "cla": cla_f + cla_r, "cda": cda,
                "front": q * cla_f, "rear": q * cla_r, "downforce": q * (cla_f + cla_r), "drag": q * cda}

    rows = []
    for c in aero.components:
        cf, cr, cd = c.forces_at(geo, rh_f, rh_r, roll_deg, yaw_deg)
        sc = aero.cla_scale
        rows.append(row(c.name, _design(c), sc * cf, sc * cr, aero.cda_scale * cd))
    mode = car.default_mode if hasattr(car, "default_mode") else aero.default_mode
    tf, tr, td, _ = aero.coefficients(mode, rh_f, rh_r, roll_deg, yaw_deg)
    total = row("TOTAL", "", tf, tr, td)
    rest = row("body / map / trim", "", total["cla_f"] - sum(r["cla_f"] for r in rows),
               total["cla_r"] - sum(r["cla_r"] for r in rows), total["cda"] - sum(r["cda"] for r in rows))
    return {"rho": car.rho, "rh_f": rh_f, "rh_r": rh_r, "roll": roll_deg, "yaw": yaw_deg,
            "speeds": np.asarray(speeds_kmh, dtype=float), "rows": rows + [rest], "total": total}


def _design(c) -> str:
    out = []
    for suffix, _key, _rng, unit, val in c.design_variables():
        out.append(f"{suffix.lstrip('.') or 'main'}={val:g}{'°' if unit == 'deg' else ' ' + unit}")
    return " ".join(out)


def downforce_table(res: dict) -> str:
    sp = res["speeds"]
    lines = [f"air density {res['rho']:.3f} kg/m^3, ride height F/R {res['rh_f']:.1f}/{res['rh_r']:.1f} mm, "
             f"roll {res['roll']:.1f} deg, yaw {res['yaw']:.1f} deg",
             "downforce = 1/2 rho V^2 ClA  (N, positive = down)", ""]
    head = f"{'device':18s} {'variables':34s} {'ClA':>7s} {'CdA':>7s} {'front%':>7s}"
    lines.append(head + "".join(f"{v:>9.0f}" for v in sp) + "   [km/h]")
    for r in res["rows"] + [res["total"]]:
        share = 100.0 * r["cla_f"] / r["cla"] if abs(r["cla"]) > 1e-9 else float("nan")
        lines.append(f"{r['name']:18s} {r['design'][:34]:34s} {r['cla']:7.3f} {r['cda']:7.3f} {share:7.1f}"
                     + "".join(f"{x:9.1f}" for x in r["downforce"]))
    t = res["total"]
    lines += ["", f"{'axle split (N)':18s}"]
    lines.append(f"{'  front axle':53s}" + "".join(f"{x:9.1f}" for x in t["front"]))
    lines.append(f"{'  rear axle':53s}" + "".join(f"{x:9.1f}" for x in t["rear"]))
    lines.append(f"{'  drag':53s}" + "".join(f"{x:9.1f}" for x in t["drag"]))
    return "\n".join(lines)
