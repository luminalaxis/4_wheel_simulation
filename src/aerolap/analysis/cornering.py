"""Steady-state limit analyses: cornering limit vs speed and straight-line braking limit.

These show how the balance moves from *mechanical* (low speed: weight distribution,
roll-stiffness distribution) to *aerodynamic* (high speed: CoP, ride-height / roll /
yaw sensitivity of the aero map), and give the attitude (ride heights, roll, yaw,
floor clearance) the optimiser constrains.
"""

from __future__ import annotations

import numpy as np

from ..solvers.ggv import SolverOptions, _bisect, ay_max_curve
from ..vehicle import G, LIMIT_NAMES, VehicleModel, VehicleState


def state_channels(st: VehicleState) -> dict[str, np.ndarray]:
    fo, fi, ro, ri = st.fz
    return {
        "rh_front_mm": st.rh_f * 1000.0,
        "rh_rear_mm": st.rh_r * 1000.0,
        "clearance_mm": st.clearance * 1000.0,
        "roll_deg": np.degrees(st.roll),
        "pitch_deg": np.degrees(st.pitch),
        "yaw_deg": np.degrees(st.yaw),
        "steer_deg": np.degrees(st.steer),
        "understeer_deg": np.degrees(st.alpha_f - st.alpha_r),
        "cla": st.cla,
        "cda": st.cda,
        "cop_front": st.cop_front,
        "downforce_N": st.df_f + st.df_r,
        "fz_front_outer_N": fo, "fz_front_inner_N": fi, "fz_rear_outer_N": ro, "fz_rear_inner_N": ri,
        "understeer_index": st.understeer_index,
        "u_front": st.u_front, "u_rear": st.u_rear,
    }


def cornering_sweep(vehicle: VehicleModel, speeds, mode: int | None = None,
                    opts: SolverOptions | None = None) -> dict[str, np.ndarray]:
    """Steady-state cornering limit (ax = 0) at each speed and the attitude at that limit."""
    opts = opts or SolverOptions()
    mode = vehicle.default_mode if mode is None else mode
    v = np.asarray(speeds, dtype=float)
    ay = ay_max_curve(vehicle, mode, v, opts)
    st = vehicle.solve_state(v, 0.0, ay * 0.999, mode, max_iter=opts.eq_max_iter, tol=opts.eq_tol)
    out = {"v_mps": v, "v_kmh": v * 3.6, "ay_max_mps2": ay, "ay_max_g": ay / G,
           "radius_m": np.where(ay > 0, v * v / np.where(ay > 0, ay, 1.0), np.inf)}
    out.update(state_channels(st))
    out["limit"] = np.array([LIMIT_NAMES[i] for i in st.limit], dtype=object)
    return out


def braking_sweep(vehicle: VehicleModel, speeds, mode: int | None = None,
                  opts: SolverOptions | None = None) -> dict[str, np.ndarray]:
    """Straight-line braking limit at each speed (worst case for pitch / front ride height)."""
    opts = opts or SolverOptions()
    mode = vehicle.default_mode if mode is None else mode
    v = np.asarray(speeds, dtype=float)
    kw = dict(max_iter=opts.eq_max_iter, tol=opts.eq_tol)
    st0 = vehicle.solve_state(v, 0.0, 0.0, mode, **kw)
    coast = vehicle.coast_ax(st0)

    def fn(ax, init):
        st = vehicle.solve_state(v, ax, 0.0, mode, init=init, **kw)
        return st.feasible, st

    ax = _bisect(fn, coast, coast - opts.ax_upper, opts.bisect_iters)
    st = vehicle.solve_state(v, ax, 0.0, mode, **kw)
    out = {"v_mps": v, "v_kmh": v * 3.6, "ax_min_mps2": ax, "ax_min_g": ax / G}
    out.update(state_channels(st))
    return out


def format_table(data: dict[str, np.ndarray], columns: list[tuple[str, str, str]]) -> str:
    """Plain-text table. ``columns`` = (key, header, format)."""
    head = "  ".join(f"{h:>10s}" for _, h, _ in columns)
    lines = [head, "-" * len(head)]
    n = len(next(iter(data.values())))
    for i in range(n):
        cells = []
        for key, _, fmt in columns:
            val = data[key][i]
            cells.append(f"{val:>10s}" if isinstance(val, str) else f"{val:>10{fmt}}")
        lines.append("  ".join(cells))
    return "\n".join(lines)


CORNERING_COLUMNS = [
    ("v_kmh", "v[km/h]", ".1f"), ("ay_max_g", "ay[g]", ".3f"), ("radius_m", "R[m]", ".1f"),
    ("limit", "limit", "s"), ("understeer_index", "US index", ".3f"),
    ("rh_front_mm", "rh_f[mm]", ".1f"), ("rh_rear_mm", "rh_r[mm]", ".1f"),
    ("roll_deg", "roll[deg]", ".2f"), ("yaw_deg", "yaw[deg]", ".2f"), ("cla", "ClA", ".3f"),
    ("cop_front", "CoP_f", ".3f"), ("clearance_mm", "clear[mm]", ".1f"),
]

BRAKING_COLUMNS = [
    ("v_kmh", "v[km/h]", ".1f"), ("ax_min_g", "ax[g]", ".3f"), ("rh_front_mm", "rh_f[mm]", ".1f"),
    ("rh_rear_mm", "rh_r[mm]", ".1f"), ("pitch_deg", "pitch[deg]", ".2f"), ("cla", "ClA", ".3f"),
    ("cop_front", "CoP_f", ".3f"), ("clearance_mm", "clear[mm]", ".1f"),
]
