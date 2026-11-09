"""Export of envelopes and lap traces for other tools (MATLAB / Excel / MoTeC-style CSV).

``export_ggv`` writes the g-g-V envelope of each aero mode as

* ``ggv.csv`` - long format, one row per (mode, speed, lateral fraction), SI units and g
  (header row only, so MATLAB ``readtable`` / Excel read it directly);
* ``ggv.mat`` - MATLAB struct ``ggv.<mode>`` with fields ``v`` [m/s] (nv x 1), ``frac`` (1 x nf),
  ``ay_max`` (nv x 1) and the ``ay``, ``ax_acc``, ``ax_brk``, ``ax_coast`` tables (nv x nf, m/s^2),
  plus ``meta`` (vehicle, model level, units). Load it in MATLAB with ``S = load('ggv.mat')``.

The lateral axis of the tables is ``frac = ay / ay_max(v)`` sampled as ``sin(theta)`` (dense near the
cornering limit), so ``ay = frac .* ay_max``. ``matlab/aerolap_ggv_demo.m`` shows how to use it.

The envelope is a snapshot of one setup: the suspension / aero coupling is already inside it, but a
setup change (springs, ride height, wing) needs a new export.
"""

from __future__ import annotations

import csv
import re
from pathlib import Path

import numpy as np

G0 = 9.80665


def _field(name: str) -> str:
    """Valid MATLAB identifier from a mode name."""
    f = re.sub(r"\W", "_", str(name))
    if not f or not f[0].isalpha():
        f = "m_" + f
    return f[:63]


def ggv_arrays(ggv) -> dict[str, np.ndarray]:
    v = np.asarray(ggv.v, dtype=float)
    frac = np.asarray(ggv.fracs, dtype=float)
    ay_max = np.asarray(ggv.ay_max, dtype=float)
    return {
        "v": v[:, None],
        "frac": frac[None, :],
        "ay_max": ay_max[:, None],
        "ay": ay_max[:, None] * frac[None, :],
        "ax_acc": np.asarray(ggv.ax_acc, dtype=float),
        "ax_brk": np.asarray(ggv.ax_brk, dtype=float),
        "ax_coast": np.asarray(ggv.ax_coast, dtype=float),
    }


def export_ggv(ggvs, mode_names, out_dir: str | Path, meta: dict | None = None,
               formats=("csv", "mat")) -> list[Path]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    if "csv" in formats:
        path = out_dir / "ggv.csv"
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)  # no comment lines: plain readtable / Excel import
            w.writerow(["mode", "v_mps", "v_kmh", "lat_frac", "ay_mps2", "ay_g", "ax_acc_mps2", "ax_acc_g",
                        "ax_brk_mps2", "ax_brk_g", "ax_coast_mps2"])
            for g in ggvs:
                a = ggv_arrays(g)
                name = mode_names[g.mode]
                for i, vi in enumerate(a["v"][:, 0]):
                    for j, fj in enumerate(a["frac"][0]):
                        ay, acc, brk = a["ay"][i, j], a["ax_acc"][i, j], a["ax_brk"][i, j]
                        w.writerow([name, f"{vi:.4f}", f"{vi * 3.6:.3f}", f"{fj:.6f}", f"{ay:.5f}",
                                    f"{ay / G0:.5f}", f"{acc:.5f}", f"{acc / G0:.5f}", f"{brk:.5f}",
                                    f"{brk / G0:.5f}", f"{a['ax_coast'][i, j]:.5f}"])
        written.append(path)
    if "mat" in formats:
        from scipy.io import savemat

        data = {"ggv": {_field(mode_names[g.mode]): ggv_arrays(g) for g in ggvs}}
        m = {str(k): str(v) for k, v in (meta or {}).items()}
        m["units"] = "v m/s, accelerations m/s^2 (positive ax = accelerating, ax_brk < 0), frac = ay/ay_max"
        m["modes"] = ", ".join(_field(mode_names[g.mode]) for g in ggvs)
        data["meta"] = m
        path = out_dir / "ggv.mat"
        savemat(path, data, oned_as="column")
        written.append(path)
    return written


def lap_to_mat(res, path: str | Path) -> Path:
    """Lap trace as a MATLAB struct ``lap`` (one column per channel, SI units)."""
    from scipy.io import savemat

    lap = {"s": res.s, "t": res.t, "v": res.v, "ax": res.ax, "ay": res.ay, "kappa": res.kappa,
           "aero_mode": np.array([res.mode_names[m] for m in res.mode], dtype=object)}
    for k, v in res.channels.items():
        arr = np.asarray(v)
        if arr.dtype.kind in "fiub":
            lap[_field(k)] = arr.astype(float)
        else:
            lap[_field(k)] = arr.astype(str).astype(object)
    meta = {"track": res.track_name, "vehicle": res.vehicle_name, "model": res.level, "solver": res.solver,
            "lap_time_s": res.lap_time, "energy_kWh": res.energy_J / 3.6e6}
    savemat(path, {"lap": lap, "meta": meta}, oned_as="column")
    return Path(path)
