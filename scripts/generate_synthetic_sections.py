"""Write SYNTHETIC AERO_AUTOMATION-style 2-D section results for testing the CFD hook-up.

    python scripts/generate_synthetic_sections.py

The files have the columns and sign conventions of ``runs/summary.csv`` of
https://github.com/luminalaxis/AERO_AUTOMATION (``cl`` = downforce coefficient, positive down,
based on the main-element chord; ``ground_clearance`` [mm] only for ground-effect runs), but the
numbers come from a simple made-up model (thin-aerofoil slope, flap camber, ground-effect gain,
ground stall, viscous stall, Re-dependent profile drag, a little noise). They are NOT CFD results.
Replace them with the real ``summary.csv`` once Fluent has run.
"""

from __future__ import annotations

import csv
import itertools
import math
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parents[1] / "data" / "aero_sections"
RHO, MU = 1.225, 1.7894e-5
RNG = np.random.default_rng(2026)
CAMBER = {"2412": 0.24, "4412": 0.47, "6412": 0.70}  # zero-angle lift of the main element (thin aerofoil)


def section(main_airfoil, main_chord, flaps, aoa, speed_kmh, h=None):
    """(cl, cd, cl_main) per main chord; flaps = [(chord_mm, deflection_deg, gap_mm), ...]."""
    c = main_chord / 1000.0
    ctot = 1.0 + sum(f[0] for f in flaps) / main_chord
    cl0 = CAMBER[main_airfoil] + sum(4.2 * (fc / main_chord) * math.radians(d) for fc, d, _ in flaps)
    slope = 0.9 * 2.0 * math.pi * ctot
    cl_lin = cl0 + slope * math.radians(aoa)
    clmax = 1.6 + 1.6 * sum(fc for fc, _, _ in flaps) / main_chord
    clmax -= sum(0.035 * max(d - (25.0 if i == 0 else 40.0), 0.0) for i, (_, d, _) in enumerate(flaps))
    clmax -= sum(0.02 * abs(g - 7.0) for _, _, g in flaps)  # slot gap away from its best value
    x = cl_lin / clmax
    knee = 0.85
    y = x if x <= knee else knee + (1 - knee) * math.tanh((x - knee) / (1 - knee))
    cl = clmax * (y - 0.7 * max(x - 1.12, 0.0))
    if h is not None:  # ground effect: suction under the inverted wing, ground stall when very close
        hc = h / main_chord
        g = 1.0 + 0.16 / math.sqrt(hc)
        g *= 1.0 - 2.5 * max(0.10 - hc, 0.0)
        g *= 1.0 - 0.6 * max(x - 0.9, 0.0) * (0.25 / hc) ** 0.5  # earlier stall close to the ground
        cl *= g
    re = RHO * speed_kmh / 3.6 * c / MU
    cd = (0.011 * ctot * (5e5 / re) ** 0.2 + 0.00025 * sum(d for _, d, _ in flaps)
          + 0.008 * cl ** 2 / ctot + 0.35 * max(x - 1.0, 0.0) ** 2)
    cl *= 1.0 + RNG.normal(0.0, 0.004)
    cd *= 1.0 + RNG.normal(0.0, 0.01)
    return cl, cd, cl * (0.55 + 0.1 / ctot)


def rows(case_set, main_airfoils, main_chord, flap_sets, aoas, speeds, heights):
    out = []
    for af, flaps, aoa, v, h in itertools.product(main_airfoils, flap_sets, aoas, speeds, heights):
        cl, cd, clm = section(af, main_chord, flaps, aoa, v, h)
        geo = f"{case_set}_{af}c{main_chord}" + "".join(f"_f{fc}d{d}g{g}" for fc, d, g in flaps)
        r = {"name": f"{geo}" + (f"_h{h}" if h is not None else "") + f"_v{v}_a{aoa}",
             "group": geo, "state": "done", "conv_state": "converged", "case_set": case_set,
             "main_airfoil": af, "main_chord": main_chord}
        for i, (fc, d, g) in enumerate(flaps, 1):
            r.update({f"flap{i}_airfoil": "4412", f"flap{i}_chord": fc, f"flap{i}_deflection": d,
                      f"flap{i}_gap": g, f"flap{i}_overlap": 5})
        if h is not None:
            r["ground_clearance"] = h
        vel = v / 3.6
        r.update({"aoa": aoa, "speed_kmh": v, "chord": main_chord / 1000.0, "velocity": round(vel, 4),
                  "reynolds": round(RHO * vel * main_chord / 1000.0 / MU), "cl": round(cl, 5), "cd": round(cd, 6),
                  "l/d": round(cl / cd, 4), "cl-main": round(clm, 5), "cl-flap1": round(cl - clm, 5),
                  "downforce_N_per_m": round(0.5 * RHO * vel ** 2 * main_chord / 1000.0 * cl, 3),
                  "drag_N_per_m": round(0.5 * RHO * vel ** 2 * main_chord / 1000.0 * cd, 4)})
        out.append(r)
    # a few cases a real batch would also have: diverged / failed
    for k, (state, conv) in zip((5, 17, 40), (("done", "diverged"), ("failed", "no_data"), ("done", "diverged"))):
        bad = dict(out[k])
        bad.update({"name": bad["name"] + "_rerun", "state": state, "conv_state": conv,
                    "cl": 37.0 if conv == "diverged" else "", "cd": 0.9 if conv == "diverged" else ""})
        out.append(bad)
    return out


def write(path: Path, data: list[dict]):
    cols: list[str] = []
    for r in data:
        cols += [k for k in r if k not in cols]
    with open(path, "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        w.writerows(data)
    print(f"written {path} ({len(data)} rows)")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    front = rows("2el", ["2412", "4412"], 175,
                 [[(fc, d, g)] for fc, d, g in itertools.product((60, 80, 100), (15, 25), (6, 10))],
                 [-4, 0, 4, 8, 12, 16], [40, 60, 80, 100], [40, 60, 80, 100])
    rear = rows("2el", ["4412", "6412"], 200, [[(80, 25, 7)], [(100, 30, 7)], [(100, 40, 7)]],
                [-4, -2, 0, 2, 4, 6, 8, 10, 12, 14, 16], [40, 60, 80, 100], [None])
    rear += rows("3el", ["4412"], 180, [[(80, 20, 7), (60, 40, 6)], [(90, 25, 7), (70, 50, 6)]],
                 [-4, -2, 0, 2, 4, 6, 8, 10, 12, 14, 16], [40, 60, 80, 100], [None])
    for r in front + rear:  # the first column says what this is
        r.update({"SYNTHETIC_NOT_CFD": 1})
    write(OUT / "synthetic_front_summary.csv", [{"SYNTHETIC_NOT_CFD": 1, **r} for r in front])
    write(OUT / "synthetic_rear_summary.csv", [{"SYNTHETIC_NOT_CFD": 1, **r} for r in rear])


if __name__ == "__main__":
    main()
