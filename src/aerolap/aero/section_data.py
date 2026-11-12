"""2-D section results of AERO_AUTOMATION (``runs/summary.csv``) as wing polars.

https://github.com/luminalaxis/AERO_AUTOMATION runs ANSYS Fluent 2-D on multi-element downforce
wings and collects one row per case: case variables (``main_airfoil``, ``main_chord`` [mm],
``flapN_chord``, ``flapN_deflection``, ``flapN_gap`` ..., ``ground_clearance`` [mm] for ground-
effect runs - absent for free-stream runs such as a rear wing), the conditions ``aoa`` [deg] and
``speed_kmh``, and the results ``cl`` (downforce coefficient, positive down, main-chord based) and
``cd``. This module

* reads the file (utf-8-sig), drops failed / diverged / non-numeric rows,
* finds the variables that distinguish geometries (the same rule as AERO_AUTOMATION's
  ``geometry_keys``: case variables with more than one value, conditions and derived values excluded),
* builds one :class:`SectionPolar` per geometry: ``cl``, ``cd`` on the (aoa [, ground_clearance]
  [, speed_kmh]) grid with linear interpolation; outside the data the nearest value is used and
  flagged.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import numpy as np

CONDITIONS = ("aoa", "ground_clearance", "speed_kmh")
NOT_GEOMETRY = {"name", "group", "geom", "state", "conv_state", "conv_reason", "aoa", "speed_kmh",
                "ground_clearance", "chord", "velocity", "reynolds", "mach", "density", "flow_angle", "downforce",
                "base_case", "iterations", "ground_ref", "SYNTHETIC_NOT_CFD", "avg_iters", "yplus-max", "yplus-avg"}
BAD_CONV = {"diverged", "no_data"}


def _num(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return x
    return v if math.isfinite(v) else None


def _is_result(key: str) -> bool:
    k = key.lower()
    return (k in ("cl", "cd", "l/d", "cm") or k.startswith(("cl-", "cd-", "cx", "cy", "res_", "conv_", "l/d"))
            or k.endswith(("_n_per_m", "_s", "_time")))


@dataclass
class SectionPolar:
    geometry_id: str
    params: dict  # geometry variables of this section
    aoa: np.ndarray
    heights: np.ndarray | None  # mm, None = free stream
    speeds: np.ndarray | None  # km/h
    cl: np.ndarray  # shape (n_aoa[, n_h][, n_v])
    cd: np.ndarray
    c_ref: float  # m, main chord
    source: str = ""
    n_cases: int = 0
    synthetic: bool = False
    _interp: dict = field(default_factory=dict, repr=False)

    @property
    def has_ground(self) -> bool:
        return self.heights is not None

    def _axes(self):
        ax = [self.aoa]
        if self.heights is not None:
            ax.append(self.heights)
        if self.speeds is not None:
            ax.append(self.speeds)
        return ax

    def _points(self, aoa, h=None, v=None):
        pts = [np.asarray(aoa, dtype=float)]
        if self.heights is not None:
            pts.append(np.asarray(self.heights.mean() if h is None else h, dtype=float))
        if self.speeds is not None:
            pts.append(np.asarray(np.median(self.speeds) if v is None else v, dtype=float))
        b = np.broadcast_arrays(*pts)
        return np.stack(b, axis=-1)

    def _eval(self, which: str, aoa, h=None, v=None):
        from scipy.interpolate import RegularGridInterpolator

        axes = self._axes()
        if which not in self._interp:
            vals = self.cl if which == "cl" else self.cd
            if len(axes) == 1:
                self._interp[which] = None
            else:
                self._interp[which] = RegularGridInterpolator(axes, vals, bounds_error=False, fill_value=None)
        pts = self._points(aoa, h, v)
        lo = np.array([a[0] for a in axes])
        hi = np.array([a[-1] for a in axes])
        clipped = np.clip(pts, lo, hi)
        if self._interp[which] is None:
            vals = self.cl if which == "cl" else self.cd
            out = np.interp(clipped[..., 0], axes[0], vals)
        else:
            out = self._interp[which](clipped.reshape(-1, clipped.shape[-1]))
        return np.asarray(out).reshape(pts.shape[:-1])

    def cl_at(self, aoa, h=None, v=None):
        if v is not None or self.speeds is None:
            return self._fast("cl", aoa, h, v)
        return self._eval("cl", aoa, h, v)

    def cd_at(self, aoa, h=None, v=None):
        if v is not None or self.speeds is None:
            return self._fast("cd", aoa, h, v)
        return self._eval("cd", aoa, h, v)

    def _slice(self, v):
        """(cl, cd) on the (aoa[, h]) grid at speed v (cached): the lap solvers read one speed."""
        key = ("slice", None if v is None else round(float(v), 6))
        if key not in self._interp:
            if self.speeds is None:
                self._interp[key] = (self.cl, self.cd)
            else:
                vv = float(np.clip(v, self.speeds[0], self.speeds[-1]))
                j = int(np.clip(np.searchsorted(self.speeds, vv) - 1, 0, len(self.speeds) - 2)) if len(self.speeds) > 1 else 0
                if len(self.speeds) == 1:
                    self._interp[key] = (self.cl[..., 0], self.cd[..., 0])
                else:
                    t = (vv - self.speeds[j]) / (self.speeds[j + 1] - self.speeds[j])
                    self._interp[key] = tuple(a[..., j] * (1 - t) + a[..., j + 1] * t for a in (self.cl, self.cd))
        return self._interp[key]

    def _fast(self, which: str, aoa, h=None, v=None):
        """Bilinear (aoa, h) / linear (aoa) lookup on the speed slice, clamped to the data."""
        cl2, cd2 = self._slice(v)
        tab = cl2 if which == "cl" else cd2
        a = np.clip(np.asarray(aoa, dtype=float), self.aoa[0], self.aoa[-1])
        if self.heights is None:
            return np.interp(a, self.aoa, tab)
        hh = np.clip(np.asarray(self.heights.mean() if h is None else h, dtype=float), self.heights[0], self.heights[-1])
        a, hh = np.broadcast_arrays(a, hh)
        i = np.clip(np.searchsorted(self.aoa, a) - 1, 0, len(self.aoa) - 2)
        j = np.clip(np.searchsorted(self.heights, hh) - 1, 0, len(self.heights) - 2)
        ta = (a - self.aoa[i]) / (self.aoa[i + 1] - self.aoa[i])
        th = (hh - self.heights[j]) / (self.heights[j + 1] - self.heights[j])
        return ((1 - ta) * (1 - th) * tab[i, j] + ta * (1 - th) * tab[i + 1, j]
                + (1 - ta) * th * tab[i, j + 1] + ta * th * tab[i + 1, j + 1])

    def out_of_range(self, aoa, h=None, v=None):
        pts = self._points(aoa, h, v)
        axes = self._axes()
        lo = np.array([a[0] for a in axes]) - 1e-9
        hi = np.array([a[-1] for a in axes]) + 1e-9
        return np.any((pts < lo) | (pts > hi), axis=-1)

    def max_cl(self, h=None, v=None) -> tuple[float, float]:
        """(aoa of the maximum cl, maximum cl) at a ground clearance / speed (data aoa grid, refined)."""
        a = np.linspace(self.aoa[0], self.aoa[-1], 4 * len(self.aoa))
        c = self.cl_at(a, h, v)
        i = int(np.argmax(c))
        return float(a[i]), float(c[i])

    def max_ld(self, h=None, v=None) -> tuple[float, float]:
        a = np.linspace(self.aoa[0], self.aoa[-1], 4 * len(self.aoa))
        ld = self.cl_at(a, h, v) / np.maximum(self.cd_at(a, h, v), 1e-6)
        i = int(np.argmax(ld))
        return float(a[i]), float(ld[i])

    def describe(self) -> str:
        cond = [f"aoa {self.aoa[0]:g}..{self.aoa[-1]:g} deg ({len(self.aoa)})"]
        if self.heights is not None:
            cond.append(f"ground clearance {self.heights[0]:g}..{self.heights[-1]:g} mm ({len(self.heights)})")
        else:
            cond.append("free stream")
        if self.speeds is not None:
            cond.append(f"{self.speeds[0]:g}..{self.speeds[-1]:g} km/h ({len(self.speeds)})")
        return (f"{self.geometry_id}: main chord {self.c_ref * 1000:.0f} mm, {self.n_cases} cases, "
                + ", ".join(cond) + (" [SYNTHETIC]" if self.synthetic else ""))


@dataclass
class SectionSet:
    path: Path
    keys: list[str]  # geometry variables
    sections: dict[str, SectionPolar]
    dropped: dict[str, int]
    synthetic: bool = False
    constants: dict = field(default_factory=dict)  # geometry columns with one value in the whole file

    def full_params(self, sec: "SectionPolar") -> dict:
        """All geometry columns of a section (the varying ones and the file's constants)."""
        return {**self.constants, **sec.params}

    def select(self, where: dict | str | None) -> SectionPolar:
        """One section: by geometry id, or by a dict of geometry variables (must leave exactly one)."""
        if isinstance(where, str):
            if where not in self.sections:
                raise ValueError(f"section {where!r} not in {self.path.name} (have {', '.join(list(self.sections)[:6])} ...)")
            return self.sections[where]
        where = where or {}
        unknown = [k for k in where if k not in self.keys]
        if unknown:
            raise ValueError(f"section filter {unknown} not among the geometry variables {self.keys} of {self.path.name}")
        hits = [s for s in self.sections.values() if all(_same(s.params.get(k), v) for k, v in where.items())]
        if len(hits) != 1:
            ids = ", ".join(h.geometry_id for h in hits[:6])
            raise ValueError(f"section filter {where} matches {len(hits)} geometries in {self.path.name}"
                             + (f" ({ids} ...)" if hits else "") + "; add variables until exactly one is left. "
                             f"Geometry variables: {', '.join(self.keys)}")
        return hits[0]


def _same(a, b) -> bool:
    try:
        return abs(float(a) - float(b)) < 1e-9
    except (TypeError, ValueError):
        return str(a) == str(b)


def _gid(params: dict, keys: list[str]) -> str:
    """Compact geometry id in AERO_AUTOMATION's naming style, e.g. ``3el_4412c180_f80d20_f60d40g6``."""
    def fmt(v):
        return f"{v:g}" if isinstance(v, float) else str(v)

    p = {k: params[k] for k in keys if params.get(k) not in (None, "")}
    parts, used = [], set()
    if "case_set" in p:
        parts.append(fmt(p["case_set"]))
        used.add("case_set")
    main = ""
    for k, pre in (("main_airfoil", ""), ("main_chord", "c")):
        if k in p:
            main += pre + fmt(p[k])
            used.add(k)
    if main:
        parts.append(main)
    n = 1
    while any(k.startswith(f"flap{n}_") for k in p):
        f = ""
        for suf, pre in (("airfoil", "a"), ("chord", "f"), ("deflection", "d"), ("gap", "g"), ("overlap", "o")):
            k = f"flap{n}_{suf}"
            if k in p:
                f += pre + fmt(p[k])
                used.add(k)
        parts.append(f)
        n += 1
    parts += [f"{k}{fmt(v)}" for k, v in p.items() if k not in used]
    return "_".join(parts) or "section"


@lru_cache(maxsize=16)
def _load_cached(path: str, mtime: float) -> SectionSet:
    p = Path(path)
    with open(p, newline="", encoding="utf-8-sig") as fh:
        raw = list(csv.DictReader(fh))
    if not raw:
        raise ValueError(f"{p}: no rows")
    cols = list(raw[0])
    for need in ("aoa", "cl", "cd"):
        if need not in cols:
            raise ValueError(f"{p}: column {need!r} missing - is this an AERO_AUTOMATION runs/summary.csv?")
    dropped = {"not done": 0, "diverged / no data": 0, "no cl / cd": 0}
    rows = []
    for r in raw:
        r = {k: _num(v) for k, v in r.items()}
        if r.get("state") not in (None, "", "done"):
            dropped["not done"] += 1
            continue
        if str(r.get("conv_state") or "") in BAD_CONV:
            dropped["diverged / no data"] += 1
            continue
        if not isinstance(r.get("cl"), float) or not isinstance(r.get("cd"), float):
            dropped["no cl / cd"] += 1
            continue
        rows.append(r)
    if not rows:
        raise ValueError(f"{p}: no usable rows ({dropped})")
    keys, constants = [], {}
    for k in cols:
        if k in NOT_GEOMETRY or _is_result(k):
            continue
        if len({repr(r.get(k)) for r in rows}) > 1:
            keys.append(k)
        elif rows[0].get(k) not in (None, ""):
            constants[k] = rows[0].get(k)
    groups: dict[tuple, list[dict]] = {}
    for r in rows:
        groups.setdefault(tuple(repr(r.get(k)) for k in keys), []).append(r)
    synthetic = "SYNTHETIC_NOT_CFD" in cols
    sections = {}
    for grp in groups.values():
        params = {k: grp[0].get(k) for k in keys}
        sec = _build(grp, params, _gid(params, keys), str(p))
        sec.synthetic = synthetic
        sections[sec.geometry_id] = sec
    return SectionSet(p, keys, sections, {k: v for k, v in dropped.items() if v}, synthetic, constants)


def _build(rows: list[dict], params: dict, gid: str, src: str) -> SectionPolar:
    aoa = np.unique([r["aoa"] for r in rows])
    has_h = any(isinstance(r.get("ground_clearance"), float) for r in rows)
    has_v = any(isinstance(r.get("speed_kmh"), float) for r in rows)
    hs = np.unique([r["ground_clearance"] for r in rows]) if has_h else None
    vs = np.unique([r["speed_kmh"] for r in rows]) if has_v else None
    shape = [len(aoa)] + ([len(hs)] if has_h else []) + ([len(vs)] if has_v else [])
    cl = np.full(shape, np.nan)
    cd = np.full(shape, np.nan)
    sums: dict[tuple, list] = {}
    for r in rows:
        idx = [int(np.searchsorted(aoa, r["aoa"]))]
        if has_h:
            idx.append(int(np.searchsorted(hs, r["ground_clearance"])))
        if has_v:
            idx.append(int(np.searchsorted(vs, r["speed_kmh"])))
        sums.setdefault(tuple(idx), []).append((r["cl"], r["cd"]))
    for idx, vals in sums.items():  # repeated cases (reruns): average
        cl[idx] = np.mean([a for a, _ in vals])
        cd[idx] = np.mean([b for _, b in vals])
    # holes in the grid: fill along aoa, then along the other axes
    axes = [aoa] + ([hs] if has_h else []) + ([vs] if has_v else [])
    for arr in (cl, cd):
        for ax, grid in enumerate(axes):
            moved = np.moveaxis(arr, ax, -1)  # a view: writing into it fills arr
            for idx in np.ndindex(moved.shape[:-1]):
                col = moved[idx]
                ok = np.isfinite(col)
                if 0 < ok.sum() < len(col):
                    col[~ok] = np.interp(grid[~ok], grid[ok], col[ok])
    if np.isnan(cl).any():
        raise ValueError(f"section {gid}: the data do not cover a full aoa grid")
    c_ref = rows[0].get("chord")
    if not isinstance(c_ref, float):
        mc = rows[0].get("main_chord")
        if not isinstance(mc, float):
            raise ValueError(f"section {gid}: need a 'chord' [m] or 'main_chord' [mm] column")
        c_ref = mc / 1000.0
    return SectionPolar(gid, params, aoa, hs, vs, cl, cd, float(c_ref), src, len(rows))


def load_sections(path: str | Path) -> SectionSet:
    p = Path(path).resolve()
    return _load_cached(str(p), p.stat().st_mtime)
