"""Aero maps: aerodynamic coefficients as a function of the car's attitude.

Every map returns, per aero mode, the axle downforce areas ``ClA_f``, ``ClA_r``
(m^2, positive = downforce) and the drag area ``CdA`` (m^2) for given front/rear
ride heights (mm), body roll (deg) and yaw / sideslip angle (deg). Roll and yaw
are treated as symmetric (absolute values).

CSV format (one row per sample, ``#`` comment lines allowed)::

    mode,front_rh_mm,rear_rh_mm,roll_deg,yaw_deg,ClA,CdA,cop_front

``roll_deg`` and ``yaw_deg`` are optional columns. ``cop_front`` is the fraction of
the total downforce acting on the front axle.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.interpolate import RegularGridInterpolator

DIMS = ("front_rh_mm", "rear_rh_mm", "roll_deg", "yaw_deg")
_ALIASES = {
    "mode": "mode",
    "front_rh_mm": "front_rh_mm", "rh_front_mm": "front_rh_mm",
    "rear_rh_mm": "rear_rh_mm", "rh_rear_mm": "rear_rh_mm",
    "roll_deg": "roll_deg", "yaw_deg": "yaw_deg",
    "cla": "ClA", "cda": "CdA", "cop_front": "cop_front", "aero_balance_front": "cop_front",
}


def read_aero_csv(path: str | Path) -> tuple[list[str], dict[str, dict[str, np.ndarray]]]:
    """Read an aero CSV into ``(mode_order, {mode: {column: array}})``.

    ``ClA``/``cop_front`` are converted to ``ClA_f``/``ClA_r``.
    """
    with open(path, encoding="utf-8") as fh:
        lines = [ln for ln in fh if ln.strip() and not ln.lstrip().startswith("#")]
    reader = csv.DictReader(lines)
    if reader.fieldnames is None:
        raise ValueError(f"{path}: empty aero map")
    names = {}
    for col in reader.fieldnames:
        key = _ALIASES.get(col.strip().lower())
        if key is None:
            raise ValueError(f"{path}: unknown aero map column {col!r}")
        names[col] = key
    for needed in ("ClA", "CdA", "cop_front"):
        if needed not in names.values():
            raise ValueError(f"{path}: aero map needs a {needed} column")
    rows: dict[str, dict[str, list]] = {}
    order: list[str] = []
    for raw in reader:
        rec = {names[k]: v.strip() for k, v in raw.items() if k in names}
        mode = rec.get("mode", "default") or "default"
        if mode not in rows:
            rows[mode] = {}
            order.append(mode)
        for key, val in rec.items():
            if key == "mode":
                continue
            rows[mode].setdefault(key, []).append(float(val))
    out: dict[str, dict[str, np.ndarray]] = {}
    for mode, cols in rows.items():
        arr = {k: np.asarray(v, dtype=float) for k, v in cols.items()}
        cla, cop = arr.pop("ClA"), arr.pop("cop_front")
        arr["ClA_f"] = cla * cop
        arr["ClA_r"] = cla * (1.0 - cop)
        out[mode] = arr
    return order, out


def _active_dims(data: dict[str, np.ndarray]) -> list[str]:
    return [d for d in DIMS if d in data and np.unique(data[d]).size > 1]


def _stack_inputs(dims, bounds, rh_f, rh_r, roll, yaw):
    vals = {"front_rh_mm": rh_f, "rear_rh_mm": rh_r, "roll_deg": roll, "yaw_deg": yaw}
    arrays = np.broadcast_arrays(*(np.asarray(vals[d], dtype=float) for d in DIMS))
    shape = arrays[0].shape
    lookup = dict(zip(DIMS, arrays))
    cols, oor = [], np.zeros(shape, dtype=bool)
    for d in dims:
        lo, hi = bounds[d]
        x = lookup[d]
        oor |= (x < lo - 1e-9) | (x > hi + 1e-9)
        cols.append(np.clip(x, lo, hi).reshape(-1))
    return shape, cols, oor


class ConstantAero:
    """Attitude-independent coefficients per mode (what OptimumLap uses)."""

    def __init__(self, modes: dict[str, tuple[float, float, float]]):
        self.mode_names = list(modes)
        self._coef = [modes[m] for m in self.mode_names]  # (ClA, CdA, cop_front)

    def ranges(self, mode: int) -> dict[str, tuple[float, float]]:
        return {}

    @classmethod
    def from_config(cls, aero: dict) -> "ConstantAero":
        if "modes" in aero:
            modes = {}
            for name, c in aero["modes"].items():
                cla = float(c["cla"]) if "cla" in c else float(c["cl"]) * float(c["frontal_area_m2"])
                cda = float(c["cda"]) if "cda" in c else float(c["cd"]) * float(c["frontal_area_m2"])
                modes[name] = (cla, cda, float(c.get("cop_front", 0.5)))
            return cls(modes)
        area = float(aero.get("frontal_area_m2", 1.0))
        cla = float(aero["cla"]) if "cla" in aero else float(aero.get("cl", 0.0)) * area
        cda = float(aero["cda"]) if "cda" in aero else float(aero.get("cd", 0.0)) * area
        return cls({"default": (cla, cda, float(aero.get("cop_front", 0.5)))})

    def coefficients(self, mode: int, rh_f, rh_r, roll, yaw):
        shape = np.broadcast(np.asarray(rh_f), np.asarray(rh_r), np.asarray(roll), np.asarray(yaw)).shape
        cla, cda, cop = self._coef[mode]
        ones = np.ones(shape)
        return cla * cop * ones, cla * (1 - cop) * ones, cda * ones, np.zeros(shape, dtype=bool)


class TableAero:
    """Full-factorial grid per mode, multilinear interpolation, clamped at the edges."""

    def __init__(self, mode_order: list[str], data: dict[str, dict[str, np.ndarray]]):
        self.mode_names = list(mode_order)
        self._modes = []
        for mode in self.mode_names:
            d = data[mode]
            dims = _active_dims(d)
            values = np.stack([d["ClA_f"], d["ClA_r"], d["CdA"]], axis=-1)
            if not dims:
                self._modes.append((dims, {}, None, values.mean(axis=0)))
                continue
            axes = [np.unique(d[k]) for k in dims]
            n_expected = int(np.prod([a.size for a in axes]))
            if values.shape[0] != n_expected:
                raise ValueError(
                    f"aero map mode {mode!r}: {values.shape[0]} rows but the grid "
                    f"{' x '.join(str(a.size) for a in axes)} needs {n_expected} (use a full grid "
                    "or type: response_surface for scattered CFD points)")
            grid = np.full([a.size for a in axes] + [3], np.nan)
            idx = tuple(np.searchsorted(a, d[k]) for a, k in zip(axes, dims))
            grid[idx] = values
            if np.isnan(grid).any():
                raise ValueError(f"aero map mode {mode!r}: duplicated or missing grid points")
            interp = RegularGridInterpolator(axes, grid, method="linear")
            bounds = {k: (a[0], a[-1]) for k, a in zip(dims, axes)}
            self._modes.append((dims, bounds, interp, None))

    @classmethod
    def from_csv(cls, path) -> "TableAero":
        return cls(*read_aero_csv(path))

    def ranges(self, mode: int) -> dict[str, tuple[float, float]]:
        return dict(self._modes[mode][1])

    def coefficients(self, mode: int, rh_f, rh_r, roll, yaw):
        dims, bounds, interp, const = self._modes[mode]
        shape, cols, oor = _stack_inputs(dims, bounds, rh_f, rh_r, roll, yaw)
        if interp is None:
            ones = np.ones(shape)
            return const[0] * ones, const[1] * ones, const[2] * ones, oor
        vals = interp(np.stack(cols, axis=-1)).reshape(shape + (3,))
        return vals[..., 0], vals[..., 1], vals[..., 2], oor


@dataclass
class SurfaceFit:
    mode: str
    dims: list[str]
    n_points: int
    r2: dict[str, float]
    rms: dict[str, float]


class ResponseSurfaceAero:
    """Quadratic response surface fitted to scattered CFD / wind-tunnel points."""

    TARGETS = ("ClA_f", "ClA_r", "CdA")

    def __init__(self, mode_order: list[str], data: dict[str, dict[str, np.ndarray]]):
        self.mode_names = list(mode_order)
        self._modes = []
        self.fits: list[SurfaceFit] = []
        for mode in self.mode_names:
            d = data[mode]
            dims = _active_dims(d)
            x = np.stack([d[k] for k in dims], axis=-1) if dims else np.zeros((len(d["CdA"]), 0))
            mu = x.mean(axis=0) if dims else np.zeros(0)
            sd = x.std(axis=0) if dims else np.ones(0)
            sd = np.where(sd > 0, sd, 1.0)
            phi = self._basis((x - mu) / sd)
            if phi.shape[0] < phi.shape[1]:
                raise ValueError(
                    f"aero mode {mode!r}: {phi.shape[0]} points are not enough for a quadratic "
                    f"surface in {len(dims)} variables (need >= {phi.shape[1]})")
            y = np.stack([d[t] for t in self.TARGETS], axis=-1)
            coef, *_ = np.linalg.lstsq(phi, y, rcond=None)
            pred = phi @ coef
            ss_res = ((y - pred) ** 2).sum(axis=0)
            ss_tot = ((y - y.mean(axis=0)) ** 2).sum(axis=0)
            r2 = np.where(ss_tot > 0, 1.0 - ss_res / np.where(ss_tot > 0, ss_tot, 1.0), 1.0)
            rms = np.sqrt(ss_res / y.shape[0])
            bounds = {k: (float(x[:, i].min()), float(x[:, i].max())) for i, k in enumerate(dims)}
            self._modes.append((dims, bounds, mu, sd, coef))
            self.fits.append(SurfaceFit(mode, dims, y.shape[0],
                                        dict(zip(self.TARGETS, r2.tolist())),
                                        dict(zip(self.TARGETS, rms.tolist()))))

    @classmethod
    def from_csv(cls, path) -> "ResponseSurfaceAero":
        return cls(*read_aero_csv(path))

    @staticmethod
    def _basis(z: np.ndarray) -> np.ndarray:
        n, k = z.shape
        cols = [np.ones(n)]
        cols += [z[:, i] for i in range(k)]
        cols += [z[:, i] * z[:, j] for i in range(k) for j in range(i, k)]
        return np.stack(cols, axis=-1)

    def ranges(self, mode: int) -> dict[str, tuple[float, float]]:
        return dict(self._modes[mode][1])

    def coefficients(self, mode: int, rh_f, rh_r, roll, yaw):
        dims, bounds, mu, sd, coef = self._modes[mode]
        shape, cols, oor = _stack_inputs(dims, bounds, rh_f, rh_r, roll, yaw)
        n = int(np.prod(shape)) if shape else 1
        x = np.stack(cols, axis=-1) if cols else np.zeros((n, 0))
        vals = (self._basis((x - mu) / sd) @ coef).reshape(shape + (3,))
        return vals[..., 0], vals[..., 1], vals[..., 2], oor


class AeroModel:
    """Aero map + setup modifiers (``cla_scale``, ``cda_scale``, ``cop_shift``)."""

    def __init__(self, aero_cfg: dict, base_dir: str | Path = "."):
        kind = aero_cfg.get("type", "constant")
        if kind == "constant":
            self.map = ConstantAero.from_config(aero_cfg)
        else:
            path = Path(aero_cfg["file"])
            if not path.is_absolute():
                path = Path(base_dir) / path
            if kind == "table":
                self.map = TableAero.from_csv(path)
            elif kind == "response_surface":
                self.map = ResponseSurfaceAero.from_csv(path)
            else:
                raise ValueError(f"unknown aero.type {kind!r} (constant | table | response_surface)")
        self.kind = kind
        self.mode_names = self.map.mode_names
        default = aero_cfg.get("default_mode") or self.mode_names[0]
        if default not in self.mode_names:
            raise ValueError(f"aero.default_mode {default!r} not in aero modes {self.mode_names}")
        self.default_mode = self.mode_names.index(default)
        self.cla_scale = float(aero_cfg.get("cla_scale", 1.0))
        self.cda_scale = float(aero_cfg.get("cda_scale", 1.0))
        self.cop_shift = float(aero_cfg.get("cop_shift", 0.0))

    def mode_index(self, name: str | None, fallback: int) -> int:
        if name is None:
            return fallback
        if name not in self.mode_names:
            raise ValueError(f"aero mode {name!r} not in {self.mode_names}")
        return self.mode_names.index(name)

    def coefficients(self, mode: int, rh_f_mm, rh_r_mm, roll_deg, yaw_deg):
        cla_f, cla_r, cda, oor = self.map.coefficients(mode, rh_f_mm, rh_r_mm,
                                                       np.abs(roll_deg), np.abs(yaw_deg))
        if self.cla_scale != 1.0 or self.cop_shift != 0.0:
            total = cla_f + cla_r
            with np.errstate(divide="ignore", invalid="ignore"):
                cop = np.where(np.abs(total) > 1e-9, cla_f / np.where(total == 0, 1.0, total), 0.5)
            cop = cop + self.cop_shift
            total = total * self.cla_scale
            cla_f, cla_r = total * cop, total * (1.0 - cop)
        return cla_f, cla_r, cda * self.cda_scale, oor


def grid_rows(aero_map, step_mm: float = 5.0, step_deg: float = 0.5) -> list[str]:
    """Sample any aero map on a full-factorial grid (CSV lines incl. header) for ``TableAero``."""
    dims_all: list[str] = []
    for m in range(len(aero_map.mode_names)):
        for d in aero_map.ranges(m):
            if d not in dims_all:
                dims_all.append(d)
    dims_all = [d for d in DIMS if d in dims_all]
    lines = [",".join(["mode"] + dims_all + ["ClA", "CdA", "cop_front"])]
    for m, name in enumerate(aero_map.mode_names):
        rng = aero_map.ranges(m)
        axes = []
        for d in dims_all:
            if d not in rng:
                axes.append(np.array([0.0]))
                continue
            lo, hi = rng[d]
            step = step_mm if d.endswith("_mm") else step_deg
            lo_r, hi_r = math.floor(lo / step) * step, math.ceil(hi / step) * step
            axes.append(np.arange(lo_r, hi_r + 0.5 * step, step) if hi_r > lo_r else np.array([lo]))
        mesh = np.meshgrid(*axes, indexing="ij") if axes else []
        cols = {d: g.reshape(-1) for d, g in zip(dims_all, mesh)}
        n = len(next(iter(cols.values()))) if cols else 1
        args = [cols.get(d, np.zeros(n)) for d in DIMS]
        cla_f, cla_r, cda, _ = aero_map.coefficients(m, *args)
        cla = cla_f + cla_r
        cop = np.where(np.abs(cla) > 1e-9, cla_f / np.where(cla == 0, 1.0, cla), 0.5)
        for i in range(n):
            vals = [f"{cols[d][i]:g}" for d in dims_all]
            lines.append(",".join([name] + vals + [f"{cla[i]:.4f}", f"{cda[i]:.4f}", f"{cop[i]:.4f}"]))
    return lines
