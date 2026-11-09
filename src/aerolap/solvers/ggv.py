"""g-g-V performance envelope of the coupled vehicle model.

For every speed on a grid the envelope stores

* ``ay_max(v)`` – steady-state cornering limit at ax = 0 (the tyres also overcome drag),
* ``ax_acc(v, ay)`` / ``ax_brk(v, ay)`` – longitudinal limits for ``ay = f * ay_max(v)``,
* ``ax_coast(v, ay)`` – deceleration with zero tyre longitudinal force (gear shifts).

Every grid point is a full quasi-static equilibrium (aero <-> loads <-> ride height /
roll / yaw), so braking pitch, cornering roll etc. are already inside the envelope.
The fraction axis is ``f = sin(theta)`` with uniform ``theta`` to resolve the steep
part of the envelope near the cornering limit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class SolverOptions:
    ds: float = 0.5  # track step, m
    v_min: float = 1.0  # first speed of the envelope grid, m/s
    v_step: float = 0.5
    n_frac: int = 17  # lateral fraction samples
    bisect_iters: int = 22
    ay_upper: float = 80.0  # m/s^2, search bound
    ax_upper: float = 40.0  # m/s^2, search bound
    eq_max_iter: int = 40
    eq_tol: float = 1e-5

    @classmethod
    def coarse(cls) -> "SolverOptions":
        """Faster settings for optimisation loops (~0.1 % lap-time error)."""
        return cls(ds=1.0, v_step=1.5, n_frac=9, bisect_iters=16, eq_tol=5e-5)


def _bisect(fn, ok, bad, iters):
    """Vectorised bisection between known-feasible ``ok`` and infeasible ``bad`` bounds."""
    ok = np.array(ok, dtype=float)
    bad = np.array(bad, dtype=float)
    state = None
    for _ in range(iters):
        mid = 0.5 * (ok + bad)
        feas, state = fn(mid, state)
        ok = np.where(feas, mid, ok)
        bad = np.where(feas, bad, mid)
    return ok


def ay_max_curve(vehicle, mode: int, speeds, opts: SolverOptions | None = None) -> np.ndarray:
    """Steady-state lateral limit (ax = 0) for each speed; 0 where even ay = 0 is infeasible."""
    opts = opts or SolverOptions()
    v = np.asarray(speeds, dtype=float)
    kw = dict(max_iter=opts.eq_max_iter, tol=opts.eq_tol)
    st0 = vehicle.solve_state(v, 0.0, 0.0, mode, **kw)
    hi = np.full(v.shape, opts.ay_upper)
    st_hi = vehicle.solve_state(v, 0.0, hi, mode, **kw)

    def fn(ay, init):
        st = vehicle.solve_state(v, 0.0, ay, mode, init=init, **kw)
        return st.feasible, st

    res = _bisect(fn, np.zeros(v.shape), hi, opts.bisect_iters)
    res = np.where(st_hi.feasible, hi, res)
    return np.where(st0.feasible, res, 0.0)


class GGV:
    def __init__(self, mode: int, v: np.ndarray, ay_max: np.ndarray, fracs: np.ndarray,
                 ax_acc: np.ndarray, ax_brk: np.ndarray, ax_coast: np.ndarray):
        self.mode = mode
        self.v = v
        self.ay_max = ay_max
        self.fracs = fracs
        self.ax_acc = ax_acc
        self.ax_brk = ax_brk
        self.ax_coast = ax_coast
        self.v0 = float(v[0])
        self.dv = float(v[1] - v[0])
        self.nv = len(v)
        self.nf = len(fracs)
        # Python lists for fast scalar interpolation inside the speed integration loops.
        self._aym = ay_max.tolist()
        self._acc = ax_acc.tolist()
        self._brk = ax_brk.tolist()
        self._coast = ax_coast.tolist()
        self._fscale = (self.nf - 1) / (0.5 * math.pi)

    @property
    def v_max(self) -> float:
        return float(self.v[-1])

    # ---------------------------------------------------------- scalar lookups
    def _vi(self, v: float):
        x = (v - self.v0) / self.dv
        if x <= 0.0:
            return 0, 0.0
        if x >= self.nv - 1:
            return self.nv - 2, 1.0
        i = int(x)
        return i, x - i

    def ay_max_at(self, v: float) -> float:
        i, t = self._vi(v)
        a = self._aym
        return a[i] + (a[i + 1] - a[i]) * t

    def _lookup(self, tab, v: float, ay: float) -> float:
        i, t = self._vi(v)
        a = self._aym
        aym = a[i] + (a[i + 1] - a[i]) * t
        if aym > 1e-9:
            f = ay / aym
            f = 0.0 if f < 0.0 else (1.0 if f > 1.0 else f)
        else:
            f = 1.0 if ay > 1e-9 else 0.0
        th = math.asin(f) * self._fscale
        j = int(th)
        if j >= self.nf - 1:
            j = self.nf - 2
        u = th - j
        r0, r1 = tab[i], tab[i + 1]
        lo = r0[j] + (r0[j + 1] - r0[j]) * u
        hi = r1[j] + (r1[j + 1] - r1[j]) * u
        return lo + (hi - lo) * t

    def acc_at(self, v: float, ay: float) -> float:
        return self._lookup(self._acc, v, ay)

    def brk_at(self, v: float, ay: float) -> float:
        return self._lookup(self._brk, v, ay)

    def coast_at(self, v: float, ay: float) -> float:
        return self._lookup(self._coast, v, ay)

    # ---------------------------------------------------------- vector lookups
    def apex_speed(self, kappa_abs: np.ndarray) -> np.ndarray:
        """Highest speed reachable from below with v^2 |kappa| <= ay_max(v)."""
        k = np.asarray(kappa_abs, dtype=float)
        out = np.full(k.shape, self.v_max)
        idx = np.flatnonzero(k > 1e-9)
        if idx.size == 0:
            return out
        kk = k[idx][:, None]
        g = self.ay_max[None, :] - self.v[None, :] ** 2 * kk
        neg = g < 0.0
        has = neg.any(axis=1)
        first = np.argmax(neg, axis=1)
        res = np.full(idx.size, self.v_max)
        j = first[has]
        rows = np.flatnonzero(has)
        jm = np.maximum(j - 1, 0)
        g0 = g[rows, jm]
        g1 = g[rows, j]
        v0 = self.v[jm]
        v1 = self.v[j]
        with np.errstate(divide="ignore", invalid="ignore"):
            frac = np.where(g0 - g1 > 0, g0 / (g0 - g1), 0.0)
        interp = v0 + (v1 - v0) * np.clip(frac, 0.0, 1.0)
        below_grid = j == 0  # limit is below the first grid speed
        low = np.sqrt(np.maximum(self.ay_max[0], 0.0) / kk[rows, 0])
        res[rows] = np.where(below_grid, np.minimum(low, self.v[0]), interp)
        out[idx] = res
        return out

    def table_vec(self, which: str, v, ay):
        """Vectorised bilinear lookup of ``acc``, ``brk`` or ``coast``."""
        tab = {"acc": self.ax_acc, "brk": self.ax_brk, "coast": self.ax_coast}[which]
        v = np.asarray(v, dtype=float)
        ay = np.abs(np.asarray(ay, dtype=float))
        x = np.clip((v - self.v0) / self.dv, 0.0, self.nv - 1 - 1e-9)
        i = x.astype(int)
        t = x - i
        aym = self.ay_max[i] + (self.ay_max[i + 1] - self.ay_max[i]) * t
        with np.errstate(divide="ignore", invalid="ignore"):
            f = np.where(aym > 1e-9, ay / np.where(aym > 1e-9, aym, 1.0), (ay > 1e-9).astype(float))
        th = np.arcsin(np.clip(f, 0.0, 1.0)) * self._fscale
        j = np.minimum(th.astype(int), self.nf - 2)
        u = th - j
        lo = tab[i, j] + (tab[i, j + 1] - tab[i, j]) * u
        hi = tab[i + 1, j] + (tab[i + 1, j + 1] - tab[i + 1, j]) * u
        return lo + (hi - lo) * t


def compute_ggv(vehicle, mode: int, opts: SolverOptions | None = None) -> GGV:
    opts = opts or SolverOptions()
    v_hi = min(vehicle.powertrain.v_max + 2.0 * opts.v_step, 120.0)
    v = np.arange(opts.v_min, v_hi + 0.5 * opts.v_step, opts.v_step)
    ay_max = ay_max_curve(vehicle, mode, v, opts)
    fracs = np.sin(np.linspace(0.0, 0.5 * math.pi, opts.n_frac))
    V, F = np.meshgrid(v, fracs, indexing="ij")
    AY = F * ay_max[:, None]
    kw = dict(max_iter=opts.eq_max_iter, tol=opts.eq_tol)
    st0 = vehicle.solve_state(V, 0.0, AY, mode, **kw)
    coast = vehicle.coast_ax(st0)
    st_c = vehicle.solve_state(V, coast, AY, mode, init=st0, **kw)
    coast = vehicle.coast_ax(st_c)

    def fn(ax, init):
        st = vehicle.solve_state(V, ax, AY, mode, init=init, **kw)
        return st.feasible, st

    ax_acc = _bisect(fn, coast, coast + opts.ax_upper, opts.bisect_iters)
    ax_brk = _bisect(fn, coast, coast - opts.ax_upper, opts.bisect_iters)
    return GGV(mode, v, ay_max, fracs, ax_acc, ax_brk, coast)
