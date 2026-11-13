"""Racing line inside the track limits.

The line is the reference line (centre line / driven line of the track file) moved sideways by an
offset ``a_i`` along its left normal ``n_i``: ``Q_i = P_i + a_i n_i``, with
``-(w_right - m) <= a_i <= w_left - m`` (``m`` = half the car width + a clearance to the edge).

Objective (Braghin et al. 2008; Heilmeier et al. 2019 "minimum curvature")::

    min  sum_i kappa_i(a)^2  +  eps * sum_i |Q_{i+1} - Q_i|^2  /  h^4

``kappa_i`` is the discrete curvature of the moved points (central differences), linearised with an
analytic Jacobian and solved as a bounded linear least-squares problem (sparse ``lsq_linear``) until
the offsets stop changing (Gauss-Newton with a trust region). ``eps = 0`` is the minimum-curvature line
(highest corner speeds), a larger ``eps`` pulls the line shorter (shortest path in the limit). With a
vehicle, several ``eps`` are solved and the one with the lowest quasi-steady-state lap time is kept -
an approximation of the minimum-time line that needs no optimal-control solver.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .track import Track

DEFAULT_WEIGHTS = (0.0, 0.002, 0.005, 0.01, 0.02, 0.05)


class LineError(ValueError):
    pass


@dataclass
class RefLine:
    """Uniformly sampled reference line with its normals and edge distances."""

    x: np.ndarray
    y: np.ndarray
    nx: np.ndarray
    ny: np.ndarray
    wl: np.ndarray
    wr: np.ndarray
    z: np.ndarray | None
    closed: bool
    h: float

    @property
    def n(self) -> int:
        return len(self.x)


def reference_line(track: Track, step: float = 1.0) -> RefLine:
    """Reference points every ``step`` metres (closed tracks: the last point is not repeated)."""
    if not track.has_width:
        raise LineError("트랙 폭 정보가 없습니다: --width로 트랙 폭[m]을 주거나 width_left_m/width_right_m 열이 있는 "
                        "트랙(Assetto Corsa 가져오기)을 쓰세요")
    x, y = track.xy()
    length = track.length
    n = max(int(round(length / step)), 8)
    s = np.linspace(0.0, length, n + 1)
    h = length / n
    if track.closed:
        s = s[:-1]
    px, py = np.interp(s, track.s, x), np.interp(s, track.s, y)
    if track.closed:  # spread a small closure gap of an integrated (segment) line over the lap
        frac = s / length
        px = px - frac * (x[-1] - x[0])
        py = py - frac * (y[-1] - y[0])
    wl, wr = np.interp(s, track.s, track.width_left), np.interp(s, track.s, track.width_right)
    z = None if track.z is None else np.interp(s, track.s, track.z)
    if track.closed:
        tx, ty = np.roll(px, -1) - np.roll(px, 1), np.roll(py, -1) - np.roll(py, 1)
    else:
        tx, ty = np.gradient(px), np.gradient(py)
    norm = np.hypot(tx, ty)
    tx, ty = tx / norm, ty / norm
    return RefLine(px, py, -ty, tx, wl, wr, z, track.closed, h)


def _curvature(x, y, closed: bool):
    """Discrete curvature (central differences in the point index) and its parts; open lines: interior only."""
    if closed:
        xm, xp, ym, yp = np.roll(x, 1), np.roll(x, -1), np.roll(y, 1), np.roll(y, -1)
        xc, yc = x, y
    else:
        xm, xp, ym, yp = x[:-2], x[2:], y[:-2], y[2:]
        xc, yc = x[1:-1], y[1:-1]
    a, b = 0.5 * (xp - xm), 0.5 * (yp - ym)
    c, d = xp - 2.0 * xc + xm, yp - 2.0 * yc + ym
    q = a * a + b * b
    den = q ** 1.5
    num = a * d - b * c
    return num / den, (a, b, c, d, q, den, num)


def _curvature_jacobian(ref: RefLine, alpha):
    """kappa_i and the sparse matrix d kappa_i / d alpha_j (j = i-1, i, i+1)."""
    import scipy.sparse as sps

    x, y = ref.x + alpha * ref.nx, ref.y + alpha * ref.ny
    k, (a, b, c, d, q, den, num) = _curvature(x, y, ref.closed)
    n = ref.n
    if ref.closed:
        rows = np.arange(n)
        cols = {-1: (rows - 1) % n, 0: rows, 1: (rows + 1) % n}
    else:
        rows = np.arange(n - 2)
        cols = {-1: rows, 0: rows + 1, 1: rows + 2}
    nx, ny = ref.nx, ref.ny
    data, ri, ci = [], [], []
    sq = np.sqrt(q)
    for off, da_f, dc_f in ((-1, -0.5, 1.0), (0, 0.0, -2.0), (1, 0.5, 1.0)):
        j = cols[off]
        da, db = da_f * nx[j], da_f * ny[j]
        dc, dd = dc_f * nx[j], dc_f * ny[j]
        dnum = d * da + a * dd - c * db - b * dc
        dden = 3.0 * sq * (a * da + b * db)
        data.append((dnum * den - num * dden) / (den * den))
        ri.append(rows)
        ci.append(j)
    jac = sps.csr_matrix((np.concatenate(data), (np.concatenate(ri), np.concatenate(ci))), shape=(len(rows), n))
    return k, jac


def _length_operator(ref: RefLine):
    """Rows of Q_{i+1} - Q_i = (P_{i+1} - P_i) + a_{i+1} n_{i+1} - a_i n_i (x and y), as (L, dP)."""
    import scipy.sparse as sps

    n = ref.n
    i = np.arange(n) if ref.closed else np.arange(n - 1)
    ip = (i + 1) % n
    m = len(i)
    rows = np.concatenate([np.arange(m), np.arange(m), m + np.arange(m), m + np.arange(m)])
    cols = np.concatenate([ip, i, ip, i])
    data = np.concatenate([ref.nx[ip], -ref.nx[i], ref.ny[ip], -ref.ny[i]])
    op = sps.csr_matrix((data, (rows, cols)), shape=(2 * m, n))
    dp = np.concatenate([ref.x[ip] - ref.x[i], ref.y[ip] - ref.y[i]])
    return op, dp


def _box_qp(hess, c, lb, ub, x0, max_iter: int = 50):
    """min 1/2 x'Hx - c'x  s.t. lb <= x <= ub (H sparse SPD): projected Newton (Bertsekas) with
    a sparse factorisation of the free block per iteration."""
    from scipy.sparse.linalg import splu

    x = np.clip(x0, lb, ub)
    hess = hess.tocsc()

    def f(v):
        return 0.5 * float(v @ (hess @ v)) - float(c @ v)

    fx = f(x)
    eps = 1e-10
    for _ in range(max_iter):
        g = hess @ x - c
        active = ((x <= lb + eps) & (g > 0.0)) | ((x >= ub - eps) & (g < 0.0))
        free = np.flatnonzero(~active)
        if free.size == 0:
            break
        d = np.zeros_like(x)
        sub = hess[free][:, free].tocsc()
        d[free] = splu(sub).solve(-g[free])
        t, improved = 1.0, False
        for _ls in range(30):  # projected (Armijo) line search
            xn = np.clip(x + t * d, lb, ub)
            fn = f(xn)
            if fn <= fx + 1e-4 * float(g @ (xn - x)):
                improved = True
                break
            t *= 0.5
        if not improved:
            break
        step = np.max(np.abs(xn - x))
        x, fx = xn, fn
        if step < 1e-9:
            break
    return x


def optimal_offsets(ref: RefLine, lo: np.ndarray, hi: np.ndarray, length_weight: float = 0.0,
                    alpha0: np.ndarray | None = None, max_iter: int = 60, tol: float = 2e-4,
                    trust: float = 0.75) -> tuple[np.ndarray, int]:
    """Offsets minimising sum kappa^2 + length_weight * sum |dQ|^2 / h^4 within ``lo <= a <= hi``."""
    import scipy.sparse as sps

    n = ref.n
    alpha = np.clip(np.zeros(n) if alpha0 is None else np.asarray(alpha0, dtype=float), lo, hi)
    lop, dp = _length_operator(ref)
    w_len = math.sqrt(max(length_weight, 0.0)) / ref.h ** 2
    reg = 1e-3 / ref.h ** 2  # small step damping (Levenberg-Marquardt)
    it = 0
    for it in range(1, max_iter + 1):
        k, jac = _curvature_jacobian(ref, alpha)
        blocks, rhs = [jac], [-k]
        if w_len > 0.0:
            blocks.append(w_len * lop)
            rhs.append(-w_len * (lop @ alpha + dp))
        blocks.append(reg * sps.identity(n, format="csr"))
        rhs.append(np.zeros(n))
        a_mat = sps.vstack(blocks).tocsr()
        b = np.concatenate(rhs)
        lb = np.maximum(lo - alpha, -trust)
        ub = np.minimum(hi - alpha, trust)
        ub = np.maximum(ub, lb)
        step = _box_qp((a_mat.T @ a_mat).tocsc(), a_mat.T @ b, lb, ub, np.zeros(n))
        alpha = np.clip(alpha + step, lo, hi)
        if float(np.max(np.abs(step))) < tol:
            break
    return alpha, it


@dataclass
class Candidate:
    weight: float
    lap_time: float
    length: float
    max_kappa: float
    iterations: int
    alpha: np.ndarray = field(repr=False)
    track: Track = field(repr=False)


@dataclass
class LineResult:
    ref: RefLine
    lo: np.ndarray
    hi: np.ndarray
    best: Candidate
    candidates: list[Candidate]
    reference_lap: object  # LapResult on the input line
    lap: object  # LapResult on the chosen line
    car_width: float
    clearance: float
    narrow_points: int = 0
    csv_path: Path | None = None  # where write_csv put the line

    @property
    def gain_s(self) -> float:
        return self.reference_lap.lap_time - self.lap.lap_time

    def edge_lines(self):
        r = self.ref
        return ((r.x + r.wl * r.nx, r.y + r.wl * r.ny), (r.x - r.wr * r.nx, r.y - r.wr * r.ny))

    def line_xy(self):
        r = self.ref
        return r.x + self.best.alpha * r.nx, r.y + self.best.alpha * r.ny

    def report(self) -> str:
        r, b = self.ref, self.best
        lines = [f"racing line - {self.lap.track_name}",
                 f"  reference line: {len(r.x)} points every {r.h:.2f} m, track width {np.min(r.wl + r.wr):.2f}-"
                 f"{np.max(r.wl + r.wr):.2f} m, car {self.car_width:.2f} m + {self.clearance:.2f} m to the edge "
                 f"-> lateral freedom {np.min(self.hi - self.lo):.2f}-{np.max(self.hi - self.lo):.2f} m",
                 ""]
        if self.narrow_points:
            lines.append(f"  WARNING: at {self.narrow_points} points the track is narrower than the car + clearance "
                         "(line kept on the reference there)")
        lines.append(f"  {'length weight':>13s} {'lap [s]':>9s} {'length [m]':>10s} {'max kappa':>10s} {'iter':>5s}")
        for c in self.candidates:
            mark = "  <- chosen" if c is b else ""
            lines.append(f"  {c.weight:13.4g} {c.lap_time:9.3f} {c.length:10.1f} {c.max_kappa:10.4f} {c.iterations:5d}"
                         + mark)
        ref_len = self.reference_lap.s[-1]
        lines += ["",
                  f"  input line:    lap {self.reference_lap.lap_time:8.3f} s, length {ref_len:7.1f} m, max |kappa| "
                  f"{np.max(np.abs(self.reference_lap.kappa)):.4f} 1/m",
                  f"  optimised:     lap {self.lap.lap_time:8.3f} s, length {self.lap.s[-1]:7.1f} m, max |kappa| "
                  f"{np.max(np.abs(self.lap.kappa)):.4f} 1/m  ->  {self.gain_s:+.3f} s "
                  f"({self.gain_s / self.reference_lap.lap_time * 100:+.2f} %)",
                  f"  offset from the input line: {np.min(b.alpha):+.2f} .. {np.max(b.alpha):+.2f} m (left +), "
                  f"{np.mean(np.abs(b.alpha) > np.minimum(self.hi, -self.lo) - 0.05) * 100:.0f} % of the points at "
                  "an edge"]
        return "\n".join(lines)

    def write_csv(self, path: str | Path, name: str | None = None) -> Path:
        """The optimised line as a track CSV (``x_m, y_m[, z_m], width_left_m, width_right_m``)."""
        path = Path(path)
        r, a = self.ref, self.best.alpha
        x, y = self.line_xy()
        wl, wr = r.wl - a, r.wr + a
        with open(path, "w", newline="", encoding="utf-8") as fh:
            fh.write(f"# name: {name or self.lap.track_name}\n# closed: {'true' if r.closed else 'false'}\n")
            fh.write("# cutoff_m: 3\n# source: aerolap racing line (minimum curvature + length weight "
                     f"{self.best.weight:g}, chosen by QSS lap time)\n")
            w = csv.writer(fh)
            w.writerow(["x_m", "y_m"] + (["z_m"] if r.z is not None else []) + ["width_left_m", "width_right_m"])
            for i in range(r.n):
                row = [f"{x[i]:.3f}", f"{y[i]:.3f}"] + ([f"{r.z[i]:.3f}"] if r.z is not None else [])
                w.writerow(row + [f"{wl[i]:.3f}", f"{wr[i]:.3f}"])
        self.csv_path = path
        return path


def line_track(ref: RefLine, alpha: np.ndarray, name: str, ds: float = 0.5, cutoff_m: float = 3.0) -> Track:
    """Track along the moved points (widths and elevation follow)."""
    x, y = ref.x + alpha * ref.nx, ref.y + alpha * ref.ny
    if ref.closed:
        x, y = np.append(x, x[0]), np.append(y, y[0])
        wl, wr = np.append(ref.wl - alpha, ref.wl[0] - alpha[0]), np.append(ref.wr + alpha, ref.wr[0] + alpha[0])
        z = None if ref.z is None else np.append(ref.z, ref.z[0])
    else:
        wl, wr, z = ref.wl - alpha, ref.wr + alpha, ref.z
    return Track.from_xy(x, y, name=name, closed=ref.closed, ds=ds, cutoff_m=cutoff_m, z=z, width_left=wl,
                         width_right=wr)


def optimise_line(vehicle, track: Track, car_width: float | None = None, clearance: float = 0.2,
                  step: float = 1.0, weights=DEFAULT_WEIGHTS, opts=None, start: str | None = None,
                  progress=None) -> LineResult:
    """Racing line within the track width, the length weight chosen by the QSS lap time of ``vehicle``."""
    from .solvers.qss import QSSSolver

    if car_width is None:
        car_width = max(vehicle.front.track, vehicle.rear.track) + 0.2 if vehicle is not None else 1.4
    ref = reference_line(track, step)
    margin = 0.5 * car_width + clearance
    hi, lo = ref.wl - margin, -(ref.wr - margin)
    narrow = hi < lo
    mid = 0.5 * (hi + lo)
    hi = np.where(narrow, np.clip(mid, -ref.wr, ref.wl), hi)
    lo = np.where(narrow, hi, lo)
    start = start or ("flying" if track.closed else "standing")
    solver = QSSSolver(opts)
    ds = solver.opts.ds
    reference_lap = solver.solve(vehicle, track, start=start)
    cands: list[Candidate] = []
    alpha = None
    for k, w in enumerate(weights):
        alpha, it = optimal_offsets(ref, lo, hi, w, alpha0=alpha)
        trk = line_track(ref, alpha, f"{track.name} (racing line)", ds=min(ds, 0.5))
        lap = solver.solve(vehicle, trk, start=start)
        cands.append(Candidate(float(w), lap.lap_time, trk.length, float(np.max(np.abs(trk.kappa))), it,
                               alpha.copy(), trk))
        if progress:
            progress(k + 1, len(weights), float(w), lap.lap_time)
    best = min(cands, key=lambda c: c.lap_time)
    lap = solver.solve(vehicle, best.track, start=start)
    return LineResult(ref, lo, hi, best, cands, reference_lap, lap, float(car_width), float(clearance),
                      int(narrow.sum()))
