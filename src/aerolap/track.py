"""Track description as curvature along the centre line, discretised at a fixed step.

Supported CSV inputs (``#`` comment lines allowed; ``# name: ...`` and
``# closed: true|false`` set metadata):

* segments (like the OptimumLap track editor)::

      type,length_m,radius_m,direction
      straight,75,,
      corner,23.6,15,left

* centre-line / driven-line coordinates: ``x_m,y_m`` (Whittaker smoothing, ``# cutoff_m: 8``)
* curvature: ``s_m,curvature_1pm`` or ``s_m,radius_m[,direction]``
* Assetto Corsa AI spline ``fast_lane.ai`` / ``ideal_line.ai`` and OptimumLap ``.oltra``
  (see :mod:`aerolap.importers`)

Curvature is positive for left-hand corners.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class Track:
    """Nodes ``s`` (not necessarily equally spaced) and curvature.

    ``kappa`` is the curvature at the nodes (at a segment boundary: the tighter of the
    two segments, which sets the corner speed there). ``kappa_int`` is the curvature of
    each interval between two nodes; the speed integration uses it so that corner
    entries and exits are resolved exactly for segment tracks.
    """

    name: str
    s: np.ndarray  # node positions, s[0] = 0, s[-1] = length
    kappa: np.ndarray  # curvature at the nodes, 1/m
    closed: bool
    x: np.ndarray | None = None
    y: np.ndarray | None = None
    kappa_int: np.ndarray | None = None
    segments: list | None = None
    z: np.ndarray | None = None  # elevation at the nodes, m (optional)
    width_left: np.ndarray | None = None  # distance from the line to the left / right track edge, m (optional)
    width_right: np.ndarray | None = None
    use_grade: bool = True  # the solvers use the slope of z ('# grade: off' in a track CSV switches it off)
    grade_smoothing_m: float = 10.0  # elevation is smoothed over this length before the slope is taken

    def interval_grade(self) -> np.ndarray | None:
        """sin(slope) of each interval (positive uphill), from the smoothed elevation; None when flat."""
        if self.z is None or not self.use_grade:
            return None
        z = np.asarray(self.z, dtype=float)
        if float(np.ptp(z)) < 1e-6:
            return None
        ds = np.diff(self.s)
        h = float(np.mean(ds))
        w = max(1, int(round(self.grade_smoothing_m / max(h, 1e-6))))
        if w > 1:
            kernel = np.ones(w) / w
            if self.closed:
                zz = np.concatenate([z[-w - 1:-1], z, z[1:w + 1]])
                z = np.convolve(zz, kernel, mode="same")[w:-w]
            else:  # point reflection at the ends keeps a constant slope exact
                w = min(w, len(z) - 1)
                pad = np.concatenate([2.0 * z[0] - z[w:0:-1], z, 2.0 * z[-1] - z[-2:-w - 2:-1]])
                z = np.convolve(pad, np.ones(w) / w, mode="same")[w:-w]
        slope = np.clip(np.diff(z) / np.maximum(ds, 1e-9), -0.5, 0.5)
        return slope / np.sqrt(1.0 + slope * slope)

    def node_grade(self) -> np.ndarray | None:
        g = self.interval_grade()
        if g is None:
            return None
        if self.closed:
            return 0.5 * (np.concatenate([g[-1:], g]) + np.concatenate([g, g[:1]]))
        return 0.5 * (np.concatenate([g[:1], g]) + np.concatenate([g, g[-1:]]))

    @property
    def has_width(self) -> bool:
        return self.width_left is not None and self.width_right is not None

    def _extras_at(self, s_new) -> dict:
        """Elevation / widths interpolated to new node positions."""
        out = {}
        for key in ("z", "width_left", "width_right"):
            val = getattr(self, key)
            if val is not None:
                out[key] = np.interp(s_new, self.s, val)
        return out

    @property
    def length(self) -> float:
        return float(self.s[-1])

    @property
    def ds(self) -> float:
        """Nominal (largest) step."""
        return float(np.diff(self.s).max())

    def interval_kappa(self) -> np.ndarray:
        if self.kappa_int is not None:
            return self.kappa_int
        return 0.5 * (self.kappa[1:] + self.kappa[:-1])

    @property
    def n_intervals(self) -> int:
        return len(self.s) - 1

    # ------------------------------------------------------------- builders
    @classmethod
    def from_segments(cls, segments: list[tuple[str, float, float, str]], name: str = "track",
                      closed: bool = True, ds: float = 0.5) -> "Track":
        """``segments``: (type, length_m, radius_m, direction) tuples."""
        bounds, curv = [0.0], []
        for kind, length, radius, direction in segments:
            if length <= 0:
                raise ValueError("segment length must be positive")
            if kind.lower().startswith("s"):
                k = 0.0
            else:
                if radius <= 0:
                    raise ValueError("corner radius must be positive")
                k = 1.0 / radius * (1.0 if direction.lower().startswith("l") else -1.0)
            bounds.append(bounds[-1] + length)
            curv.append(k)
        bounds_a = np.asarray(bounds)
        curv_a = np.asarray(curv)
        # Nodes on every segment boundary, each segment split into equal steps <= ds.
        pieces, k_int = [], []
        for i, k in enumerate(curv_a):
            n_i = max(1, int(math.ceil((bounds_a[i + 1] - bounds_a[i]) / ds - 1e-9)))
            pieces.append(np.linspace(bounds_a[i], bounds_a[i + 1], n_i + 1)[:-1])
            k_int.append(np.full(n_i, k))
        s = np.concatenate(pieces + [bounds_a[-1:]])
        kappa_int = np.concatenate(k_int)
        before = np.concatenate([[kappa_int[-1] if closed else kappa_int[0]], kappa_int])
        after = np.concatenate([kappa_int, [kappa_int[0] if closed else kappa_int[-1]]])
        kappa = np.where(np.abs(before) >= np.abs(after), before, after)  # tighter corner at boundaries
        x, y = _segment_xy(s, bounds_a, curv_a)
        return cls(name, s, kappa, closed, x, y, kappa_int, list(segments))

    @classmethod
    def from_curvature(cls, s_in, kappa_in, name: str = "track", closed: bool = True,
                       ds: float = 0.5) -> "Track":
        s_in = np.asarray(s_in, dtype=float)
        kappa_in = np.asarray(kappa_in, dtype=float)
        total = s_in[-1] - s_in[0]
        n = max(int(round(total / ds)), 2)
        s = np.linspace(0.0, total, n + 1)
        kappa = np.interp(s, s_in - s_in[0], kappa_in)
        return cls(name, s, kappa, closed)

    @classmethod
    def from_xy(cls, x, y, name: str = "track", closed: bool | None = None, ds: float = 0.5,
                cutoff_m: float = 8.0, method: str = "whittaker", smoothing: float = 0.05,
                z=None, width_left=None, width_right=None) -> "Track":
        """Centre line (or driven line) from coordinates.

        ``method="whittaker"`` (default): the points are resampled by arc length and smoothed with a
        Whittaker smoother that penalises the third difference (curvature rate), cut-off wavelength
        ``cutoff_m`` (transfer 0.5; shorter wiggles - point noise - are removed, longer ones such as
        a 15-20 m slalom are kept). Curvature then follows from finite differences. This is robust to
        centimetre-level point noise, which a plain interpolating spline turns into curvature spikes.
        ``method="spline"``: FITPACK smoothing spline, ``smoothing`` = expected point noise in metres.
        """
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        extra = {k: np.asarray(v, dtype=float) for k, v in (("z", z), ("width_left", width_left),
                                                            ("width_right", width_right)) if v is not None}
        if closed is None:
            seg = np.hypot(np.diff(x), np.diff(y))
            closed = math.hypot(x[-1] - x[0], y[-1] - y[0]) < 3.0 * max(np.median(seg), ds)
        if closed and math.hypot(x[-1] - x[0], y[-1] - y[0]) < 1e-6:
            x, y = x[:-1], y[:-1]
            extra = {k: v[:-1] for k, v in extra.items()}
        keep = np.concatenate([[True], np.hypot(np.diff(x), np.diff(y)) > 1e-9])
        x, y = x[keep], y[keep]
        extra = {k: v[keep] for k, v in extra.items()}
        # extra columns follow the line by arc-length fraction (smoothing changes the length slightly)
        xr, yr = (np.append(x, x[0]), np.append(y, y[0])) if closed else (x, y)
        f_raw = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xr), np.diff(yr)))])
        f_raw = f_raw / f_raw[-1]
        if closed:
            extra = {k: np.append(v, v[0]) for k, v in extra.items()}

        def with_extra(trk: "Track") -> "Track":
            f_out = trk.s / trk.s[-1]
            for k, v in extra.items():
                setattr(trk, k, np.interp(f_out, f_raw, v))
            return trk

        if method == "whittaker":
            s, kappa, xs, ys = _whittaker_line(x, y, bool(closed), ds, cutoff_m)
            return with_extra(cls(name, s, kappa, bool(closed), xs, ys))
        if method != "spline":
            raise ValueError("method must be 'whittaker' or 'spline'")
        from scipy.interpolate import splev, splprep

        tck, _ = splprep([x, y], s=len(x) * smoothing ** 2, per=1 if closed else 0, k=3)
        u_fine = np.linspace(0.0, 1.0, max(20 * len(x), 2000))
        xf, yf = splev(u_fine, tck)
        sf = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xf), np.diff(yf)))])
        total = sf[-1]
        n = max(int(round(total / ds)), 2)
        s = np.linspace(0.0, total, n + 1)
        u = np.interp(s, sf, u_fine)
        dx, dy = splev(u, tck, der=1)
        ddx, ddy = splev(u, tck, der=2)
        kappa = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-12)
        xs, ys = splev(u, tck)
        return with_extra(cls(name, s, np.asarray(kappa), bool(closed), np.asarray(xs), np.asarray(ys)))

    @classmethod
    def straight(cls, length: float, name: str = "straight", ds: float = 0.5) -> "Track":
        return cls.from_segments([("straight", length, 0.0, "")], name=name, closed=False, ds=ds)

    @classmethod
    def circle(cls, radius: float, name: str = "circle", direction: str = "right",
               ds: float = 0.25) -> "Track":
        return cls.from_segments([("corner", 2.0 * math.pi * radius, radius, direction)],
                                 name=name, closed=True, ds=ds)

    @classmethod
    def from_file(cls, path: str | Path, ds: float = 0.5, closed: bool | None = None) -> "Track":
        path = Path(path)
        if path.suffix.lower() == ".ai":  # Assetto Corsa AI spline (fast_lane.ai / ideal_line.ai)
            from .importers import track_from_ac

            return track_from_ac(path, closed=closed, ds=ds)
        if path.suffix.lower() == ".oltra":  # OptimumLap track
            from .importers import track_from_optimumlap

            return track_from_optimumlap(path, ds=ds, closed=closed)
        meta: dict[str, str] = {}
        lines = []
        with open(path, encoding="utf-8") as fh:
            for ln in fh:
                st = ln.strip()
                if not st:
                    continue
                if st.startswith("#"):
                    body = st.lstrip("#").strip()
                    if ":" in body:
                        k, v = body.split(":", 1)
                        meta[k.strip().lower()] = v.strip()
                    continue
                lines.append(ln)
        rows = list(csv.DictReader(lines))
        if not rows:
            raise ValueError(f"{path}: no track data")
        cols = {c.strip().lower(): c for c in rows[0].keys()}
        name = meta.get("name", path.stem)
        meta_closed = meta.get("closed")
        if closed is None and meta_closed is not None:
            closed = meta_closed.lower() in ("1", "true", "yes", "y")

        def col(key):
            return [r[cols[key]].strip() for r in rows]

        if "type" in cols:
            segs = []
            for r in rows:
                kind = r[cols["type"]].strip()
                length = float(r[cols["length_m"]])
                rad_txt = r.get(cols.get("radius_m", ""), "") if "radius_m" in cols else ""
                radius = float(rad_txt) if rad_txt and rad_txt.strip() else 0.0
                direction = r[cols["direction"]].strip() if "direction" in cols else "right"
                segs.append((kind, length, radius, direction))
            return _meta_width(cls.from_segments(segs, name=name, closed=True if closed is None else closed, ds=ds),
                               meta)
        if "x_m" in cols and "y_m" in cols:
            x = np.array(col("x_m"), dtype=float)
            y = np.array(col("y_m"), dtype=float)
            method = meta.get("smoothing_method", "whittaker").lower()

            def opt(key):
                return np.array(col(key), dtype=float) if key in cols else None

            out = cls.from_xy(x, y, name=name, closed=closed, ds=ds, method=method,
                              cutoff_m=float(meta.get("cutoff_m", 8.0)),
                              smoothing=float(meta.get("smoothing_m", 0.05)), z=opt("z_m"),
                              width_left=opt("width_left_m"), width_right=opt("width_right_m"))
            return _meta_width(out, meta)
        if "s_m" in cols:
            s = np.array(col("s_m"), dtype=float)
            if "curvature_1pm" in cols:
                kappa = np.array(col("curvature_1pm"), dtype=float)
            elif "radius_m" in cols:
                rad = np.array([float(v) if v else 0.0 for v in col("radius_m")])
                sign = np.ones_like(rad)
                if "direction" in cols:
                    sign = np.array([1.0 if d.lower().startswith("l") else -1.0 for d in col("direction")])
                kappa = np.where(rad > 0, sign / np.where(rad > 0, rad, 1.0), 0.0)
            else:
                raise ValueError(f"{path}: s_m needs a curvature_1pm or radius_m column")
            return cls.from_curvature(s, kappa, name=name, closed=True if closed is None else closed, ds=ds)
        raise ValueError(f"{path}: unrecognised track columns {list(rows[0].keys())}")

    # ------------------------------------------------------------- helpers
    def resampled(self, ds: float) -> "Track":
        if self.segments is not None:
            out = Track.from_segments(self.segments, self.name, self.closed, ds)
            for k, v in self._extras_at(out.s).items():
                setattr(out, k, v)
            out.use_grade, out.grade_smoothing_m = self.use_grade, self.grade_smoothing_m
            return out
        if abs(ds - self.ds) < 1e-9:
            return self
        n = max(int(round(self.length / ds)), 2)
        s = np.linspace(0.0, self.length, n + 1)
        out = Track(self.name, s, np.interp(s, self.s, self.kappa), self.closed, **self._extras_at(s))
        out.use_grade, out.grade_smoothing_m = self.use_grade, self.grade_smoothing_m
        if self.x is not None and self.y is not None:
            out.x = np.interp(s, self.s, self.x)
            out.y = np.interp(s, self.s, self.y)
        return out

    def with_width(self, total_m: float | None = None, left_m: float | None = None,
                   right_m: float | None = None) -> "Track":
        """Copy with a constant track width (``total_m`` split evenly, or left / right distances)."""
        import copy as _copy

        out = _copy.copy(self)
        if total_m is not None:
            left_m = right_m = 0.5 * float(total_m)
        out.width_left = np.full(len(self.s), float(left_m))
        out.width_right = np.full(len(self.s), float(right_m))
        return out

    def smoothed(self, length: float, ds: float = 0.5) -> "Track":
        """Uniformly sampled copy whose curvature is a moving average over ``length`` metres.

        Used by the transient solver: a real driver needs a finite distance to turn in, so
        curvature steps of a segment track become ramps (periodic for closed tracks).
        """
        n = max(int(round(self.length / ds)), 2)
        s = np.linspace(0.0, self.length, n + 1)
        k_int = self.interval_kappa()
        mid = 0.5 * (s[1:] + s[:-1])
        idx = np.clip(np.searchsorted(self.s, mid, side="right") - 1, 0, len(k_int) - 1)
        k = k_int[idx]  # interval curvature on the uniform grid
        w = max(1, int(round(length / (self.length / n))))
        if w > 1:
            kernel = np.ones(w) / w
            if self.closed:
                k = np.convolve(np.concatenate([k[-w:], k, k[:w]]), kernel, mode="same")[w:-w]
            else:
                pad = np.concatenate([np.full(w, k[0]), k, np.full(w, k[-1])])
                k = np.convolve(pad, kernel, mode="same")[w:-w]
        before = np.concatenate([[k[-1] if self.closed else k[0]], k])
        after = np.concatenate([k, [k[0] if self.closed else k[-1]]])
        kappa = 0.5 * (before + after)
        out = Track(self.name, s, kappa, self.closed, kappa_int=k, **self._extras_at(s))
        out.use_grade, out.grade_smoothing_m = self.use_grade, self.grade_smoothing_m
        x, y = self.xy()
        out.x = np.interp(s, self.s, x)
        out.y = np.interp(s, self.s, y)
        return out

    def xy(self) -> tuple[np.ndarray, np.ndarray]:
        """Centre-line coordinates (integrated from curvature when not given)."""
        if self.x is not None and self.y is not None:
            return self.x, self.y
        ds = np.diff(self.s)
        k_mid = 0.5 * (self.kappa[1:] + self.kappa[:-1])
        heading = np.concatenate([[0.0], np.cumsum(k_mid * ds)])
        h_mid = 0.5 * (heading[1:] + heading[:-1])
        x = np.concatenate([[0.0], np.cumsum(np.cos(h_mid) * ds)])
        y = np.concatenate([[0.0], np.cumsum(np.sin(h_mid) * ds)])
        return x, y


def _meta_width(track: Track, meta: dict) -> Track:
    """``# width_m: 4.0`` in a track CSV header: constant track width (unless the file has width columns);
    ``# grade: off`` ignores the elevation; ``# grade_smoothing_m: 10`` smooths it before the slope."""
    w = meta.get("width_m")
    if w and not track.has_width:
        track = track.with_width(float(w))
    if str(meta.get("grade", "on")).strip().lower() in ("off", "false", "no", "0"):
        track.use_grade = False
    if meta.get("grade_smoothing_m"):
        track.grade_smoothing_m = float(meta["grade_smoothing_m"])
    return track


def _segment_xy(s: np.ndarray, bounds: np.ndarray, curv: np.ndarray):
    """Exact centre-line coordinates of a straight/arc segment track at positions ``s``."""
    n_seg = len(curv)
    x0 = np.zeros(n_seg)
    y0 = np.zeros(n_seg)
    h0 = np.zeros(n_seg)
    x = y = h = 0.0
    for i in range(n_seg):
        x0[i], y0[i], h0[i] = x, y, h
        x, y, h = _advance(x, y, h, curv[i], bounds[i + 1] - bounds[i])
    idx = np.clip(np.searchsorted(bounds, s, side="right") - 1, 0, n_seg - 1)
    out = np.array([_advance(x0[k], y0[k], h0[k], curv[k], si - bounds[k]) for k, si in zip(idx, s)])
    return out[:, 0], out[:, 1]


def _advance(x: float, y: float, h: float, k: float, d: float):
    if abs(k) < 1e-12:
        return x + d * math.cos(h), y + d * math.sin(h), h
    h2 = h + k * d
    return x + (math.sin(h2) - math.sin(h)) / k, y - (math.cos(h2) - math.cos(h)) / k, h2


def _whittaker_line(x, y, closed: bool, ds: float, cutoff_m: float):
    """Arc-length resampling + Whittaker smoothing (3rd differences) -> s, curvature, x, y."""
    import scipy.sparse as sps
    from scipy.sparse.linalg import spsolve

    if closed:
        x, y = np.append(x, x[0]), np.append(y, y[0])
    sp = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    total = sp[-1]
    h_target = min(ds, 0.5)
    n = max(int(round(total / h_target)), 8)
    su = np.linspace(0.0, total, n + 1)
    h = total / n
    xu, yu = np.interp(su, sp, x), np.interp(su, sp, y)
    m = n if closed else n + 1
    xu, yu = xu[:m], yu[:m]
    order = 3
    coef = np.array([-1.0, 3.0, -3.0, 1.0])
    if cutoff_m > 0:
        lam = (cutoff_m / (2.0 * math.pi * h)) ** (2 * order)
        r = m if closed else m - order
        rows = np.repeat(np.arange(r), order + 1)
        cols = np.arange(r)[:, None] + np.arange(order + 1)
        if closed:
            cols = cols % m
        D = sps.csr_matrix((np.tile(coef, r), (rows, cols.ravel())), shape=(r, m))
        A = (sps.identity(m, format="csc") + lam * (D.T @ D)).tocsc()
        xu, yu = spsolve(A, xu), spsolve(A, yu)
    if closed:
        xe, ye = np.concatenate([xu[-1:], xu, xu[:1]]), np.concatenate([yu[-1:], yu, yu[:1]])
        dx, dy = (xe[2:] - xe[:-2]) / (2 * h), (ye[2:] - ye[:-2]) / (2 * h)
        ddx = (xe[2:] - 2 * xe[1:-1] + xe[:-2]) / h ** 2
        ddy = (ye[2:] - 2 * ye[1:-1] + ye[:-2]) / h ** 2
    else:
        dx, dy = np.gradient(xu, h), np.gradient(yu, h)
        ddx, ddy = np.gradient(dx, h), np.gradient(dy, h)
    kappa = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-12)
    if closed:
        kappa, xu, yu = np.append(kappa, kappa[0]), np.append(xu, xu[0]), np.append(yu, yu[0])
    # the smoothed line is slightly shorter than the noisy polyline: use its own length
    s_sm = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xu), np.diff(yu)))])
    n_out = max(int(round(s_sm[-1] / ds)), 2)
    s_out = np.linspace(0.0, s_sm[-1], n_out + 1)
    return (s_out, np.interp(s_out, s_sm, kappa), np.interp(s_out, s_sm, xu), np.interp(s_out, s_sm, yu))
