"""Track importers for files from other tools.

Assetto Corsa AI spline (``<track>/ai/fast_lane.ai``, also ``ideal_line.ai``)
-------------------------------------------------------------------------------
Binary, little-endian (layout as read by Content Manager / AcTools ``AiSpline``)::

    int32 version, int32 n_points, int32 lap_time, int32 sample_count
    n_points x { float32 x, y, z;  float32 length;  int32 id }                     20 bytes
    int32 n_extra (== n_points)
    n_extra x { float32 speed, gas, brake, obsolete_lat_g, radius, side_left, side_right,
                camber, direction, normal[3], length, forward[3], tag, grade }      72 bytes
    (a grid section may follow - ignored)

Assetto Corsa is y-up; the plan view is (x, -z) (``map.png`` is drawn with x to the right and
z downwards). ``fast_lane.ai`` is the AI racing line, i.e. the path a driver actually takes, which
is what a lap simulation should follow. The extra block (track edges ``side_left/right``) is
optional and only used for information. Elevation is read (``z_m`` / grade) but the lap
solvers do not use it yet.

The layout above is the community-documented one; the reader checks it against the file size
and fails with a clear message when a file does not match.
"""

from __future__ import annotations

import csv
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .track import Track

_POINT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("length", "<f4"), ("id", "<i4")])
_EXTRA_FIELDS = ["speed", "gas", "brake", "obsolete_lat_g", "radius", "side_left", "side_right", "camber",
                 "direction", "normal_x", "normal_y", "normal_z", "length", "forward_x", "forward_y",
                 "forward_z", "tag", "grade"]
_EXTRA = np.dtype([(f, "<f4") for f in _EXTRA_FIELDS])


@dataclass
class AcSpline:
    version: int
    x: np.ndarray  # plan view, m
    y: np.ndarray  # plan view, m (= -z of Assetto Corsa)
    elevation: np.ndarray  # m (Assetto Corsa y)
    length: np.ndarray  # cumulative distance stored in the file, m
    side_left: np.ndarray | None = None  # distance to the track edges, m
    side_right: np.ndarray | None = None
    lap_time_ms: int = 0

    @property
    def n(self) -> int:
        return len(self.x)


def read_ac_ai(path: str | Path) -> AcSpline:
    path = Path(path)
    raw = path.read_bytes()
    if len(raw) < 16:
        raise ValueError(f"{path}: too short for an Assetto Corsa AI spline")
    version, n, lap_time, _samples = struct.unpack_from("<4i", raw, 0)
    end_points = 16 + n * _POINT.itemsize
    if not (0 < version < 100) or n < 3 or end_points > len(raw):
        raise ValueError(f"{path}: not an Assetto Corsa AI spline (version {version}, {n} points, "
                         f"{len(raw)} bytes)")
    pts = np.frombuffer(raw, dtype=_POINT, count=n, offset=16)
    x = pts["x"].astype(float)
    elev = pts["y"].astype(float)
    y = -pts["z"].astype(float)
    step = np.hypot(np.diff(x), np.diff(y))
    if not np.all(np.isfinite(step)) or np.median(step) <= 0.01 or np.median(step) > 50.0:
        raise ValueError(f"{path}: point spacing (median {np.median(step):.3g} m) does not look like an AI line")
    out = AcSpline(version, x, y, elev, pts["length"].astype(float), lap_time_ms=lap_time)
    off = end_points
    if off + 4 <= len(raw):
        (n_extra,) = struct.unpack_from("<i", raw, off)
        if n_extra == n and off + 4 + n * _EXTRA.itemsize <= len(raw):
            ex = np.frombuffer(raw, dtype=_EXTRA, count=n, offset=off + 4)
            left, right = ex["side_left"].astype(float), ex["side_right"].astype(float)
            if np.all(np.isfinite(left)) and np.all(np.isfinite(right)) and 0 < np.median(left + right) < 100:
                out.side_left, out.side_right = left, right
    return out


def track_from_ac(path: str | Path, name: str | None = None, closed: bool | None = None, ds: float = 0.5,
                  cutoff_m: float = 10.0, reverse: bool = False) -> Track:
    """Track from an Assetto Corsa AI spline (``cutoff_m``: smoothing wavelength, see ``Track.from_xy``)."""
    sp = read_ac_ai(path)
    x, y = sp.x, sp.y
    if reverse:
        x, y = x[::-1], y[::-1]
    return Track.from_xy(x, y, name=name or Path(path).parent.parent.name or Path(path).stem,
                         closed=closed, ds=ds, cutoff_m=cutoff_m)


def write_xy_csv(sp: AcSpline, path: str | Path, name: str, closed: bool, cutoff_m: float) -> Path:
    """Centre-line CSV readable by ``Track.from_file`` (extra columns are kept for reference)."""
    path = Path(path)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        fh.write(f"# name: {name}\n# closed: {'true' if closed else 'false'}\n# cutoff_m: {cutoff_m:g}\n")
        fh.write("# source: Assetto Corsa AI spline, plan view (x, -z)\n")
        w = csv.writer(fh)
        cols = ["x_m", "y_m", "z_m"] + (["width_left_m", "width_right_m"] if sp.side_left is not None else [])
        w.writerow(cols)
        for i in range(sp.n):
            row = [f"{sp.x[i]:.3f}", f"{sp.y[i]:.3f}", f"{sp.elevation[i]:.3f}"]
            if sp.side_left is not None:
                row += [f"{sp.side_left[i]:.3f}", f"{sp.side_right[i]:.3f}"]
            w.writerow(row)
    return path
