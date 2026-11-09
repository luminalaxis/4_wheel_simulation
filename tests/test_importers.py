"""Assetto Corsa AI spline import, checked on a synthetic file written in the same binary layout."""

import struct

import numpy as np
import pytest

from aerolap.cli import main
from aerolap.importers import read_ac_ai, track_from_ac
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

R, L_STRAIGHT, L_RAMP = 20.0, 100.0, 10.0


def _profile(h=0.01):
    """Stadium with clothoid corner entries / exits (like a driven line): s, curvature (left +)."""
    arc = np.pi * R - L_RAMP
    pieces = [(L_STRAIGHT, 0.0, 0.0), (L_RAMP, 0.0, 1 / R), (arc, 1 / R, 1 / R), (L_RAMP, 1 / R, 0.0)] * 2
    s_all, k_all, s0 = [], [], 0.0
    for length, k0, k1 in pieces:
        s = np.arange(0.0, length, h)
        s_all.append(s0 + s)
        k_all.append(k0 + (k1 - k0) * s / length)
        s0 += length
    return np.concatenate(s_all + [[s0]]), np.concatenate(k_all + [[0.0]])


def _stadium(step=1.0, noise=0.01, seed=0):
    """Counter-clockwise points every ``step`` m with Gaussian position noise."""
    s, k = _profile()
    head = np.concatenate([[0.0], np.cumsum(0.5 * (k[1:] + k[:-1]) * np.diff(s))])
    hm = 0.5 * (head[1:] + head[:-1])
    x = np.concatenate([[0.0], np.cumsum(np.cos(hm) * np.diff(s))])
    y = np.concatenate([[0.0], np.cumsum(np.sin(hm) * np.diff(s))])
    idx = np.searchsorted(s, np.arange(0.0, s[-1] - 0.5 * step, step))
    rng = np.random.default_rng(seed)
    return s[idx], x[idx] + rng.normal(0, noise, idx.size), y[idx] + rng.normal(0, noise, idx.size)


LENGTH = 2 * L_STRAIGHT + 2 * (np.pi * R + L_RAMP)


def _write_ai(path, s, x, y, extras=True):
    n = len(s)
    buf = bytearray(struct.pack("<4i", 7, n, 45000, n))
    for i in range(n):  # Assetto Corsa: y up, plan (x, -z)
        buf += struct.pack("<4fi", x[i], 0.01 * i, -y[i], s[i], i)
    if extras:
        buf += struct.pack("<i", n)
        for i in range(n):
            vals = [20.0, 1.0, 0.0, 0.0, R, 4.0, 3.5, 0.0, 0.0, 0.0, 1.0, 0.0, s[i], 1.0, 0.0, 0.0, 0.0, 0.0]
            buf += struct.pack("<18f", *vals)
    path.write_bytes(bytes(buf))


def test_read_ac_spline(tmp_path):
    s, x, y = _stadium()
    f = tmp_path / "fast_lane.ai"
    _write_ai(f, s, x, y)
    sp = read_ac_ai(f)
    assert sp.version == 7 and sp.n == len(s) and sp.lap_time_ms == 45000
    np.testing.assert_allclose(sp.x, x, atol=1e-4)
    np.testing.assert_allclose(sp.y, y, atol=1e-4)
    np.testing.assert_allclose(sp.side_left + sp.side_right, 7.5)
    _write_ai(f, s, x, y, extras=False)
    assert read_ac_ai(f).side_left is None
    bad = tmp_path / "bad.ai"
    bad.write_bytes(struct.pack("<4i", 7, 100000, 0, 0) + b"\0" * 64)
    with pytest.raises(ValueError, match="not an Assetto Corsa"):
        read_ac_ai(bad)


def test_ac_track_matches_the_geometry(tmp_path, ev_cfg):
    s, x, y = _stadium()
    f = tmp_path / "ai" / "fast_lane.ai"
    f.parent.mkdir()
    _write_ai(f, s, x, y)
    tr = Track.from_file(f)
    assert tr.closed
    assert tr.length == pytest.approx(LENGTH, rel=0.01)
    corner = np.abs(tr.kappa) > 0.5 / R
    assert np.median(tr.kappa[corner]) == pytest.approx(1.0 / R, rel=0.05)  # left-hand: positive
    ref = Track.from_curvature(*_profile(), closed=True)
    car = VehicleModel(ev_cfg)
    solver = QSSSolver(SolverOptions.coarse())
    t_ai = solver.solve(car, tr, start="flying").lap_time
    t_ref = solver.solve(car, ref, start="flying").lap_time
    assert t_ai == pytest.approx(t_ref, rel=0.005)
    rev = track_from_ac(f, reverse=True)
    assert np.median(rev.kappa[np.abs(rev.kappa) > 0.5 / R]) < 0  # clockwise when reversed


def test_import_track_cli(tmp_path):
    s, x, y = _stadium()
    f = tmp_path / "fast_lane.ai"
    _write_ai(f, s, x, y)
    out = tmp_path / "stadium.csv"
    assert main(["import-track", str(f), "--out", str(out), "--name", "stadium"]) == 0
    tr = Track.from_file(out)
    assert tr.name == "stadium" and tr.closed
    assert tr.length == pytest.approx(LENGTH, rel=0.01)
