"""Track / vehicle importers: Assetto Corsa AI splines (synthetic and the real FSK 2026 file) and
OptimumLap tracks / vehicles (real files in ``data/import``)."""

import math
import struct
from pathlib import Path

import numpy as np
import pytest

from aerolap.cli import main
from aerolap.config import ConfigError, load_vehicle_config
from aerolap.features import features_for_level
from aerolap.importers import (read_ac_ai, read_optimumlap_track, read_optimumlap_vehicle, track_from_ac,
                               track_from_optimumlap)
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


IMPORT = Path(__file__).resolve().parents[1] / "data" / "import"
OLTRA = IMPORT / "FSK2026_GiantCorners_ConeMap_V4_851m.oltra"
OLVEH = IMPORT / "fsk2026_4.0.olveh"
ROOT = IMPORT.parents[1]
AC_FSK = IMPORT / "fsk2026_fast_lane.ai"


def test_real_assetto_corsa_file():
    sp = read_ac_ai(AC_FSK)
    assert sp.version == 7 and sp.n == 872
    assert np.median(np.hypot(np.diff(sp.x), np.diff(sp.y))) == pytest.approx(1.0, abs=0.01)
    assert np.median(sp.side_left + sp.side_right) == pytest.approx(5.5, abs=0.2)
    tr = Track.from_file(AC_FSK)
    assert tr.closed and tr.length == pytest.approx(869.0, abs=2.0)
    heading = np.sum(0.5 * (tr.kappa[1:] + tr.kappa[:-1]) * np.diff(tr.s))
    assert math.degrees(heading) == pytest.approx(360.0, abs=1.0)


def test_real_optimumlap_track():
    ol = read_optimumlap_track(OLTRA)
    assert ol.name == "FSK2026_GiantCorners_ConeMap_V4_851m" and ol.closed
    assert len(ol.lengths) == 535 and ol.length == pytest.approx(851.0, abs=1e-6)
    assert set(np.unique(ol.directions)) <= {-1.0, 0.0, 1.0}
    left = ol.lengths[ol.directions > 0].sum() / ol.length
    assert left == pytest.approx(ol.props.scalars["percentleft"], abs=1e-9)  # +1 = left
    assert ol.radii[ol.directions != 0].min() == pytest.approx(ol.props.scalars["mincornerradius"])
    tr = track_from_optimumlap(OLTRA)
    x, y = tr.xy()
    assert math.hypot(x[-1] - x[0], y[-1] - y[0]) < 1e-6  # the segment list closes exactly
    assert Track.from_file(OLTRA).length == pytest.approx(851.0)


def test_real_optimumlap_vehicle():
    v = read_optimumlap_vehicle(OLVEH)
    assert v.name == "4.0" and v.transmission == "Sequential Gearbox"
    assert v.get("mass") == pytest.approx(310.0) and v.get("tireradius") == pytest.approx(0.232)
    rpm, tq = v.torque_curve
    assert len(rpm) == 27 and tq.max() == pytest.approx(220.0) and v.rev_limit_rpm == pytest.approx(6500.0)
    assert (tq * rpm).max() * 2 * math.pi / 60 == pytest.approx(80e3, rel=0.01)  # 80 kW at the shaft
    ratio = v.get("finaldriveratio") * v.gear_ratios[-1]
    v_top = v.rev_limit_rpm * 2 * math.pi / 60 / ratio * v.get("tireradius")
    assert v_top == pytest.approx(v.get("topSpeed"), rel=1e-6)


def test_optimumlap_import_cli_and_l0_lap(tmp_path):
    veh, trk = tmp_path / "car.yaml", tmp_path / "track.csv"
    assert main(["import-vehicle", str(OLVEH), "--out", str(veh)]) == 0
    assert main(["import-track", str(OLTRA), "--out", str(trk)]) == 0
    cfg = load_vehicle_config(veh)
    assert cfg["mass"]["total_kg"] == 310.0 and cfg["aero"]["type"] == "constant"
    assert cfg["powertrain"]["ev"]["reduction_ratio"] == 4.0 and "battery" not in load_yaml_keys(veh)
    car = VehicleModel(cfg, features_for_level("L0"))
    solver = QSSSolver(SolverOptions.coarse())
    t_csv = solver.solve(car, Track.from_file(trk), start="flying").lap_time
    t_ol = solver.solve(car, Track.from_file(OLTRA), start="flying").lap_time
    assert t_csv == pytest.approx(t_ol, rel=1e-4)  # CSV keeps 6 decimals
    assert 60.0 < t_csv < 80.0
    full = tmp_path / "full.yaml"
    assert main(["import-vehicle", str(OLVEH), "--full", "--out", str(full)]) == 0
    fc = load_vehicle_config(full)
    assert fc["powertrain"]["ev"]["motor"] == "emrax_228" and fc["battery"]["capacity_kWh"] is not None
    assert "ASSUMED" in full.read_text()


def _strip(cfg):
    return {k: v for k, v in cfg.items() if not k.startswith("_") and k != "name"}


def test_olveh_used_directly_as_the_vehicle(tmp_path):
    """An .olveh path loads like a YAML and equals the 'import-vehicle' result."""
    veh = tmp_path / "car.yaml"
    assert main(["import-vehicle", str(OLVEH), "--out", str(veh)]) == 0
    direct = load_vehicle_config(OLVEH, warn=False)
    assert _strip(direct) == _strip(load_vehicle_config(veh, warn=False))
    assert any("ASSUMED" in n for n in direct["_notes"])
    # a <file>.olveh.yaml next to it holds options and overrides of the assumed values
    car = tmp_path / "team.olveh"
    car.write_bytes(OLVEH.read_bytes())
    (tmp_path / "team.olveh.yaml").write_text("mass:\n  cg_height_mm: 300\n", encoding="utf-8")
    side = load_vehicle_config(car, warn=False)
    assert side["mass"]["cg_height_mm"] == 300 and side["mass"]["total_kg"] == 310.0
    assert side["name"].startswith("4.0")


def test_yaml_linked_to_an_olveh(tmp_path):
    """optimumlap: in a YAML reads the chosen groups from the file and keeps the rest of the YAML."""
    linked = load_vehicle_config(ROOT / "data" / "vehicles" / "fsk2026_olveh_link.yaml", warn=False)
    ref = load_vehicle_config(ROOT / "data" / "vehicles" / "fsk2026.yaml", warn=False)
    assert _strip(linked) == _strip(ref)  # fsk2026.yaml was imported from the same file
    raw = OLVEH.read_bytes().replace(struct.pack("<d", 310.0), struct.pack("<d", 295.0), 1)
    (tmp_path / "car.olveh").write_bytes(raw)
    (tmp_path / "car.yaml").write_text(
        f"optimumlap:\n  file: car.olveh\n  take: [mass]\n  base: {ROOT / 'data' / 'vehicles' / 'fsk2026.yaml'}\n"
        "mass:\n  cg_height_mm: 280\n", encoding="utf-8")
    cfg = load_vehicle_config(tmp_path / "car.yaml", warn=False)
    assert cfg["mass"]["total_kg"] == 295.0 and cfg["mass"]["cg_height_mm"] == 280
    assert cfg["aero"] == ref["aero"] and cfg["powertrain"] == ref["powertrain"]
    (tmp_path / "bad.yaml").write_text("optimumlap:\n  file: car.olveh\n  take: [wings]\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="take"):
        load_vehicle_config(tmp_path / "bad.yaml", warn=False)


def test_smoothed_optimumlap_line_stays_on_the_track(tmp_path):
    out = tmp_path / "smooth.csv"
    assert main(["import-track", str(OLTRA), "--smooth", "15", "--out", str(out)]) == 0
    sm, raw = Track.from_file(out), track_from_optimumlap(OLTRA)
    assert sm.closed and sm.length == pytest.approx(raw.length, rel=0.02)
    assert 1.0 / np.abs(sm.kappa).max() > 1.0 / np.abs(raw.kappa).max()


def load_yaml_keys(path):
    import yaml

    return set(yaml.safe_load(Path(path).read_text()))
