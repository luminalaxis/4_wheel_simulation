"""Track / vehicle importers: Assetto Corsa AI splines (synthetic and the real FSK 2026 file) and
OptimumLap tracks / vehicles (real files in ``data/import``)."""

import math
import struct
from pathlib import Path

import numpy as np
import pytest

from aerolap.cli import main
from aerolap.config import load_vehicle_config
from aerolap.features import features_for_level
from aerolap.analysis.ol_compare import compare_with_optimumlap
from aerolap.config import load_yaml
from aerolap.importers import (read_ac_ai, read_optimumlap_result, read_optimumlap_track, read_optimumlap_vehicle,
                               track_from_ac, track_from_optimumlap, vehicle_cfg_from_optimumlap)
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


def test_smoothed_optimumlap_line_stays_on_the_track(tmp_path):
    out = tmp_path / "smooth.csv"
    assert main(["import-track", str(OLTRA), "--smooth", "15", "--out", str(out)]) == 0
    sm, raw = Track.from_file(out), track_from_optimumlap(OLTRA)
    assert sm.closed and sm.length == pytest.approx(raw.length, rel=0.02)
    assert 1.0 / np.abs(sm.kappa).max() > 1.0 / np.abs(raw.kappa).max()


def load_yaml_keys(path):
    import yaml

    return set(yaml.safe_load(Path(path).read_text()))


# ------------------------------------------------------------------ OptimumLap result (.OLRes)
OLRES = Path(__file__).resolve().parents[1] / "data" / "import" / "fsk2026_v6_fd4_diagnostic.OLRes"
OL_SWEEP = OLRES.with_name("fsk2026_v6_final_drive_sweep_optimumlap.csv")
BASE = Path(__file__).resolve().parents[1] / "data" / "vehicles" / "fsae_ev_aero.yaml"


def test_read_optimumlap_result():
    res = read_optimumlap_result(OLRES)
    assert res.track.name == "FSK2026_GiantCorners_Smoothed_V6_FIXED_851m" and res.track.closed
    assert len(res.track.lengths) == 211 and res.track.length == pytest.approx(851.0)
    assert res.vehicle.name == "MIP_228MV_FD4_Diagnostic" and res.vehicle.get("latfriction") == pytest.approx(1.3)
    assert res.lap_time == pytest.approx(70.15791, abs=1e-4)
    ch = res.channels
    assert len(ch["speed"]) == 8510 and ch["elapsedTime"][-1] == pytest.approx(res.lap_time)
    m, g = res.vehicle.get("mass"), 9.81
    assert np.allclose(ch["fz"], m * g + ch["downForce"])  # one point mass: weight + downforce


def test_optimumlap_drives_blended_corner_radii():
    """A corner followed by a corner in the same direction: OptimumLap's 'Corner Radius' channel
    changes linearly to the next radius (checked at every sample away from the segment ends)."""
    res = read_optimumlap_result(OLRES)
    segs = res.track.segments()
    bounds = np.concatenate([[0.0], np.cumsum([sg[1] for sg in segs])])
    s, r_ol = res.channels["elapsedDistance"], res.channels["cornerRadius"]
    i = np.clip(np.searchsorted(bounds, s, side="right") - 1, 0, len(segs) - 1)
    f = (s - bounds[i]) / np.array([segs[k][1] for k in i])
    inside = (f > 0.05) & (f < 0.95)
    r_model = np.array([0.0 if sg[0] == "straight" else
                        (1 if sg[3] == "left" else -1) * (sg[2] + ((sg[4] - sg[2]) * fk if len(sg) > 4 else 0.0))
                        for sg, fk in zip((segs[k] for k in i), f)])
    # the channel is sampled at the end of each 0.1 m step
    step = np.array([0.0 if len(segs[k]) < 5 else (segs[k][4] - segs[k][2]) * 0.1 / segs[k][1] for k in i])
    r_model = r_model + np.sign(r_model) * step
    assert sum(len(sg) > 4 for sg in segs) == 133
    assert np.max(np.abs(r_model - r_ol)[inside]) < 1e-4


def test_l0_matches_the_optimumlap_result():
    res = read_optimumlap_result(OLRES)
    cmp = compare_with_optimumlap(res, load_yaml(BASE))  # ds 0.1 m like OptimumLap
    assert abs(cmp.lap_diff_pct) < 1.0  # -0.4 % (OptimumLap: 70.158 s)
    rows = {r[0]: r for r in cmp.rows}
    for key in ("lowest speed", "highest speed", "max lateral acceleration", "energy, OptimumLap definition"):
        assert rows[key][2] == pytest.approx(rows[key][1], rel=0.005)
    # with constant radii (how aerolap read OptimumLap tracks before) the lap is ~3 % slow
    flat = compare_with_optimumlap(res, load_yaml(BASE), opts=SolverOptions(ds=0.25), transitions=False)
    assert flat.lap_diff_pct > 2.0


def test_l0_final_drive_sweep_matches_optimumlap():
    rows = {}
    for line in OL_SWEEP.read_text(encoding="utf-8").splitlines():
        p = line.rstrip(",").split(",")
        if len(p) > 5:
            rows[p[0]] = p[1:]
    fd = [float(x) for x in rows["Final Drive Ratio [-]"]]
    lap = [float(x) for x in rows["Lap time [s]"]]
    assert float(rows["Lateral Friction [-]"][0]) == 1.5 and float(rows["Drivetrain Efficiency [%]"][0]) == 90
    ol = read_optimumlap_result(OLRES)
    ol.vehicle.props.scalars.update(longfriction=1.5, latfriction=1.5, driveefficiency=0.9)
    trk = Track.from_segments(ol.track.segments(), closed=True)
    out = []
    for k in (0, 10, 20):  # final drive 2.0, 4.0, 6.0
        ol.vehicle.props.scalars["finaldriveratio"] = fd[k]
        raw, *_ = vehicle_cfg_from_optimumlap(ol.vehicle, load_yaml(BASE))
        from aerolap.config import vehicle_config_from_dict

        car = VehicleModel(vehicle_config_from_dict(raw), features_for_level("L0"))
        out.append(QSSSolver(SolverOptions(ds=0.25)).solve(car, trk, start="flying").lap_time / lap[k] - 1.0)
    assert max(abs(x) for x in out) < 0.01
    assert max(out) - min(out) < 0.002  # the same offset at every ratio: the final-drive trend is reproduced


def test_linear_radius_segment():
    r0, r1, length = 10.0, 30.0, 6.0
    tr = Track.from_segments([("straight", 20.0, 0.0, ""), ("corner", length, r0, "left", r1),
                              ("straight", 20.0, 0.0, "")], closed=False, ds=0.5)
    turned = np.sum(tr.interval_kappa() * np.diff(tr.s))
    assert turned == pytest.approx(length * math.log(r1 / r0) / (r1 - r0), rel=1e-9)  # exact mean of 1/r
    i0 = np.argmin(np.abs(tr.s - 20.0))
    assert tr.kappa[i0] == pytest.approx(1 / r0)  # corner entry node: start radius
    mid = np.argmin(np.abs(tr.s - 23.0))
    assert tr.kappa[mid] == pytest.approx(1 / 20.0)


def test_olres_track_import_cli(tmp_path):
    out, rev = tmp_path / "v6.csv", tmp_path / "v6_rev.csv"
    assert main(["import-track", str(OLRES), "--out", str(out)]) == 0
    assert "radius_end_m" in out.read_text().splitlines()[3]
    a = Track.from_file(out)
    b = track_from_optimumlap(OLRES)
    assert a.length == pytest.approx(851.0) and np.allclose(a.kappa, b.kappa, atol=1e-5)
    x, y = a.xy()
    assert math.hypot(x[-1] - x[0], y[-1] - y[0]) < 1e-3  # 6-decimal CSV; drawn like OptimumLap's map: closes
    assert main(["import-track", str(OLRES), "--reverse", "--out", str(rev)]) == 0
    r = Track.from_file(rev)
    assert np.allclose(np.sort(np.abs(r.interval_kappa())), np.sort(np.abs(a.interval_kappa())), atol=1e-5)


def test_compare_optimumlap_cli(tmp_path):
    assert main(["compare-optimumlap", str(OLRES), "--ds", "0.5", "--out", str(tmp_path)]) == 0
    text = (tmp_path / "ol_compare.txt").read_text()
    assert "lap time" in text and (tmp_path / "ol_compare.png").exists()
