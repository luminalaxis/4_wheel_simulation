import math

import numpy as np
import pytest

from aerolap.analysis.energy import energy_breakdown
from aerolap.analysis.energy_strategy import LapDP
from aerolap.config import with_overrides
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.solvers.transient import TransientOptions, TransientSolver
from aerolap.track import Track
from aerolap.vehicle import G, VehicleModel

from .conftest import DATA


def _straight(grade: float, length: float = 300.0) -> Track:
    x = np.linspace(0.0, length, 301)
    return Track.from_xy(x, np.zeros_like(x), name=f"straight {grade:+.0%}", closed=False, ds=0.5, cutoff_m=2.0,
                         z=grade * x)


def _hill_ring(amp: float = 2.0, radius: float = 60.0) -> Track:
    th = np.linspace(0.0, 2.0 * math.pi, 721)[:-1]
    return Track.from_xy(radius * np.cos(th), radius * np.sin(th), name="hill ring", closed=True, ds=0.5,
                         cutoff_m=4.0, z=amp * np.sin(th))


def test_interval_grade(simple_cfg):
    up = _straight(0.05)
    g = up.interval_grade()
    assert g is not None and np.allclose(g, 0.05 / math.sqrt(1.0 + 0.05 ** 2), atol=1e-6)
    flat = Track.from_xy(np.linspace(0, 100, 101), np.zeros(101), closed=False, z=np.zeros(101))
    assert flat.interval_grade() is None  # all zero: treated as flat
    off = _straight(0.05)
    off.use_grade = False
    assert off.interval_grade() is None


def test_uphill_costs_acceleration_by_g_sin_theta(simple_cfg):
    car = VehicleModel(simple_cfg)
    solver = QSSSolver()
    flat = solver.solve(car, _straight(0.0), start="standing")
    up = solver.solve(car, _straight(0.05), start="standing")
    down = solver.solve(car, _straight(-0.05), start="standing")
    assert down.lap_time < flat.lap_time < up.lap_time
    # traction-limited launch: the tyres give the same force, the slope takes g sin(theta)
    i = 4
    assert flat.ax[i] - up.ax[i] == pytest.approx(G * 0.05 / math.sqrt(1.0025), rel=0.03)
    assert "grade_pct" in up.channels and up.channels["grade_pct"][10] == pytest.approx(5.0, rel=1e-3)
    # force balance on the slope: F_drive = m (a_path + g sin(theta)) + drag + rolling resistance
    ch = up.channels
    fz = ch["fz_fl_N"] + ch["fz_fr_N"] + ch["fz_rl_N"] + ch["fz_rr_N"]
    for j in (20, 200):
        f = car.m * (up.ax[j] + G * 0.05 / math.sqrt(1.0025)) + ch["drag_N"][j] + car.tire.rolling_resistance * fz[j]
        assert ch["drive_force_N"][j] == pytest.approx(f, rel=1e-6)


def test_braking_is_shorter_uphill(simple_cfg):
    car = VehicleModel(simple_cfg)
    x = np.linspace(0.0, 200.0, 201)
    y = np.where(x > 150.0, -60.0 + np.sqrt(np.maximum(60.0 ** 2 - (x - 150.0) ** 2, 0.0)), 0.0)  # straight into a bend
    times = {}
    for grade in (-0.06, 0.0, 0.06):
        trk = Track.from_xy(x, y, closed=False, ds=0.5, cutoff_m=2.0, z=grade * x)
        res = QSSSolver().solve(car, trk, start="flying")
        brake_start = res.s[np.argmax(res.channels["brake_force_N"] > 1.0)]
        times[grade] = brake_start
    assert times[0.06] > times[0.0] > times[-0.06]  # uphill: brake later


def test_closed_hill_track_energy_balance(simple_cfg):
    cfg = with_overrides(simple_cfg, {"tires.rolling_resistance": 0.015})
    car = VehicleModel(cfg)
    ring = _hill_ring()
    res = QSSSolver().solve(car, ring)
    flat = QSSSolver().solve(car, _hill_ring(0.0))
    assert res.lap_time != pytest.approx(flat.lap_time, rel=1e-6)
    b = energy_breakdown(car, res)
    assert abs(b["potential_change_kWh"]) < 1e-6  # closed lap: back at the start height
    open_up = QSSSolver().solve(car, _straight(0.05), start="standing")
    b_up = energy_breakdown(car, open_up)
    climb = car.m * G * 0.05 * open_up.s[-1] / 3.6e6
    assert b_up["potential_change_kWh"] == pytest.approx(climb, rel=0.01)


def test_flat_z_column_changes_nothing(simple_cfg):
    car = VehicleModel(simple_cfg)
    x = np.linspace(0.0, 300.0, 301)
    a = QSSSolver().solve(car, Track.from_xy(x, np.zeros_like(x), closed=False, ds=0.5, cutoff_m=2.0),
                          start="standing")
    b = QSSSolver().solve(car, Track.from_xy(x, np.zeros_like(x), closed=False, ds=0.5, cutoff_m=2.0,
                                             z=np.full_like(x, 3.0)), start="standing")
    assert b.lap_time == a.lap_time


def test_track_file_grade_switch(tmp_path):
    p = tmp_path / "hill.csv"
    x = np.linspace(0.0, 100.0, 51)
    rows = "\n".join(f"{xi:.3f},0,{0.04 * xi:.4f}" for xi in x)
    p.write_text("# closed: false\nx_m,y_m,z_m\n" + rows + "\n")
    assert Track.from_file(p).interval_grade() is not None
    p.write_text("# closed: false\n# grade: off\nx_m,y_m,z_m\n" + rows + "\n")
    assert Track.from_file(p).interval_grade() is None


def test_transient_and_dp_follow_the_grade(simple_cfg):
    car = VehicleModel(simple_cfg)
    opts = SolverOptions.coarse()
    up = TransientSolver(TransientOptions(iterations=0, lateral_dof=False, qss=opts)).solve(car, _straight(0.05),
                                                                                             start="standing")
    flat = TransientSolver(TransientOptions(iterations=0, lateral_dof=False, qss=opts)).solve(car, _straight(0.0),
                                                                                               start="standing")
    assert up.lap_time > flat.lap_time
    assert up.lap_time == pytest.approx(QSSSolver(opts).solve(car, _straight(0.05), start="standing").lap_time,
                                        rel=0.03)
    dp = LapDP(car, _hill_ring(), opts)
    v, t, _ = dp.solve(0.0)
    assert t == pytest.approx(dp.flat.lap_time, rel=1e-9) and np.allclose(v, dp.flat.v)
    assert dp.lap(0.2).energy_J < dp.flat.energy_J  # 1 kJ worth 0.2 s: slowing down on the ring pays


def test_ac_track_uses_its_elevation(simple_cfg):
    track = Track.from_file(DATA / "tracks" / "fsk2026_ac_fast_lane.csv")
    g = track.interval_grade()
    assert g is not None and 0.003 < np.max(np.abs(g)) < 0.02
