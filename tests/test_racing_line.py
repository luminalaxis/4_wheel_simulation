import numpy as np
import pytest

from aerolap.cli import main as cli_main
from aerolap.gui_tasks import TASKS
from aerolap.racing_line import LineError, optimal_offsets, optimise_line, reference_line
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

from .conftest import DATA

AUTOX = DATA / "tracks" / "fsae_autocross_example.csv"
AC = DATA / "tracks" / "fsk2026_ac_fast_lane.csv"


def test_track_keeps_width_and_elevation_columns():
    t = Track.from_file(AC)
    assert t.has_width and t.z is not None
    assert 5.0 < float(np.mean(t.width_left + t.width_right)) < 6.5
    r = t.resampled(1.0)
    assert r.has_width and len(r.width_left) == len(r.s)
    seg = Track.from_file(AUTOX)
    assert not seg.has_width
    assert seg.with_width(4.0).resampled(1.0).width_right[0] == pytest.approx(2.0)


def test_minimum_curvature_on_a_ring_runs_along_the_outer_edge():
    ring = Track.circle(20.0, direction="left", ds=0.5).with_width(6.0)
    ref = reference_line(ring, 1.0)
    margin = 1.0
    hi, lo = ref.wl - margin, -(ref.wr - margin)
    alpha, _ = optimal_offsets(ref, lo, hi, 0.0)
    assert np.allclose(alpha, lo, atol=1e-3)  # left turn: outside = right = negative offset
    alpha_short, _ = optimal_offsets(ref, lo, hi, 0.05)  # strong length weight: the inner edge
    assert np.allclose(alpha_short, hi, atol=1e-3)


def test_the_chosen_ring_line_is_the_fastest_candidate(simple_cfg):
    """On a ring the lap time grows with sqrt(radius): the length weight must win over pure min-curvature."""
    car = VehicleModel(simple_cfg)
    ring = Track.circle(20.0, direction="left", ds=0.5).with_width(6.0)
    res = optimise_line(car, ring, car_width=1.4, clearance=0.3, opts=SolverOptions.coarse())
    times = [c.lap_time for c in res.candidates]
    assert res.best.lap_time == pytest.approx(min(times))
    assert res.candidates[0].lap_time > res.reference_lap.lap_time  # outer edge: longer and slower
    assert res.lap.lap_time < res.reference_lap.lap_time
    assert res.best.length < res.reference_lap.s[-1]


def test_line_stays_inside_the_track_and_is_faster(simple_cfg, tmp_path):
    car = VehicleModel(simple_cfg)
    track = Track.from_file(AC)
    opts = SolverOptions.coarse()
    res = optimise_line(car, track, clearance=0.2, opts=opts)
    assert np.all(res.best.alpha <= res.hi + 1e-9) and np.all(res.best.alpha >= res.lo - 1e-9)
    margin = 0.5 * res.car_width + res.clearance
    # the moved points keep the clearance to both edges
    assert np.all(res.ref.wl - res.best.alpha >= margin - 1e-6)
    assert np.all(res.ref.wr + res.best.alpha >= margin - 1e-6)
    assert res.lap.lap_time < res.reference_lap.lap_time
    assert np.max(np.abs(res.lap.kappa)) < np.max(np.abs(res.reference_lap.kappa))
    # the written line is an ordinary track that gives the same lap
    path = res.write_csv(tmp_path / "line.csv")
    again = Track.from_file(path)
    assert again.has_width and again.closed
    assert QSSSolver(opts).solve(car, again).lap_time == pytest.approx(res.lap.lap_time, rel=2e-3)
    assert "racing line" in res.report()


def test_open_track_with_a_given_width(simple_cfg):
    car = VehicleModel(simple_cfg)
    res = optimise_line(car, Track.from_file(AUTOX).with_width(4.0), opts=SolverOptions.coarse())
    assert not res.ref.closed
    assert res.lap.lap_time < res.reference_lap.lap_time


def test_track_without_width_is_an_error(simple_cfg):
    with pytest.raises(LineError, match="--width"):
        optimise_line(VehicleModel(simple_cfg), Track.from_file(AUTOX))


def test_narrow_track_keeps_the_reference_line(simple_cfg):
    car = VehicleModel(simple_cfg)
    res = optimise_line(car, Track.from_file(AUTOX).with_width(1.5), opts=SolverOptions.coarse(), weights=(0.0,))
    assert res.narrow_points == res.ref.n
    assert np.allclose(res.best.alpha, 0.0)
    assert "narrower than the car" in res.report()


def test_line_cli_and_gui_task(tmp_path):
    car = DATA / "vehicles" / "fsk2026.yaml"
    assert cli_main(["line", str(car), str(AUTOX), "--width", "4", "--coarse", "--weights", "0,0.01",
                     "--out", str(tmp_path / "cli")]) == 0
    for f in ("racing_line.csv", "racing_line.png", "racing_line.txt", "trace.csv"):
        assert (tmp_path / "cli" / f).exists()
    from aerolap.config import load_vehicle_config

    res = TASKS["line"][1](load_vehicle_config(car), str(AUTOX), "L4", True, tmp_path / "gui", log=lambda m: None,
                           track_width=4.0)
    assert res.data.csv_path.exists() and res.images[0].exists() and "라인 랩" in res.summary
