import numpy as np
import pytest

from aerolap.cli import main
from aerolap.config import with_overrides
from aerolap.features import features_for_level
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.transient import BodyDynamics, TransientOptions, TransientSolver
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

from .conftest import DATA


def _step_response(cfg, zeta, t_end=1.0, dt=0.001):
    cfg = with_overrides(cfg, {"suspension.front.damping_ratio": zeta, "suspension.rear.damping_ratio": zeta})
    car = VehicleModel(cfg)
    body = BodyDynamics(car)
    el_f, el_r = 800.0, 600.0  # sudden downforce
    hist = []
    for _ in range(int(t_end / dt)):
        body.step(el_f, el_r, 0.0, dt)
        hist.append(body.q_f)
    return np.asarray(hist), el_f / body.kf


def test_body_settles_to_static_deflection_and_damping_controls_overshoot(ev_cfg):
    low, target = _step_response(ev_cfg, 0.2)
    high, _ = _step_response(ev_cfg, 1.0)
    assert low[-1] == pytest.approx(target, rel=1e-3)
    assert high[-1] == pytest.approx(target, rel=1e-3)
    assert low.max() > 1.3 * target  # under-damped: large overshoot
    assert high.max() < 1.05 * target  # critically damped: (almost) none


def test_roll_settles_to_quasi_static_roll(ev_cfg):
    car = VehicleModel(ev_cfg)
    body = BodyDynamics(car)
    m_roll = 600.0
    for _ in range(3000):
        body.step(0.0, 0.0, m_roll, 0.001)
    assert body.phi == pytest.approx(m_roll / car._k_roll_eff, rel=1e-3)


def test_steady_state_matches_qss(ev_cfg):
    """On a constant-radius circle the transient car stays in the QSS equilibrium."""
    car = VehicleModel(ev_cfg)
    tr = TransientSolver(TransientOptions(iterations=0, qss=SolverOptions.coarse()))
    res = tr.solve(car, Track.circle(9.125), start="flying")
    assert res.lap_time == pytest.approx(tr.qss_reference.lap_time, rel=1e-3)
    assert np.allclose(res.channels["roll_deg"][5:-5], tr.qss_reference.channels["roll_deg"][5:-5], atol=0.02)


def test_straight_line_acceleration_close_to_qss(ev_cfg):
    car = VehicleModel(ev_cfg, features_for_level("L4"))
    tr = TransientSolver(TransientOptions(iterations=0, qss=SolverOptions.coarse()))
    res = tr.solve(car, Track.straight(75.0), start="standing")
    assert res.lap_time == pytest.approx(tr.qss_reference.lap_time, rel=0.015)
    assert res.lap_time >= tr.qss_reference.lap_time - 1e-3  # dynamics never help in a straight line


def test_learning_removes_tyre_over_use(ev_cfg):
    track = Track.from_segments([("straight", 45.0, 0.0, ""), ("corner", 15.7, 10.0, "left"),
                                 ("straight", 30.0, 0.0, ""), ("corner", 12.6, 8.0, "right"),
                                 ("straight", 20.0, 0.0, "")], name="test", closed=False)
    car = VehicleModel(ev_cfg, features_for_level("L4"))
    tr = TransientSolver(TransientOptions(iterations=4, qss=SolverOptions.coarse()))
    res = tr.solve(car, track, start="standing")
    hist = res.info["iterations"]
    assert hist[-1]["max_lateral_excess"] <= max(tr.opts.excess_tol, 0.03)
    assert hist[-1]["max_lateral_excess"] <= hist[0]["max_lateral_excess"] + 1e-9
    assert res.lap_time >= tr.qss_reference.lap_time - 1e-3
    for col in ("heave_front_mm", "damper_vel_front_mm_s", "flap_position", "lateral_excess", "v_ref_kmh"):
        assert col in res.channels


def test_softer_damping_dives_deeper(ev_cfg):
    track = Track.from_segments([("straight", 60.0, 0.0, ""), ("corner", 15.7, 10.0, "left"),
                                 ("straight", 10.0, 0.0, "")], name="brake", closed=False)
    clear = {}
    for z in (0.25, 1.0):
        cfg = with_overrides(ev_cfg, {"suspension.front.damping_ratio": z, "suspension.rear.damping_ratio": z})
        res = TransientSolver(TransientOptions(iterations=0, qss=SolverOptions.coarse())).solve(
            VehicleModel(cfg, features_for_level("L4")), track, start="standing")
        clear[z] = res.summary()["min_rh_front_mm"]
    assert clear[0.25] < clear[1.0]


def test_cli_transient(tmp_path):
    veh = str(DATA / "vehicles" / "fsae_ev_aero.yaml")
    trk = str(DATA / "tracks" / "fsae_accel.csv")
    assert main(["transient", veh, trk, "--coarse", "--iterations", "0", "--out", str(tmp_path)]) == 0
    assert (tmp_path / "dynamics.png").exists()
    assert "transient" in (tmp_path / "transient.txt").read_text()
