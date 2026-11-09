import math

import numpy as np
import pytest

from aerolap.cli import main
from aerolap.config import with_overrides
from aerolap.features import features_for_level
from aerolap.road import ISO_CLASSES, N0, RoadProfile
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.transient import FL, FR, RideModel, TransientOptions, TransientSolver, _ride_loads
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

from .conftest import DATA

FLAT = ([0.0] * 4, [0.0] * 4)


def _loads(car, v, ax, ay, sgn=1.0):
    st = car.solve_state(v, ax, ay)
    comp = car.load_components(st.ax, st.ay, st.df_f, st.df_r, st.drag, st.roll)
    return st, _ride_loads({k: float(x) for k, x in comp.items()}, float(st.df_f), float(st.df_r), sgn)


# ------------------------------------------------------------------ ride model
def test_ride_equilibrium_matches_quasi_static(ev_cfg):
    car = VehicleModel(ev_cfg)
    st, loads = _loads(car, 25.0, -10.0, 12.0)
    ride = RideModel(car)
    ride.set_equilibrium(loads, FLAT[0])
    rh_f, rh_r = ride.ride_heights(FLAT[0])
    fz, _ = ride.tyre_loads(*FLAT)
    assert rh_f == pytest.approx(float(st.rh_f), abs=2e-4)
    assert rh_r == pytest.approx(float(st.rh_r), abs=2e-4)
    # left turn: roll to the right. The ride model also rolls on the tyre deflection caused by the
    # roll-centre (geometric) load transfer, which the quasi-static model leaves out -> ~10 % more
    assert ride.ph == pytest.approx(float(st.roll), rel=0.15)
    assert ride.ph >= float(st.roll)
    assert sum(fz) == pytest.approx(float(st.fz.sum()), rel=1e-6)
    assert fz[FR] > fz[FL]  # outer (right) front tyre carries more in a left turn


def _step_peak(cfg, zeta):
    cfg = with_overrides(cfg, {"suspension.front.damping_ratio": zeta, "suspension.rear.damping_ratio": zeta})
    car = VehicleModel(cfg)
    ride = RideModel(car)
    loads = (800.0, 600.0, 0.0, [0.0] * 4)  # sudden downforce
    target = RideModel(car)
    target.set_equilibrium(loads, FLAT[0])
    hist = []
    for _ in range(1200):
        ride.step(loads, lambda t: FLAT, 0.0, 0.0025, 5)
        hist.append(ride.h)
    return np.asarray(hist), target.h


def test_damping_controls_overshoot_and_body_settles(ev_cfg):
    low, target = _step_peak(ev_cfg, 0.2)
    high, _ = _step_peak(ev_cfg, 1.0)
    assert low[-1] == pytest.approx(target, rel=2e-3)
    assert high[-1] == pytest.approx(target, rel=2e-3)
    # damping ratio is defined on the corner spring; the (undamped) tyre spring in series lets the
    # body overshoot somewhat even at zeta = 1, but far less than with light damping
    assert low.max() > 1.5 * target
    assert high.max() < 1.25 * target


def test_bump_excites_wheel_and_tyre_load_recovers(ev_cfg):
    car = VehicleModel(ev_cfg)
    ride = RideModel(car)
    loads = (0.0, 0.0, 0.0, [0.0] * 4)
    bump = ([0.01, 0.01, 0.0, 0.0], [0.0] * 4)  # 10 mm step under both front wheels
    peak = 0.0
    for _ in range(1200):
        ride.step(loads, lambda t: bump, 0.0, 0.0025, 5)
        fz, _ = ride.tyre_loads(*bump)
        peak = max(peak, fz[FL] - ride.fz0[FL])
    assert peak > 0.5 * car.tire.vertical_rate * 0.01  # the tyre spring feels the bump
    # the body has pitched up over the bump; loads are statically determined -> back to static
    assert fz[FL] == pytest.approx(ride.fz0[FL], rel=0.01)
    assert ride.th < 0.0


# ------------------------------------------------------------------ road
@pytest.mark.parametrize("cls", ["A", "B"])
def test_iso_road_rms_matches_psd(cls):
    road = RoadProfile.iso8608(800.0, cls, seed=3, periodic=True)
    expected = math.sqrt(ISO_CLASSES[cls] * N0 ** 2 * (1 / 0.02 - 1 / 3.0))
    assert road.rms() == pytest.approx(expected, rel=0.25)
    zl0, zr0, _, _ = road.at(0.0)
    zl1, zr1, _, _ = road.at(800.0)
    assert zl0 == pytest.approx(zl1) and zr0 == pytest.approx(zr1)  # periodic


def test_road_csv(tmp_path):
    p = tmp_path / "road.csv"
    p.write_text("s_m,z_left_mm,z_right_mm\n0,0,0\n10,5,-5\n20,0,0\n")
    road = RoadProfile.from_csv(p)
    zl, zr, sl, sr = road.at(5.0)
    assert zl == pytest.approx(0.0025) and zr == pytest.approx(-0.0025)
    assert sl == pytest.approx(0.0005) and sr == pytest.approx(-0.0005)


# ------------------------------------------------------------------ solver
def test_steady_state_matches_qss(ev_cfg):
    """On a smooth constant-radius circle the transient car settles to the QSS answer."""
    car = VehicleModel(ev_cfg)
    tr = TransientSolver(TransientOptions(iterations=0, qss=SolverOptions.coarse()))
    res = tr.solve(car, Track.circle(9.125), start="flying")
    assert res.lap_time == pytest.approx(tr.qss_reference.lap_time, rel=5e-3)
    # same roll up to the tyre deflection from the geometric load transfer (see above)
    assert np.allclose(res.channels["roll_deg"][20:-5], tr.qss_reference.channels["roll_deg"][20:-5], rtol=0.15)


def test_straight_line_acceleration_close_to_qss(ev_cfg):
    car = VehicleModel(ev_cfg, features_for_level("L4"))
    tr = TransientSolver(TransientOptions(iterations=0, qss=SolverOptions.coarse()))
    res = tr.solve(car, Track.straight(75.0), start="standing")
    assert res.lap_time == pytest.approx(tr.qss_reference.lap_time, rel=0.015)
    assert res.lap_time >= tr.qss_reference.lap_time - 1e-3


_TEST_TRACK = [("straight", 45.0, 0.0, ""), ("corner", 15.7, 10.0, "left"), ("straight", 30.0, 0.0, ""),
               ("corner", 12.6, 8.0, "right"), ("straight", 20.0, 0.0, "")]


def test_learning_removes_sustained_tyre_over_use(ev_cfg):
    track = Track.from_segments(_TEST_TRACK, name="test", closed=False)
    car = VehicleModel(ev_cfg, features_for_level("L4"))
    tr = TransientSolver(TransientOptions(iterations=4, qss=SolverOptions.coarse()))
    res = tr.solve(car, track, start="standing")
    hist = res.info["iterations"]
    assert hist[-1]["max_lateral_excess"] <= 0.03
    assert hist[-1]["max_lateral_excess"] <= hist[0]["max_lateral_excess"] + 1e-9
    assert res.lap_time >= tr.qss_reference.lap_time - 1e-3
    for col in ("heave_front_mm", "damper_vel_front_mm_s", "flap_position", "lateral_excess", "v_ref_kmh",
                "road_front_mm"):
        assert col in res.channels


def test_rough_road_loads_tyres_and_costs_time(ev_cfg):
    track = Track.from_segments(_TEST_TRACK, name="test", closed=False)
    car = VehicleModel(ev_cfg, features_for_level("L4"))
    out = {}
    for road in (None, "B"):
        res = TransientSolver(TransientOptions(iterations=1, road=road, qss=SolverOptions.coarse())).solve(
            car, track, start="standing")
        out[road] = res
    assert out["B"].info["tyre_load_std_N"] > 3.0 * out[None].info["tyre_load_std_N"]
    assert out["B"].lap_time > out[None].lap_time
    assert out["B"].info["road_rms_mm"] > 1.0


def test_softer_springs_steady_the_tyres_on_a_rough_road(ev_cfg):
    track = Track.from_segments(_TEST_TRACK, name="test", closed=False)
    std = {}
    for name, k in (("soft", 25.0), ("stiff", 70.0)):
        cfg = with_overrides(ev_cfg, {"suspension.front.spring_rate_N_per_mm": k,
                                      "suspension.rear.spring_rate_N_per_mm": k})
        res = TransientSolver(TransientOptions(iterations=0, road="B", qss=SolverOptions.coarse())).solve(
            VehicleModel(cfg, features_for_level("L4")), track, start="standing")
        std[name] = res.info["tyre_load_std_N"]
    assert std["soft"] < std["stiff"]


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


def test_cli_transient_with_road(tmp_path):
    veh = str(DATA / "vehicles" / "fsae_ev_aero.yaml")
    trk = str(DATA / "tracks" / "fsae_accel.csv")
    assert main(["transient", veh, trk, "--coarse", "--iterations", "0", "--road", "A",
                 "--out", str(tmp_path)]) == 0
    assert (tmp_path / "dynamics.png").exists()
    assert "ISO 8608 class A" in (tmp_path / "transient.txt").read_text()


def test_roll_dynamics_through_a_change_of_direction(ev_cfg):
    """Ride states live in the vehicle frame, so the roll left over from one corner carries into
    the next: through an abrupt S-bend the body swings across and overshoots. (Storing roll
    relative to the turn direction would flip it instantly onto the new steady value instead.)"""
    track = Track.from_segments([("straight", 20.0, 0.0, ""), ("corner", 15.0, 12.0, "left"),
                                 ("corner", 15.0, 12.0, "right"), ("straight", 20.0, 0.0, "")],
                                name="s-bend", closed=False)
    solver = TransientSolver(TransientOptions(iterations=0, turn_in_length=0.0))
    res = solver.solve(VehicleModel(ev_cfg, features_for_level("L4")), track, start="flying")
    roll = res.channels["roll_deg"]
    roll_qss = solver.qss_reference.channels["roll_deg"]
    i = int(np.flatnonzero(res.kappa < -1e-6)[0])
    assert roll[i - 1] > 0.5  # still rolled from the left-hander
    assert roll[i] > roll_qss[i] + 0.2  # lags the new steady roll at the reversal
    assert roll[i:i + 20].min() < roll_qss[i + 10] - 0.2  # swings past it (overshoot)
