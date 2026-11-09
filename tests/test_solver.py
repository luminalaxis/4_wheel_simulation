import math

import numpy as np
import pytest

from aerolap.config import vehicle_config_from_dict, with_overrides
from aerolap.features import features_for_level
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.track import Track
from aerolap.vehicle import G, VehicleModel

from .conftest import DATA


def _no_drag(cfg):
    return with_overrides(cfg, {"aero.cda": 0.0, "tires.rolling_resistance": 0.0})


def test_point_mass_corner_speed_matches_analytic(simple_cfg):
    cfg = _no_drag(simple_cfg)
    car = VehicleModel(cfg, features_for_level("L0"))
    R = 15.0
    res = QSSSolver().solve(car, Track.circle(R), start="flying")
    mu, cla = 1.5, 3.0
    v2 = mu * G / (1.0 / R - mu * car.rho * cla / (2.0 * car.m))
    assert res.v.mean() == pytest.approx(math.sqrt(v2), rel=2e-3)
    assert res.lap_time == pytest.approx(2 * math.pi * R / math.sqrt(v2), rel=2e-3)


def test_top_speed_matches_power_drag_balance(simple_cfg):
    cfg = with_overrides(simple_cfg, {"tires.rolling_resistance": 0.0, "tires.mu_x": 5.0, "tires.mu_y": 5.0})
    car = VehicleModel(cfg, features_for_level("L0"))
    res = QSSSolver().solve(car, Track.straight(2500.0), start="standing")
    p = 60e3
    v_top = (2 * p / (car.rho * 1.2)) ** (1 / 3)  # P = 1/2 rho CdA v^3
    assert res.v.max() == pytest.approx(v_top, rel=5e-3)


def test_full_model_reduces_to_point_mass(simple_raw):
    """No CG height, no roll moment, CoP = weight split, no sensitivities -> identical to L0."""
    simple_raw["mass"]["cg_height_mm"] = 0.0
    for ax in ("front", "rear"):
        simple_raw["suspension"][ax]["roll_center_mm"] = 0.0
    simple_raw["aero"]["cop_front"] = simple_raw["mass"]["weight_dist_front"]
    cfg = vehicle_config_from_dict(simple_raw)
    track = Track.from_file(DATA / "tracks" / "fsae_endurance_example.csv")
    t0 = QSSSolver().solve(VehicleModel(cfg, features_for_level("L0")), track).lap_time
    t4 = QSSSolver().solve(VehicleModel(cfg, features_for_level("L4")), track).lap_time
    assert t4 == pytest.approx(t0, rel=1e-4)


def test_rigid_suspension_removes_heave_pitch_effect(ev_cfg):
    cfg = with_overrides(ev_cfg, {
        "suspension.front.spring_rate_N_per_mm": 1e7, "suspension.rear.spring_rate_N_per_mm": 1e7,
        "tires.vertical_rate_N_per_mm": 1e7})
    track = Track.from_file(DATA / "tracks" / "fsae_autocross_example.csv")
    opts = SolverOptions.coarse()
    t2 = QSSSolver(opts).solve(VehicleModel(cfg, features_for_level("L2")), track).lap_time
    t3 = QSSSolver(opts).solve(VehicleModel(cfg, features_for_level("L3")), track).lap_time
    assert t3 == pytest.approx(t2, abs=2e-3)


def test_map_without_roll_yaw_columns_gives_l4_equal_l3(ev_cfg, tmp_path):
    from aerolap.aero.maps import TableAero, grid_rows

    tab = TableAero.from_csv(DATA / "aeromaps" / "fsae_example_grid.csv")

    class NoRollYaw:  # same map, roll/yaw ignored
        mode_names = tab.mode_names

        def ranges(self, m):
            return {k: v for k, v in tab.ranges(m).items() if k in ("front_rh_mm", "rear_rh_mm")}

        def coefficients(self, m, rf, rr, roll, yaw):
            return tab.coefficients(m, rf, rr, 0.0, 0.0)

    p = tmp_path / "map.csv"
    p.write_text("\n".join(grid_rows(NoRollYaw(), step_mm=5.0)) + "\n")
    cfg = with_overrides(ev_cfg, {"aero.file": str(p), "tires.camber_sensitivity_per_deg2": 0.0})
    track = Track.from_file(DATA / "tracks" / "fsae_autocross_example.csv")
    opts = SolverOptions.coarse()
    t3 = QSSSolver(opts).solve(VehicleModel(cfg, features_for_level("L3")), track).lap_time
    t4 = QSSSolver(opts).solve(VehicleModel(cfg, features_for_level("L4")), track).lap_time
    assert t4 == pytest.approx(t3, abs=2e-3)


def test_ideal_active_aero_is_an_upper_bound(ev_cfg):
    track = Track.from_file(DATA / "tracks" / "fsae_endurance_example.csv")
    opts = SolverOptions.coarse()
    times = {}
    for name, over in {
        "high_df": {"aero.strategy.type": "fixed"},
        "drs": {"aero.strategy.type": "fixed", "aero.default_mode": "drs"},
        "ideal": {"aero.strategy.type": "ideal"},
        "rule": {"aero.strategy.type": "rule"},
    }.items():
        times[name] = QSSSolver(opts).solve(VehicleModel(with_overrides(ev_cfg, over)), track).lap_time
    assert times["ideal"] <= min(times["high_df"], times["drs"]) + 1e-3
    assert times["ideal"] <= times["rule"] + 1e-3


def test_rule_drs_only_opens_on_straights(ev_cfg):
    track = Track.from_file(DATA / "tracks" / "fsae_endurance_example.csv")
    car = VehicleModel(ev_cfg)
    res = QSSSolver(SolverOptions.coarse()).solve(car, track)
    is_open = res.mode == car.strategy.open_mode
    assert is_open.any()
    assert np.all(np.abs(res.kappa[is_open]) < 1.0 / car.strategy.open_radius + 1e-9)
    # opens only above min_speed (it may still be closing below it at the start of braking)
    accelerating = res.ax > 0.0
    assert np.all(res.v[is_open & accelerating] >= car.strategy.min_speed - 1e-6)


def test_mesh_convergence(ev_cfg):
    track = Track.from_file(DATA / "tracks" / "fsae_autocross_example.csv")
    car = VehicleModel(ev_cfg, features_for_level("L4"))
    t_a = QSSSolver(SolverOptions(ds=0.5)).solve(car, track, start="standing").lap_time
    t_b = QSSSolver(SolverOptions(ds=0.25)).solve(car, track, start="standing").lap_time
    assert abs(t_a - t_b) / t_b < 1e-3


def test_fsae_event_times_are_plausible(ev_cfg):
    from aerolap.analysis.events import run_events

    ev = {e.name: e.time_s for e in run_events(VehicleModel(ev_cfg), opts=SolverOptions.coarse())}
    assert 3.5 < ev["acceleration"] < 4.5
    assert 4.6 < ev["skidpad"] < 5.6


def test_lap_result_csv(ev_cfg, tmp_path):
    track = Track.from_file(DATA / "tracks" / "fsae_accel.csv")
    res = QSSSolver(SolverOptions.coarse()).solve(VehicleModel(ev_cfg), track, start="standing")
    res.to_csv(tmp_path / "trace.csv")
    header = (tmp_path / "trace.csv").read_text().splitlines()[0].split(",")
    for col in ("s_m", "v_kmh", "rh_front_mm", "roll_deg", "cla", "cop_front", "fz_fl_N", "aero_mode"):
        assert col in header
