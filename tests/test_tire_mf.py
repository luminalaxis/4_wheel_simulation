import math

import numpy as np
import pytest

from aerolap.cli import main as cli_main
from aerolap.config import ConfigError, load_vehicle_config, unknown_keys, with_overrides
from aerolap.features import features_for_level
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.solvers.transient import TransientOptions, TransientSolver
from aerolap.tire import Tire, tire_from_config
from aerolap.tire_mf import MagicFormulaTire, read_tir
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

from .conftest import DATA

TIR = DATA / "tires" / "fsae_16x7.5-10_synthetic.tir"


def _tire(**kw):
    t = {"radius_mm": 203.0, "vertical_rate_N_per_mm": 100.0, "rolling_resistance": 0.015, "ref_load_N": 650.0,
         "mu_x": 1.5, "mu_y": 1.55, "mf_file": str(TIR)}
    t.update(kw)
    return tire_from_config(t)


def test_read_tir_file():
    p = read_tir(TIR)
    assert p["FNOMIN"] == 1100.0
    assert p["PKY1"] == -34.0
    assert p["LENGTH"] == "meter"
    assert p["NOMPRES"] == 83000.0


def test_simple_config_still_builds_the_simple_tyre(simple_cfg):
    car = VehicleModel(simple_cfg)
    assert isinstance(car.tire, Tire) and car.tire.model == "simple"
    assert car.tire.combined_exponent == 2.0


def test_friction_is_scaled_to_the_configured_mu_at_the_reference_load():
    t = _tire()
    fx, fy = t.capacity(np.array(650.0))
    assert float(fx) == pytest.approx(1.5 * 650.0, rel=1e-9)
    assert float(fy) == pytest.approx(1.55 * 650.0, rel=1e-9)
    raw = _tire(mf_mu_scaling=False)
    assert float(raw.mu_y(650.0)) == pytest.approx(2.45 - 0.55 * (650.0 - 1100.0) / 1100.0, rel=1e-9)


def test_load_sensitivity_and_load_dependent_peak_slip():
    t = _tire()
    loads = np.array([300.0, 650.0, 1000.0, 1500.0])
    mu = t.mu_y(loads)
    assert np.all(np.diff(mu) < 0.0)  # degressive
    pk = t.peak_slip_at(loads)
    assert np.all(np.diff(pk[1:]) > 0.0)  # peak slip angle grows with load
    assert 3.0 < math.degrees(float(pk[1])) < 9.0


def test_slip_curve_inverse_peak_and_sliding():
    t = _tire()
    for fz in (300.0, 650.0, 1400.0):
        for u in (0.05, 0.3, 0.7, 0.95, 0.999):
            a = t.slip_angle(u, fz)
            assert float(t.lateral_shape(a, fz)) == pytest.approx(u, abs=1e-6)
        pk = float(t.peak_slip_at(fz))
        assert float(t.lateral_shape(pk, fz)) == pytest.approx(1.0, abs=1e-9)
        assert float(t.lateral_shape(0.9 * pk, fz)) < 1.0
        assert float(t.lateral_shape(1.5 * pk, fz)) < 1.0
        assert float(t.lateral_shape(-pk, fz)) == pytest.approx(-1.0, abs=1e-9)  # odd
        far = float(t.lateral_shape(50.0, fz))
        assert far == pytest.approx(math.sin(t.cy * math.pi / 2.0), abs=0.02)  # sliding level


def test_camber_thrust_gives_an_optimum_lean_towards_the_turn_centre():
    t = _tire()
    g_opt, gain = t.camber_optimum(650.0)
    assert 1.5 < math.degrees(g_opt) < 4.5
    assert gain > 0.01
    base = float(t.capacity(np.array(650.0))[1])
    lean_in = float(t.capacity(np.array(650.0), math.radians(2.0), math.radians(-2.0))[1])
    lean_out = float(t.capacity(np.array(650.0), math.radians(-2.0), math.radians(2.0))[1])
    assert lean_out < base < lean_in
    fx0 = float(t.capacity(np.array(650.0))[0])
    fx_cam = float(t.capacity(np.array(650.0), math.radians(3.0), math.radians(-3.0))[0])
    assert fx_cam < fx0  # longitudinal grip is best upright


def test_inflation_pressure_changes_grip_and_cornering_stiffness():
    lo, nom, hi = _tire(pressure_kPa=72.0), _tire(), _tire(pressure_kPa=97.0)
    # the example tyre has its lateral grip optimum near 72.6 kPa (10.5 psi)
    assert float(lo.mu_y(650.0)) > float(nom.mu_y(650.0)) > float(hi.mu_y(650.0))
    assert float(hi.cornering_stiffness(650.0)) > float(nom.cornering_stiffness(650.0))
    # the friction scaling refers to the nominal pressure, so pressure sweeps keep their effect
    assert float(nom.mu_y(650.0)) == pytest.approx(1.55)
    # the configured vertical rate is the one at the nominal pressure (PFZ1 = 0.8 in the example)
    assert nom.vertical_rate == pytest.approx(100e3)
    assert hi.vertical_rate == pytest.approx(100e3 * (1.0 + 0.8 * (97.0 - 83.0) / 83.0))


def test_combined_slip_exponent():
    t = _tire()
    assert 1.7 < t.combined_exponent < 2.4
    assert _tire(combined_exponent=2.6).combined_exponent == 2.6
    no_terms = _tire(mf={"RBX1": 0.0})
    assert no_terms.combined_exponent == 2.0
    with pytest.raises(ConfigError):
        _tire(combined_exponent=0.5)


def test_bad_tir_file_is_reported(tmp_path):
    bad = tmp_path / "bad.tir"
    bad.write_text("[UNITS]\nLENGTH = 'meter'\nANGLE = 'degree'\n[VERTICAL]\nFNOMIN = 1000\n")
    with pytest.raises(ConfigError) as err:
        _tire(mf_file=str(bad))
    assert "tires.mf_file" in str(err.value)
    with pytest.raises(ConfigError):
        _tire(mf_file=str(tmp_path / "missing.tir"))


def test_mf_vehicle_from_base_file_runs_and_has_no_unknown_keys():
    cfg = load_vehicle_config(DATA / "vehicles" / "fsk2026_mf.yaml", warn=False)
    base = load_vehicle_config(DATA / "vehicles" / "fsk2026.yaml", warn=False)
    assert cfg["mass"] == base["mass"] and cfg["aero"]["components"] == base["aero"]["components"]
    assert not unknown_keys(cfg)
    car = VehicleModel(cfg)
    assert isinstance(car.tire, MagicFormulaTire)
    res = QSSSolver(SolverOptions.coarse()).solve(car, Track.from_file(DATA / "tracks" / "fsae_autocross_example.csv"),
                                                  start="standing")
    assert 30.0 < res.lap_time < 45.0
    assert np.isfinite(res.channels["yaw_deg"]).all()


def test_mf_with_the_simple_tyres_behaviour_gives_the_same_lap(simple_cfg):
    """MF without load / camber sensitivity and n = 2 has the simple tyre's capacities -> same L3 lap
    (L3 does not use the slip angle, which is where the two slip curves differ)."""
    mf = with_overrides(simple_cfg, {"tires.mf_file": str(TIR), "tires.combined_exponent": 2.0,
                                     "tires.mf": {"PDX2": 0.0, "PDY2": 0.0, "PDX3": 0.0, "PDY3": 0.0, "PVY3": 0.0,
                                                  "PVY4": 0.0}})
    track = Track.from_file(DATA / "tracks" / "fsae_autocross_example.csv")
    opts = SolverOptions.coarse()
    t_simple = QSSSolver(opts).solve(VehicleModel(simple_cfg, features_for_level("L3")), track).lap_time
    t_mf = QSSSolver(opts).solve(VehicleModel(mf, features_for_level("L3")), track).lap_time
    assert t_mf == pytest.approx(t_simple, rel=1e-9)


def test_combined_exponent_shapes_the_envelope(simple_cfg):
    """A fuller friction envelope (n > 2) allows more braking while cornering."""
    track = Track.from_file(DATA / "tracks" / "fsae_autocross_example.csv")
    opts = SolverOptions.coarse()
    t2 = QSSSolver(opts).solve(VehicleModel(simple_cfg), track).lap_time
    t3 = QSSSolver(opts).solve(VehicleModel(with_overrides(simple_cfg, {"tires.combined_exponent": 3.0})),
                               track).lap_time
    t15 = QSSSolver(opts).solve(VehicleModel(with_overrides(simple_cfg, {"tires.combined_exponent": 1.5})),
                                track).lap_time
    assert t3 < t2 < t15


def test_transient_lateral_dynamics_with_the_mf_tyre():
    cfg = load_vehicle_config(DATA / "vehicles" / "fsk2026_mf.yaml", warn=False)
    car = VehicleModel(cfg)
    solver = TransientSolver(TransientOptions(iterations=1, qss=SolverOptions.coarse()))
    res = solver.solve(car, Track.circle(9.125, name="skidpad"), start="flying")
    assert not res.channels["spin"].any()
    assert res.lap_time == pytest.approx(solver.qss_reference.lap_time, rel=0.04)


def test_base_file_paths_and_cycles(tmp_path):
    sub = tmp_path / "variants"
    sub.mkdir()
    (sub / "car.yaml").write_text(f"base: {(DATA / 'vehicles' / 'fsk2026_mf.yaml').as_posix()}\nname: variant\n"
                                  "tires: {pressure_kPa: 70}\n")
    cfg = load_vehicle_config(sub / "car.yaml", warn=False)
    assert cfg["name"] == "variant" and cfg["tires"]["pressure_kPa"] == 70
    car = VehicleModel(cfg)  # the .tir path of the base file still resolves from the other folder
    assert car.tire.pressure == pytest.approx(70e3)
    (tmp_path / "a.yaml").write_text("base: b.yaml\n")
    (tmp_path / "b.yaml").write_text("base: a.yaml\n")
    with pytest.raises(ConfigError):
        load_vehicle_config(tmp_path / "a.yaml", warn=False)


def test_tire_cli(tmp_path):
    assert cli_main(["tire", str(TIR), "--mu-x", "1.5", "--mu-y", "1.55", "--out", str(tmp_path)]) == 0
    assert (tmp_path / "tire.png").exists()
    assert "Magic Formula" in (tmp_path / "tire.txt").read_text()
