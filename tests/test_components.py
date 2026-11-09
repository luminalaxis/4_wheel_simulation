import math

import numpy as np
import pytest

from aerolap.aero.maps import AeroModel, ResponseSurfaceAero, TableAero
from aerolap.config import ConfigError, get_path, parse_override, vehicle_config_from_dict, with_overrides
from aerolap.environment import air_density
from aerolap.suspension import series
from aerolap.tire import Tire
from aerolap.track import Track


# ---------------------------------------------------------------- environment
def test_isa_sea_level_density():
    assert air_density(15.0, 101.325, relative_humidity=0.0) == pytest.approx(1.2250, abs=2e-4)


def test_density_drops_with_altitude_temperature_and_humidity():
    base = air_density(15.0, None, 0.0, 0.0)
    assert air_density(15.0, None, 1500.0, 0.0) < base
    assert air_density(35.0, None, 0.0, 0.0) < base
    assert air_density(15.0, None, 0.0, 1.0) < base


# ---------------------------------------------------------------- tyre
def _tire(**kw):
    p = dict(radius_mm=200, mu_x=1.4, mu_y=1.5, ref_load_N=600, load_sensitivity=-0.2, min_mu_factor=0.4,
             vertical_rate_N_per_mm=100, peak_slip_deg=6, camber_optimum_deg=-1.0,
             camber_sensitivity_per_deg2=0.01, rolling_resistance=0.0)
    p.update(kw)
    return Tire.from_config(p)


def test_load_sensitivity_is_degressive():
    t = _tire()
    fx, fy = t.capacity(np.array([600.0, 1200.0]))
    assert fy[0] == pytest.approx(1.5 * 600.0)
    assert fy[1] == pytest.approx(1.5 * 0.8 * 1200.0)  # -20 % mu at +100 % load
    assert fy[1] / 1200.0 < fy[0] / 600.0


def test_camber_optimum_gives_full_grip():
    t = _tire()
    opt = math.radians(1.0)  # inclination towards the turn centre = -optimum camber
    _, fy_opt = t.capacity(600.0, opt, -opt)
    _, fy_off = t.capacity(600.0, opt + math.radians(2.0), -opt)
    assert fy_opt == pytest.approx(900.0)
    assert fy_off == pytest.approx(900.0 * (1 - 0.01 * 4.0))


def test_slip_curve_inverse():
    t = _tire()
    u = np.array([0.0, 0.3, 0.8, 0.999])
    a = t.slip_angle(u)
    x = a / t.peak_slip
    assert np.allclose(2 * x / (1 + x * x), u, atol=1e-6)
    assert np.all(np.diff(a) > 0)


def test_series_springs():
    assert series(100.0, 100.0) == pytest.approx(50.0)
    assert series(math.inf, 70.0) == 70.0


# ---------------------------------------------------------------- config
def test_config_paths_and_overrides(simple_cfg):
    cfg = with_overrides(simple_cfg, {"suspension.front.spring_rate_N_per_mm": 55.0})
    assert get_path(cfg, "suspension.front.spring_rate_N_per_mm") == 55.0
    assert get_path(simple_cfg, "suspension.front.spring_rate_N_per_mm") == 30.0
    assert get_path(cfg, "suspension.front.motion_ratio") == 1.0  # default applied
    assert parse_override("aero.cop_shift=0.02") == ("aero.cop_shift", 0.02)


def test_config_validation(simple_raw):
    del simple_raw["mass"]["total_kg"]
    with pytest.raises(ConfigError):
        vehicle_config_from_dict(simple_raw)


# ---------------------------------------------------------------- aero maps
def _write_grid(path, fn, with_roll=True):
    rows = ["mode,front_rh_mm,rear_rh_mm" + (",roll_deg" if with_roll else "") + ",ClA,CdA,cop_front"]
    for mode in ("a", "b"):
        for rf in (10, 20, 30):
            for rr in (20, 40):
                for roll in ((0.0, 2.0) if with_roll else (0.0,)):
                    cla, cda, cop = fn(mode, rf, rr, roll)
                    vals = [mode, rf, rr] + ([roll] if with_roll else []) + [cla, cda, cop]
                    rows.append(",".join(str(v) for v in vals))
    path.write_text("\n".join(rows) + "\n")


def _linear(mode, rf, rr, roll):
    k = 1.0 if mode == "a" else 0.5
    return k * (4.0 - 0.02 * rf + 0.01 * rr - 0.1 * roll), 1.0 + 0.001 * rr, 0.4 + 0.002 * rf


def test_table_map_reproduces_and_interpolates(tmp_path):
    p = tmp_path / "map.csv"
    _write_grid(p, _linear)
    tab = TableAero.from_csv(p)
    assert tab.mode_names == ["a", "b"]
    f, r, d, oor = tab.coefficients(0, 20.0, 40.0, 2.0, 0.0)
    cla, cda, cop = _linear("a", 20, 40, 2.0)
    assert f + r == pytest.approx(cla)
    assert f / (f + r) == pytest.approx(cop)
    assert d == pytest.approx(cda)
    assert not oor
    # midpoint of a linear function: interpolation of ClA is exact
    f, r, d, _ = tab.coefficients(1, 15.0, 30.0, 1.0, 0.0)
    assert f + r == pytest.approx(_linear("b", 15, 30, 1.0)[0], rel=1e-3)
    # outside -> clamped and flagged
    f2, r2, _, oor = tab.coefficients(0, 5.0, 40.0, 0.0, 0.0)
    f3, r3, _, _ = tab.coefficients(0, 10.0, 40.0, 0.0, 0.0)
    assert oor and f2 + r2 == pytest.approx(f3 + r3)


def test_table_map_rejects_incomplete_grid(tmp_path):
    p = tmp_path / "map.csv"
    _write_grid(p, _linear)
    lines = p.read_text().splitlines()
    p.write_text("\n".join(lines[:-1]) + "\n")
    with pytest.raises(ValueError):
        TableAero.from_csv(p)


def test_response_surface_fits_quadratic_exactly(tmp_path):
    rng = np.random.default_rng(0)
    rows = ["mode,front_rh_mm,rear_rh_mm,ClA,CdA,cop_front"]
    for _ in range(30):
        rf, rr = rng.uniform(10, 50), rng.uniform(20, 60)
        cla = 3.0 + 0.01 * rf - 0.0004 * rf * rf + 0.0002 * rf * rr
        rows.append(f"m,{rf},{rr},{cla},{1.0 + 0.002 * rr},0.45")
    p = tmp_path / "pts.csv"
    p.write_text("\n".join(rows) + "\n")
    rs = ResponseSurfaceAero.from_csv(p)
    assert rs.fits[0].r2["ClA_f"] == pytest.approx(1.0, abs=1e-9)
    f, r, _, _ = rs.coefficients(0, 30.0, 40.0, 0.0, 0.0)
    assert f + r == pytest.approx(3.0 + 0.3 - 0.36 + 0.24, rel=1e-6)


def test_aero_model_setup_knobs():
    m = AeroModel({"type": "constant", "cla": 3.0, "cda": 1.0, "cop_front": 0.4, "cla_scale": 1.1,
                   "cop_shift": 0.05, "cda_scale": 0.9})
    f, r, d, _ = m.coefficients(0, 30.0, 40.0, 0.0, 0.0)
    assert f + r == pytest.approx(3.3)
    assert f / (f + r) == pytest.approx(0.45)
    assert d == pytest.approx(0.9)


# ---------------------------------------------------------------- track
def test_segment_track(tmp_path):
    p = tmp_path / "t.csv"
    p.write_text("# closed: false\ntype,length_m,radius_m,direction\nstraight,50,,\ncorner,31.4159,10,left\n"
                 "corner,15.708,5,right\n")
    t = Track.from_file(p, ds=0.5)
    assert t.length == pytest.approx(97.1239, abs=1e-3)
    assert not t.closed
    assert t.kappa[t.s < 49].max() == 0.0
    assert t.kappa[(t.s > 51) & (t.s < 80)] == pytest.approx(0.1)
    assert t.kappa[t.s > 83] == pytest.approx(-0.2)


def test_closed_segment_track_geometry_closes():
    t = Track.from_file(__import__("pathlib").Path(__file__).resolve().parents[1]
                        / "data" / "tracks" / "fsae_endurance_example.csv")
    x, y = t.xy()
    assert t.closed
    assert math.hypot(x[-1] - x[0], y[-1] - y[0]) < 0.1


def test_xy_track_circle_curvature():
    th = np.linspace(0, 2 * np.pi, 200, endpoint=False)
    t = Track.from_xy(20 * np.cos(th), 20 * np.sin(th), closed=True, ds=0.5, smoothing=0.001)
    assert t.length == pytest.approx(2 * np.pi * 20, rel=1e-3)
    assert np.median(t.kappa) == pytest.approx(1 / 20, rel=2e-3)
