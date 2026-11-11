"""Physics-based aero devices: multi-element wings and the underbody."""

from pathlib import Path

import math

import numpy as np
import pytest

from aerolap.aero.devices import Floor, Geometry, Wing
from aerolap.config import load_vehicle_config, with_overrides
from aerolap.features import features_for_level
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

ROOT = Path(__file__).resolve().parents[1]
CAR = ROOT / "data" / "vehicles" / "fsk2026.yaml"
GEO = Geometry(35.0, 45.0, 1550.0)


def _wing(**kw):
    args = dict(name="w", span=1.0, chord=0.3, elements=2, aoa=2.0, x=1800.0, height=950.0, endplate=1.8)
    args.update(kw)
    return Wing(**args)


def test_wing_lift_rises_with_angle_then_stalls():
    cla = []
    for a in np.arange(-4, 40, 2.0):
        cla.append(float(_wing(aoa=a).coefficients(GEO, 35, 45, 0, 0)[0]))
    cla = np.array(cla)
    k = int(np.argmax(cla))
    assert 0 < k < len(cla) - 1  # a maximum, then the stall
    assert np.all(np.diff(cla[:k - 2]) > 0) and cla[-1] < cla[k]
    w = _wing()
    assert float(w.coefficients(GEO, 35, 45, 0, 0)[0]) / w.area <= w.cl_max


def test_induced_drag_makes_lift_to_drag_worse_with_angle():
    # L/D peaks where induced drag equals profile drag, then falls as the angle grows
    ld = np.array([float(np.divide(*_wing(aoa=a).coefficients(GEO, 35, 45, 0, 0))) for a in range(-4, 20, 2)])
    k = int(np.argmax(ld))
    assert 0 < k < len(ld) - 1 and np.all(np.diff(ld[k:]) < 0)
    w = _wing(aoa=6.0)
    cla, cda = (float(x) for x in w.coefficients(GEO, 35, 45, 0, 0))
    cl = cla / w.area
    induced = cl ** 2 / (math.pi * w.e * w.ar_eff)
    assert cda / w.area == pytest.approx(w.cd0 * (1 + (6 / 15) ** 2) + induced)
    assert _wing(endplate=2.5).ar_eff > _wing().ar_eff


def test_front_wing_ground_effect_pitch_roll_and_yaw():
    fw = Wing("f", 1.0, 0.25, 1, 2.0, -650.0, 80.0)
    base = float(fw.coefficients(GEO, 35, 45, 0, 0)[0])
    assert float(fw.coefficients(GEO, 25, 45, 0, 0)[0]) > base  # lower nose -> more ground effect
    assert float(fw.coefficients(GEO, 35, 35, 0, 0)[0]) < base  # rear down / nose up (pitch) at x < 0
    assert float(fw.coefficients(GEO, 35 - 60, 45, 0, 0)[0]) < base  # ground stall when nearly touching
    assert float(fw.coefficients(GEO, 35, 45, 0, 5)[0]) == pytest.approx(base * math.cos(math.radians(5)) ** 2)
    assert float(fw.coefficients(GEO, 35, 45, 2, 0)[0]) != pytest.approx(base)
    assert not _wing().ground_effect  # a high rear wing is out of ground effect


def test_floor_height_choke_rake_and_yaw():
    fl = Floor("floor", 1.0, 0.34, 40.0, 800.0)
    c = lambda rf, rr, roll=0.0, yaw=0.0: float(fl.coefficients(GEO, rf, rr, roll, yaw)[0])  # noqa: E731
    assert c(35, 45) == pytest.approx(0.34)
    assert c(25, 35) > c(35, 45)  # closer to the ground: more downforce ...
    assert c(35 - 33, 45 - 33) < c(35 - 25, 45 - 25)  # ... until the flow chokes below stall_mm
    assert c(35, 50) > c(40, 45)  # more rake at the same mid-floor height
    assert c(35, 45, yaw=5) < c(35, 45) and c(35, 45, roll=1.5) < c(35, 45)


def test_car_devices_match_optimumlap_and_react_to_ride_height():
    cfg = load_vehicle_config(CAR)
    car = VehicleModel(cfg)
    cf, cr, cd, _ = car.aero.coefficients(0, 35.0, 45.0, 0.0, 0.0)
    assert float(cf + cr) == pytest.approx(0.997, rel=0.02) and float(cd) == pytest.approx(0.470, rel=0.02)
    lo = car.aero.coefficients(0, 28.0, 38.0, 0.0, 0.0)
    assert float(lo[0] + lo[1]) > float(cf + cr)
    # heave / pitch / roll now matter for this car: L3 and L4 differ from L2
    track = Track.from_file(ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv")
    t = {lv: QSSSolver(SolverOptions.coarse()).solve(VehicleModel(cfg, features_for_level(lv)), track,
                                                     start="flying").lap_time for lv in ("L2", "L3", "L4")}
    assert t["L3"] != pytest.approx(t["L2"], abs=1e-4) and t["L4"] != pytest.approx(t["L3"], abs=1e-4)


def test_angle_of_attack_is_a_design_variable(tmp_path):
    cfg = load_vehicle_config(CAR)
    more = VehicleModel(with_overrides(cfg, {"aero.components.rear.aoa_deg": 8.0}))
    base = VehicleModel(cfg)
    f0, r0, d0, _ = base.aero.coefficients(0, 35.0, 45.0, 0.0, 0.0)
    f1, r1, d1, _ = more.aero.coefficients(0, 35.0, 45.0, 0.0, 0.0)
    assert float(r1) > float(r0) and float(d1) > float(d0)


def test_exchange_rates_and_envelope(tmp_path):
    from aerolap.gui_tasks import TASKS

    cfg = load_vehicle_config(CAR)
    track = str(ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv")
    ex = TASKS["exchange"][1](cfg, track, "L4", True, tmp_path / "ex", log=lambda m: None, objective="lap_time")
    data = ex.data if ex.data is not None else None
    assert "exchange rate: ClA +0.1 is worth CdA" in ex.text and ex.images[0].exists()
    lines = {ln.split("  ")[0]: ln for ln in ex.text.splitlines()}
    assert any(k.startswith("ClA +0.1") for k in lines) and data is None
    env = TASKS["envelope"][1](cfg, track, "L4", True, tmp_path / "env", log=lambda m: None)
    assert "aero balance" in env.text and env.images[0].exists()
    assert "front: ground stall" in env.text and "side: choked" in env.text


def test_exchange_signs():
    from aerolap.analysis.aero_exchange import aero_exchange
    from aerolap.analysis.objectives import Objective

    cfg = load_vehicle_config(CAR)
    obj = Objective({"type": "lap_time", "track": str(ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv"),
                     "level": "L4"}, ".", None, SolverOptions.coarse())
    ex = aero_exchange(cfg, obj)
    assert ex.get("ClA").delta < 0 < ex.get("CdA").delta and ex.get("mass").delta > 0
    assert {"front", "side", "rear"} <= {i.label for i in ex.items}
