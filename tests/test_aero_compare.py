"""No aero vs current vs optimised aero."""

from pathlib import Path

import pytest

from aerolap.analysis.aero_compare import aero_compare, no_aero_cfg
from aerolap.config import dump_config, load_vehicle_config, with_overrides
from aerolap.gui_tasks import TASKS
from aerolap.solvers.ggv import SolverOptions
from aerolap.vehicle import VehicleModel

ROOT = Path(__file__).resolve().parents[1]
CAR = ROOT / "data" / "vehicles" / "fsk2026.yaml"
TRACK = ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv"


def test_no_aero_car_has_no_downforce_keeps_cooling_and_is_lighter():
    cfg = load_vehicle_config(CAR)
    bare, notes = no_aero_cfg(cfg, aero_mass_kg=8.0)
    car, base = VehicleModel(bare), VehicleModel(cfg)
    cf, cr, cd, _ = car.static_aero()
    assert float(cf + cr) == pytest.approx(0.0, abs=1e-9)
    assert 0.0 < float(cd) < float(base.static_aero()[2])  # sidepod / body drag stays, wing drag goes
    assert car.m == pytest.approx(base.m - 8.0)
    assert bare["aero"]["components"]["sidepod"].get("enabled", True)
    assert any("front" in n for n in notes)


def test_no_aero_of_a_map_car_zeroes_the_map_downforce():
    cfg = load_vehicle_config(ROOT / "data" / "vehicles" / "fsae_ev_aero.yaml")
    bare, _ = no_aero_cfg(cfg)
    cf, cr, _, _ = VehicleModel(bare).static_aero()
    assert float(cf + cr) == pytest.approx(0.0, abs=1e-9)


def test_compare_ranks_the_cars(tmp_path):
    cfg = load_vehicle_config(CAR)
    opt = with_overrides(cfg, {"aero.components.rear.aoa_deg": 13.0})
    res = aero_compare(cfg, str(TRACK), opt, "L4", 22.0, SolverOptions.coarse(), 8.0, log=lambda m: None)
    none, cur, best = res.variants
    assert none.cla == pytest.approx(0.0, abs=1e-9) and best.cla > cur.cla > 0.5
    assert cur.time("skidpad") < none.time("skidpad")  # downforce helps in the corner
    text = res.table()
    assert "에어로 없음" in text and "최적화" in text and "autocross" in text


def test_gui_task_with_an_optimised_yaml(tmp_path):
    cfg = load_vehicle_config(CAR)
    dump_config(with_overrides(cfg, {"aero.components.rear.aoa_deg": 13.0}), tmp_path / "opt.yaml")
    res = TASKS["aero_compare"][1](cfg, str(TRACK), "L4", True, tmp_path / "out", log=lambda m: None,
                                   compare_opt_yaml=str(tmp_path / "opt.yaml"), aero_mass_kg="")
    assert len(res.data.variants) == 3 and res.images[0].exists() and "에어로 없음 대비" in res.summary
