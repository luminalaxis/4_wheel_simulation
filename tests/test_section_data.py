"""2-D section data in the AERO_AUTOMATION summary.csv format: loader, 3-D wing, ranking."""

import csv
import math
from pathlib import Path

import numpy as np
import pytest

from aerolap.aero.devices import Geometry, device_from_config
from aerolap.aero.section_data import load_sections
from aerolap.config import load_vehicle_config, with_overrides
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

ROOT = Path(__file__).resolve().parents[1]
FRONT = ROOT / "data" / "aero_sections" / "synthetic_front_summary.csv"
REAR = ROOT / "data" / "aero_sections" / "synthetic_rear_summary.csv"
CAR = ROOT / "data" / "vehicles" / "fsk2026_cfd_sections.yaml"
TRACK = ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv"
GEO = Geometry(35.0, 45.0, 1550.0)


def _rows(path):
    with open(path, newline="", encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def test_loader_drops_bad_rows_and_finds_geometries():
    front, rear = load_sections(FRONT), load_sections(REAR)
    assert front.synthetic and rear.synthetic
    assert front.dropped and sum(front.dropped.values()) == 3  # failed + diverged rows of the file
    assert set(front.keys) == {"main_airfoil", "flap1_chord", "flap1_deflection", "flap1_gap"}
    assert len(front.sections) == 2 * 3 * 2 * 2
    sec = front.select({"main_airfoil": 4412, "flap1_chord": 80, "flap1_deflection": 15, "flap1_gap": 6})
    assert sec.has_ground and sec.geometry_id == "4412_f80d15g6" and sec.c_ref == pytest.approx(0.175)
    r = rear.select("3el_4412c180_f80d20_a4412f60d40g6o5")
    assert not r.has_ground and r.heights is None
    with pytest.raises(ValueError, match="matches"):
        front.select({"main_airfoil": 4412})


def test_grid_points_reproduce_the_csv():
    sec = load_sections(FRONT).select("4412_f80d15g6")
    for row in _rows(FRONT):
        if row["main_airfoil"] == "4412" and row["flap1_chord"] == "80" and row["flap1_deflection"] == "15" \
                and row["flap1_gap"] == "6" and row["conv_state"] == "converged" and row["state"] == "done":
            a, h, v = float(row["aoa"]), float(row["ground_clearance"]), float(row["speed_kmh"])
            assert float(sec.cl_at(a, h, v)) == pytest.approx(float(row["cl"]), rel=1e-9)
            assert float(sec.cd_at(a, h, v)) == pytest.approx(float(row["cd"]), rel=1e-9)
    assert bool(sec.out_of_range(4.0, 150.0, 80.0)) and not bool(sec.out_of_range(4.0, 60.0, 80.0))


def _wing(path, section, **kw):
    c = {"type": "wing", "section_data": str(path), "section": section, "span_m": 1.0, "aoa_deg": 4.0,
         "x_mm": -650, "height_mm": 80, "endplate_factor": 1.5}
    c.update(kw)
    return device_from_config("w", c)


def test_lifting_line_correction():
    sec = load_sections(REAR).select("2el_4412c200_f80d25")
    big = _wing(REAR, sec.geometry_id, endplate_factor=1e4, cd_add=0.0, x_mm=1800, height_mm=950)
    cla, cda = (float(x) for x in big.coefficients(GEO, 35, 45, 0, 0))
    v = big.section_speed
    assert cla / big.area == pytest.approx(float(sec.cl_at(4.0, None, v)), rel=2e-3)  # AR -> inf: 2-D
    assert cda / big.area == pytest.approx(float(sec.cd_at(4.0, None, v)), rel=2e-3)
    small = _wing(REAR, sec.geometry_id, endplate_factor=1.0, x_mm=1800, height_mm=950)
    cla_s, cda_s = (float(x) for x in small.coefficients(GEO, 35, 45, 0, 0))
    cl = cla_s / small.area
    k = math.pi * small.e * small.ar_eff
    a_eff = 4.0 - math.degrees(cl / k)
    assert cl < cla / big.area
    assert cl == pytest.approx(float(sec.cl_at(a_eff, None, v)), rel=2e-3)  # converged fixed point
    assert cda_s / small.area == pytest.approx(float(sec.cd_at(a_eff, None, v)) + small.cd_add + cl ** 2 / k, rel=2e-3)


def test_front_ground_effect_and_rear_free_stream():
    fw = _wing(FRONT, "4412_f80d15g6")
    assert fw.ground_effect
    base = float(fw.coefficients(GEO, 35, 45, 0, 0)[0])
    assert float(fw.coefficients(GEO, 20, 45, 0, 0)[0]) > base  # nose lower -> closer to the ground
    rw = _wing(REAR, "2el_4412c200_f80d25", x_mm=1800, height_mm=950)
    assert not rw.ground_effect
    assert float(rw.coefficients(GEO, 20, 30, 0, 0)[0]) == pytest.approx(float(rw.coefficients(GEO, 35, 45, 0, 0)[0]))
    assert float(rw.coefficients(GEO, 35, 45, 0, 5)[0]) < float(rw.coefficients(GEO, 35, 45, 0, 0)[0])  # yaw


def test_vehicle_with_section_wings_runs_and_matches_optimumlap():
    cfg = load_vehicle_config(CAR)
    assert cfg["_warnings"] == []
    car = VehicleModel(cfg)
    cf, cr, cd, _ = car.aero.coefficients(0, 35.0, 45.0, 0.0, 0.0)
    assert float(cf + cr) == pytest.approx(0.997, rel=0.02) and float(cd) == pytest.approx(0.470, rel=0.02)
    res = QSSSolver(SolverOptions.coarse()).solve(car, Track.from_file(TRACK), start="flying")
    assert 55.0 < res.lap_time < 70.0
    more = VehicleModel(with_overrides(cfg, {"aero.components.rear.aoa_deg": 2.0}))
    assert float(sum(more.aero.coefficients(0, 35.0, 45.0, 0.0, 0.0)[:2])) < float(cf + cr)


@pytest.mark.parametrize("csv_path,component", [(REAR, "rear"), (FRONT, "front")])
def test_rank_sections(csv_path, component, tmp_path):
    from aerolap.gui_tasks import TASKS

    res = TASKS["rank_sections"][1](load_vehicle_config(CAR), str(TRACK), "L4", True, tmp_path, log=lambda m: None,
                                    section_csv=str(csv_path), section_component=component, objective="lap_time",
                                    max_geometries=3)
    rk = res.data
    assert len(rk.scores) == 3 and res.images[0].exists() and "SYNTHETIC" in res.text
    best = rk.ordered()[0]
    lo, hi = load_sections(csv_path).sections[best.geometry_id].aoa[[0, -1]]
    assert lo <= best.aoa <= hi
    again = load_vehicle_config(tmp_path / "best_vehicle.yaml")
    assert again["aero"]["components"][component]["section"] == best.geometry_id
    car = VehicleModel(again)
    assert np.isfinite(float(sum(car.aero.coefficients(0, 35.0, 45.0, 0.0, 0.0)[:2])))
