"""Aero device split (front / side / rear components) and its per-device study."""

from pathlib import Path

import numpy as np
import pytest

from aerolap.analysis.aero_split import aero_split
from aerolap.config import load_vehicle_config, with_overrides
from aerolap.solvers.ggv import SolverOptions
from aerolap.vehicle import VehicleModel

ROOT = Path(__file__).resolve().parents[1]
CAR = ROOT / "data" / "vehicles" / "fsk2026.yaml"
TRACK = ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv"


def test_components_add_up_and_move_the_balance():
    cfg = load_vehicle_config(CAR)
    car = VehicleModel(cfg)
    cf, cr, cd, _ = car.aero.coefficients(0, 30.0, 40.0, 0.0, 0.0)
    assert float(cf + cr) == pytest.approx(0.997, abs=1e-6)  # OptimumLap total ClA
    assert float(cd) == pytest.approx(0.470, abs=1e-6)
    more_front = VehicleModel(with_overrides(cfg, {"aero.components.front.scale": 1.5}))
    f2, r2, d2, _ = more_front.aero.coefficients(0, 30.0, 40.0, 0.0, 0.0)
    assert float(f2 / (f2 + r2)) > float(cf / (cf + cr))
    assert float(d2) == pytest.approx(0.470 + 0.5 * 0.05)
    off = VehicleModel(with_overrides(cfg, {"aero.components.rear.scale": 0.0}))
    assert float(sum(off.aero.coefficients(0, 30.0, 40.0, 0.0, 0.0)[:2])) == pytest.approx(0.997 - 0.397)


def test_aero_split_study():
    cfg = load_vehicle_config(CAR)
    spec = {"type": "lap_time", "track": str(TRACK), "level": "L4"}
    res = aero_split(cfg, spec, constraints={"understeer_index": [1.0, 1.3]}, bounds=(0.5, 1.5), points=3,
                     max_evals=12, opts=SolverOptions.coarse())
    assert set(res.sweeps) == {"front", "side", "rear"}
    i1 = int(np.argmin(np.abs(res.values - 1.0)))
    assert all(res.sweeps[n][i1].value == pytest.approx(res.base.value) for n in res.names)
    assert res.joint is not None and res.joint.best_eval.value <= res.joint.initial_eval.value + 1e-9
    assert "joint optimum" in res.report(cfg)
    # one basis: the current package (fine) is the joint optimisation's starting value
    assert res.base_fine.value == pytest.approx(res.joint.initial_eval.value, rel=1e-9)
    assert set(res.best_fine) == set(res.names)


def test_component_points_replace_the_constant_lift_to_drag_ratio():
    from aerolap.aero.maps import parse_components

    pts = [[0.5, 0.25, 0.06], [1.0, 0.40, 0.13], [1.5, 0.48, 0.24]]  # drag grows faster than downforce
    comp = lambda s: parse_components({"rear": {"cop_front": 0.0, "scale": s, "points": pts}})[0].forces()  # noqa: E731
    assert sum(comp(1.0)[:2]) == pytest.approx(0.40) and comp(1.0)[2] == pytest.approx(0.13)
    assert sum(comp(1.25)[:2]) == pytest.approx(0.44) and comp(1.25)[2] == pytest.approx(0.185)
    assert sum(comp(2.0)[:2]) == pytest.approx(0.56)  # extrapolated with the last slope
    assert comp(0.0) == (0.0, 0.0, 0.0)
    cfg = with_overrides(load_vehicle_config(CAR), {"aero.components.rear.points": pts})
    car = VehicleModel(cfg)
    cf, cr, cd, _ = car.aero.coefficients(0, 30.0, 40.0, 0.0, 0.0)
    assert float(cf + cr) == pytest.approx(0.30 + 0.30 + 0.40) and float(cd) == pytest.approx(0.25 + 0.05 + 0.04 + 0.13)


def test_unreachable_balance_window_is_flagged():
    cfg = load_vehicle_config(CAR)
    spec = {"type": "lap_time", "track": str(TRACK), "level": "L4"}
    res = aero_split(cfg, spec, constraints={"understeer_index": [1.2, 1.3]}, bounds=(0.8, 1.2), points=3,
                     joint=False, opts=SolverOptions.coarse())
    assert "understeer-index window" in res.warning and "WARNING" in res.report(cfg)
    ok = aero_split(cfg, spec, constraints={"understeer_index": [1.0, 1.3]}, bounds=(0.8, 1.2), points=3,
                    joint=False, opts=SolverOptions.coarse())
    assert ok.warning == ""
