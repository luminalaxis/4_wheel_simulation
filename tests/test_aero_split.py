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
