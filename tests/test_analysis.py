import numpy as np
import pytest

from aerolap.analysis.cornering import braking_sweep, cornering_sweep
from aerolap.analysis.objectives import Objective
from aerolap.analysis.optimize import OptimizationSpec, Variable, run_optimization
from aerolap.analysis.sweep import parse_values, sweep
from aerolap.cli import main
from aerolap.config import with_overrides
from aerolap.solvers.ggv import SolverOptions
from aerolap.vehicle import VehicleModel

from .conftest import DATA


def test_parse_values():
    assert np.allclose(parse_values("0:1:0.25"), [0, 0.25, 0.5, 0.75, 1.0])
    assert np.allclose(parse_values("1,2.5"), [1, 2.5])


def test_cornering_balance_moves_with_aero(ev_cfg):
    speeds = np.array([8.0, 30.0])
    base = cornering_sweep(VehicleModel(ev_cfg), speeds, opts=SolverOptions.coarse())
    fwd = cornering_sweep(VehicleModel(with_overrides(ev_cfg, {"aero.cop_shift": 0.05})), speeds,
                          opts=SolverOptions.coarse())
    # forward CoP -> less understeer, and much more so at high speed (aero) than at low speed
    d_low = base["understeer_index"][0] - fwd["understeer_index"][0]
    d_high = base["understeer_index"][1] - fwd["understeer_index"][1]
    assert d_high > 0.02
    assert d_high > 3.0 * abs(d_low)
    assert base["ay_max_g"][1] > base["ay_max_g"][0]


def test_braking_sweep_reports_pitch(ev_cfg):
    out = braking_sweep(VehicleModel(ev_cfg), [15.0, 30.0], opts=SolverOptions.coarse())
    assert np.all(out["ax_min_g"] < -1.0)
    assert out["rh_front_mm"][1] < out["rh_front_mm"][0]


def test_sweep_finds_cop_balance_point(simple_cfg):
    """ay_max at high speed peaks where both axles saturate together."""
    obj = Objective({"type": "ay_max", "speeds_kmh": [100]}, opts=SolverOptions.coarse())
    vals = np.linspace(-0.2, 0.2, 21)
    sw = sweep(simple_cfg, ["aero.cop_shift"], [vals], obj)
    best = vals[np.argmin(sw.objective())]
    assert -0.15 < best < 0.15  # interior optimum

    spec = OptimizationSpec(
        vehicle_cfg=simple_cfg, objective_spec={"type": "ay_max", "speeds_kmh": [100]},
        variables=[Variable("aero.cop_shift", -0.2, 0.2)], constraints={}, base_dir=DATA,
        method="powell", max_evals=40, coarse=True)
    res = run_optimization(spec)
    assert res.best["aero.cop_shift"] == pytest.approx(best, abs=0.025)
    assert res.best_eval.value <= res.initial_eval.value + 1e-9


def test_optimizer_respects_constraints(ev_cfg):
    spec = OptimizationSpec(
        vehicle_cfg=ev_cfg, objective_spec={"type": "skidpad_time"},
        variables=[Variable("aero.cop_shift", -0.1, 0.1),
                   Variable("suspension.front.arb_rate_Nm_per_deg", 0.0, 600.0)],
        constraints={"understeer_index": [1.0, 1.3], "min_clearance_mm": 5.0},
        base_dir=DATA, method="powell", max_evals=40, coarse=True)
    res = run_optimization(spec)
    assert res.best_eval.feasible
    assert res.best_eval.value <= res.initial_eval.value + 1e-9


def test_cli_run_and_compare(tmp_path):
    veh = str(DATA / "vehicles" / "fsae_ev_aero.yaml")
    trk = str(DATA / "tracks" / "fsae_accel.csv")
    assert main(["run", veh, trk, "--coarse", "--out", str(tmp_path / "run")]) == 0
    assert (tmp_path / "run" / "trace.csv").exists() and (tmp_path / "run" / "lap.png").exists()
    assert main(["compare", veh, trk, "--coarse", "--levels", "L0,L5", "--out", str(tmp_path / "cmp")]) == 0
    assert "L5" in (tmp_path / "cmp" / "ladder.txt").read_text()
    assert main(["run", veh, trk, "--set", "mass.total_kg=-1", "--out", str(tmp_path / "bad")]) == 2
