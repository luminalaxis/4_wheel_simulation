"""Aero device split (front / side / rear components) and its per-device study."""

from pathlib import Path

import numpy as np
import pytest

from aerolap import plotting
from aerolap.analysis.aero_split import aero_split, design_variables, pin_ground_effect, resolve_names
from aerolap.config import load_vehicle_config, with_overrides
from aerolap.solvers.ggv import SolverOptions
from aerolap.vehicle import VehicleModel

ROOT = Path(__file__).resolve().parents[1]
CAR = ROOT / "data" / "vehicles" / "fsk2026.yaml"
TRACK = ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv"


SIMPLE = {"front": {"cla": 0.30, "cda": 0.05, "cop_front": 1.32, "scale": 1.0},
          "side": {"cla": 0.30, "cda": 0.04, "cop_front": 0.50, "scale": 1.0},
          "rear": {"cla": 0.397, "cda": 0.13, "cop_front": -0.16, "scale": 1.0}}


def simple_cfg():
    """The car with the simple (cla x scale) components instead of the physics-based devices."""
    return with_overrides(load_vehicle_config(CAR), {"aero.components": SIMPLE, "aero.cda": 0.25})


def test_components_add_up_and_move_the_balance():
    cfg = simple_cfg()
    car = VehicleModel(cfg)
    cf, cr, cd, _ = car.aero.coefficients(0, 30.0, 40.0, 0.0, 0.0)
    assert float(cf + cr) == pytest.approx(0.997, abs=1e-6)
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
    # a device name brings all its variables (main aoa + flap angles), a variable name just that one
    res = aero_split(cfg, spec, constraints={"understeer_index": [1.0, 1.3]}, names=["front", "rear.flap1", "sidepod"],
                     points=3, max_evals=12, opts=SolverOptions.coarse())
    assert res.names == ["front", "front.flap1", "front.flap2", "rear.flap1", "sidepod"]
    assert res.paths["front"].endswith("front.aoa_deg")
    assert res.paths["rear.flap1"] == "aero.components.rear.elements.flap1.deflection_deg"
    assert res.paths["sidepod"].endswith("exit_area_m2") and res.units["sidepod"] == "m2"
    assert res.values["front"][0] == pytest.approx(-4.0)
    assert res.values["front.flap1"][0] == pytest.approx(0.0) and res.values["front.flap2"][-1] == pytest.approx(50.0)
    with pytest.raises(ValueError, match="front.flap9"):
        aero_split(cfg, spec, names=["front.flap9"], points=3, joint=False, opts=SolverOptions.coarse())
    simple = aero_split(simple_cfg(), spec, bounds=(0.5, 1.5), points=3, joint=False, opts=SolverOptions.coarse())
    i1 = int(np.argmin(np.abs(simple.values["front"] - 1.0)))
    assert all(simple.sweeps[n][i1].value == pytest.approx(simple.base.value) for n in simple.names)
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
    cfg = with_overrides(simple_cfg(), {"aero.components.rear.points": pts})
    car = VehicleModel(cfg)
    cf, cr, cd, _ = car.aero.coefficients(0, 30.0, 40.0, 0.0, 0.0)
    assert float(cf + cr) == pytest.approx(0.30 + 0.30 + 0.40) and float(cd) == pytest.approx(0.25 + 0.05 + 0.04 + 0.13)


def test_unreachable_balance_window_is_flagged():
    cfg = load_vehicle_config(CAR)
    spec = {"type": "lap_time", "track": str(TRACK), "level": "L4"}
    res = aero_split(cfg, spec, constraints={"understeer_index": [1.2, 1.3]}, names=["rear"], points=3,
                     joint=False, opts=SolverOptions.coarse())
    assert "understeer-index window" in res.warning and "WARNING" in res.report(cfg)
    ok = aero_split(cfg, spec, constraints={"understeer_index": [1.0, 1.3]}, names=["rear"], points=3,
                    joint=False, opts=SolverOptions.coarse())
    assert ok.warning == ""


def test_detail_variable_names_and_patterns():
    cfg = load_vehicle_config(CAR)
    basic = set(design_variables(cfg))
    full = design_variables(cfg, detail=True)
    assert basic < set(full) and full["rear.flap1.gap"][0] == "aero.components.rear.elements.flap1.gap_mm"
    assert "rear.height" not in full and "front.height" in full  # only in ground effect
    assert resolve_names(cfg, ["rear"]) == ["rear", "rear.flap1", "rear.flap2"]
    rear = resolve_names(cfg, ["rear"], detail=True)
    assert rear[:3] == ["rear", "rear.flap1", "rear.flap2"] and {"rear.span", "rear.x", "rear.main.chord",
                                                                  "rear.flap2.overlap"} <= set(rear)
    # a detail variable or a pattern is used without --detail
    assert resolve_names(cfg, ["rear.flap1.gap"]) == ["rear.flap1.gap"]
    assert resolve_names(cfg, ["*.gap"]) == ["front.flap1.gap", "front.flap2.gap", "side.flap1.gap",
                                             "rear.flap1.gap", "rear.flap2.gap"]
    assert resolve_names(cfg, ["rear.flap2*"]) == ["rear.flap2", "rear.flap2.chord", "rear.flap2.gap",
                                                   "rear.flap2.overlap"]
    assert set(resolve_names(cfg, None, detail=True)) == set(full) and set(resolve_names(cfg, None)) == basic
    with pytest.raises(ValueError, match="rear.flap1.gapp"):
        resolve_names(cfg, ["rear.flap1.gapp"])


def test_aero_split_optimises_detail_variables(tmp_path):
    cfg = load_vehicle_config(CAR)
    spec = {"type": "lap_time", "track": str(TRACK), "level": "L4"}
    # more span = more downforce and a better lap on this track: capped at the given upper bound
    res = aero_split(cfg, spec, constraints={"understeer_index": [1.0, 1.3]}, names=["rear.span", "rear.flap1.gap"],
                     points=0, max_evals=10, opts=SolverOptions.coarse(),
                     ranges={"rear.span": (0.9, 1.0), "rear.flap1.gap": (4.0, 10.0)})
    assert res.sweeps == {} and res.ranges["rear.span"] == (0.9, 1.0)
    j = res.joint
    assert [v.path for v in j.spec.variables] == ["aero.components.rear.span_m",
                                                   "aero.components.rear.elements.flap1.gap_mm"]
    assert 0.9 <= j.best["aero.components.rear.span_m"] <= 1.0
    assert 4.0 <= j.best["aero.components.rear.elements.flap1.gap_mm"] <= 10.0
    assert j.best_eval.value <= j.initial_eval.value + 1e-9
    text = res.report(cfg)
    assert "rear.span" in text and "rear.flap1.gap" in text and " mm" in text
    if j.best["aero.components.rear.span_m"] > 0.99:
        assert ">>" in text and "on a bound: rear.span" in text
    plotting.plot_aero_split(res, tmp_path / "positions.png")
    assert (tmp_path / "positions.png").exists()
    with pytest.raises(ValueError, match="front.span"):
        aero_split(cfg, spec, names=["rear.span"], points=0, joint=False, ranges={"front.span": (0.9, 1.1)})
    with pytest.raises(ValueError, match="upper bound"):
        aero_split(cfg, spec, names=["rear.span"], points=0, joint=False, ranges={"rear.span": (1.1, 0.9)})


def test_auto_evaluation_budget_grows_with_the_variables():
    from aerolap.analysis.aero_split import auto_max_evals

    assert auto_max_evals(3) == 80 and auto_max_evals(30) == 450


def test_geometry_changes_do_not_switch_ground_effect():
    cfg = load_vehicle_config(CAR)
    side = "aero.components.side"
    bigger = {f"{side}.elements.main.chord_mm": 190.0, f"{side}.elements.flap1.chord_mm": 90.0}  # chords 280 > 250 mm
    auto = VehicleModel(with_overrides(cfg, bigger))
    assert next(c for c in auto.aero.components if c.name == "side").dev.ground_effect  # the automatic choice flips
    pinned, notes = pin_ground_effect(cfg, ["side.main.chord", "rear"])
    assert notes == ["side (off)"] and pinned["aero"]["components"]["side"]["ground_effect"] is False
    assert "ground_effect" not in pinned["aero"]["components"]["rear"]  # angles only: nothing to pin
    assert "ground_effect" not in cfg["aero"]["components"]["side"]  # the input is not changed
    fixed = VehicleModel(with_overrides(pinned, bigger))
    assert not next(c for c in fixed.aero.components if c.name == "side").dev.ground_effect
    assert pin_ground_effect(cfg, ["front", "rear.flap1"]) == (cfg, [])
