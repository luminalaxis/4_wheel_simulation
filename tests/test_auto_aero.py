"""Aero on/off options, CFD-corrected element sections, the design space, auto-aero and cfd-plan."""

import csv
import tomllib
from pathlib import Path

import numpy as np
import pytest

from aerolap.aero.devices import Geometry, device_from_config
from aerolap.analysis.auto_aero import auto_aero, hooke_jeeves
from aerolap.analysis.cfd_plan import cfd_plan
from aerolap.analysis.design_space import design_space
from aerolap.cli import main
from aerolap.config import ConfigError, get_path, load_vehicle_config, unknown_keys, with_overrides
from aerolap.vehicle import VehicleModel

ROOT = Path(__file__).resolve().parents[1]
CAR = ROOT / "data" / "vehicles" / "fsk2026.yaml"
TRACK = ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv"
FRONT = ROOT / "data" / "aero_sections" / "synthetic_front_summary.csv"
GEO = Geometry(35.0, 45.0, 1550.0)
WING = {"type": "wing", "span_m": 1.0, "aoa_deg": 4.0, "x_mm": -650, "height_mm": 80, "endplate_factor": 1.5,
        "elements": {"main": {"chord_mm": 175, "airfoil": "4412"},
                     "flap1": {"chord_mm": 80, "deflection_deg": 15, "gap_mm": 6, "overlap_mm": 5, "airfoil": "4412"}}}


def _cla(dev):
    return float(np.asarray(dev.coefficients(GEO, 35.0, 45.0, 0.0, 0.0)[0]))


# ---------------------------------------------------------------- on / off options
def test_switched_off_device_leaves_the_car_with_its_mass():
    cfg = load_vehicle_config(CAR)
    off = with_overrides(cfg, {"aero.components.side.enabled": False, "aero.components.side.mass_kg": 2.5})
    car0, car1 = VehicleModel(cfg), VehicleModel(off)
    assert [c.name for c in car1.aero.components] == ["front", "sidepod", "rear"]
    assert car1.m == pytest.approx(car0.m - 2.5)
    assert sum(car1.aero.component_totals()[:2]) < sum(car0.aero.component_totals()[:2])
    assert unknown_keys(off) == []  # enabled / mass_kg are known keys


def test_switched_off_flap_renumbers_the_model_but_keeps_config_names():
    cfg = with_overrides(load_vehicle_config(CAR), {"aero.components.front.elements.flap1.enabled": False})
    dev = next(c for c in VehicleModel(cfg).aero.components if c.name == "front").dev
    assert len(dev.model.elements) == 2 and dev.model.flaps[0].chord_mm == 60  # old flap2 is now the only flap
    keys = [k for _, k, *_ in dev.design_variables()]
    assert keys == ["aoa_deg", "elements.flap2.deflection_deg"]  # writable back into the vehicle file
    with pytest.raises(ConfigError, match="main"):
        device_from_config("w", {**WING, "elements": {"main": {**WING["elements"]["main"], "enabled": False},
                                                      "flap1": WING["elements"]["flap1"]}})


# ---------------------------------------------------------------- CFD-corrected sections
def test_section_fit_is_the_cfd_at_a_cfd_geometry_and_continuous_between():
    exact = device_from_config("w", {**WING, "section_data": str(FRONT)})  # picks the matching CFD geometry
    fit = device_from_config("w", {**WING, "section_data": str(FRONT), "section_fit": True})
    assert _cla(fit) == pytest.approx(_cla(exact), rel=1e-3)
    assert not fit.cfd and [k for _, k, *_ in fit.design_variables()] == ["aoa_deg", "elements.flap1.deflection_deg"]
    vals = []
    for d in np.arange(15.0, 25.01, 1.0):  # CFD has 15 and 25 deg: no jumps in between
        el = {**WING["elements"], "flap1": {**WING["elements"]["flap1"], "deflection_deg": float(d)}}
        vals.append(_cla(device_from_config("w", {**WING, "elements": el, "section_data": str(FRONT),
                                                  "section_fit": True})))
    steps = np.diff(vals)
    assert np.all(np.abs(steps) < 3.0 * np.median(np.abs(steps)) + 1e-6)


# ---------------------------------------------------------------- design space
def test_design_space_fix_patterns_bounds_and_errors():
    cfg = load_vehicle_config(CAR)
    sp = design_space(cfg, ["rear", "@position", "*.span_m", "front.flap1"],
                      {"front.elements.main.chord_mm": [150, 190]})
    by = {v.name: v for v in sp.variables}
    assert all(v.fixed for v in sp.variables if v.device == "rear" or v.group == "position" or v.name.endswith("span_m"))
    assert by["front.elements.flap1.gap_mm"].fixed and by["front.elements.flap1.deflection_deg"].fixed
    assert not by["front.elements.flap2.gap_mm"].fixed and not by["front.aoa_deg"].fixed
    assert (by["front.elements.main.chord_mm"].lo, by["front.elements.main.chord_mm"].hi) == (150, 190)
    opts = {o.name for o in sp.options}
    assert "side.enabled" in opts and "rear.elements.flap2.enabled" in opts and "sidepod.enabled" not in opts
    with pytest.raises(ValueError, match="matches no"):
        design_space(cfg, ["front.elements.flap9.gap_mm"])
    with pytest.raises(ValueError, match="upper bound"):
        design_space(cfg, bounds={"front.aoa_deg": [5, 1]})
    in_file = with_overrides(cfg, {"aero.design": {"fixed": ["front"], "options": False}})
    sp2 = design_space(in_file)
    assert all(v.fixed for v in sp2.variables if v.device == "front") and not sp2.free_options


def test_pattern_search_finds_a_quadratic_minimum():
    target = np.linspace(0.2, 0.8, 5)
    n = [0]

    def f(u):
        n[0] += 1
        return float(((np.asarray(u) - target) ** 2 * np.arange(1, 6)).sum())

    x, fx = hooke_jeeves(f, np.full(5, 0.5), lambda: n[0] >= 200)
    assert fx < 1e-3 and n[0] <= 200


# ---------------------------------------------------------------- auto-aero
def test_auto_aero_keeps_the_fixed_parameters_and_does_not_get_worse():
    cfg = load_vehicle_config(CAR)
    fixed = ["rear", "side", "sidepod", "@position", "@chord", "@slot", "*.span_m"]
    res = auto_aero(cfg, {"type": "lap_time", "track": str(TRACK)}, ".", {"understeer_index": [1.0, 1.3]},
                    fixed, options=True, max_evals=40)
    assert res.best_eval.feasible and res.best_eval.value <= res.initial_eval.value + 1e-9
    for v in res.space.variables:
        if v.fixed:
            assert get_path(res.best_cfg, v.path) == get_path(cfg, v.path)
    assert set(res.changes) <= {v.path for v in res.final_space.free}
    assert res.trials and all(t.name.startswith("front") for t in res.trials)
    assert "design space" in res.report(cfg) and res.evals <= 40 + 5


# ---------------------------------------------------------------- cfd-plan round trip
def test_cfd_plan_writes_an_aero_automation_project_that_section_fit_reads_back(tmp_path):
    cfg = load_vehicle_config(CAR)
    plan = cfd_plan(cfg, "front", ["flap1.deflection", "flap1.gap"])
    assert plan.ground and len(plan.rows) == 5 and plan.clearance == [60.0, 80.0, 100.0]
    files = plan.write(tmp_path)
    proj = tomllib.loads(files[0].read_text(encoding="utf-8"))
    assert proj["cases"]["csv"] == "cases.csv" and "ground_clearance" in proj["cases"]["sweep"]
    rows = list(csv.DictReader(open(files[1], encoding="utf-8")))
    assert {"geom", "main_chord", "flap1_deflection", "flap2_gap", "base_case"} <= set(rows[0])
    assert len({r["geom"] for r in rows}) == len(rows)
    # fake "Fluent" results for the plan: element model x 0.9 -> summary.csv in AERO_AUTOMATION columns
    from aerolap.aero.calibration import section_from_params

    out = []
    for r in rows:
        sec = section_from_params({k: (v if k.endswith("airfoil") or k in ("geom", "base_case") else float(v))
                                   for k, v in r.items()})
        for h in plan.clearance:
            for a in plan.aoa:
                cl, cd = sec.coefficients(a, h)
                out.append({**r, "state": "done", "conv_state": "converged", "ground_clearance": h, "aoa": a,
                            "speed_kmh": 70, "cl": 0.9 * float(cl), "cd": float(cd)})
    summary = tmp_path / "summary.csv"
    with open(summary, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(out[0]))
        w.writeheader()
        w.writerows(out)
    comp = cfg["aero"]["components"]["front"]
    model = device_from_config("f", comp)
    fit = device_from_config("f", {**comp, "section_data": str(summary), "section_fit": True})
    ratio = fit.section.cl_at(2.0, 80.0) / model.section.cl_at(2.0, 80.0)
    assert float(ratio) == pytest.approx(0.9, rel=1e-3)


def test_cli_auto_aero_dry_run_and_cfd_plan(tmp_path, capsys):
    assert main(["auto-aero", str(CAR), str(TRACK), "--fix", "rear,@option", "--dry-run", "--out",
                 str(tmp_path / "a")]) == 0
    assert "kept (--fix: rear)" in capsys.readouterr().out
    assert main(["cfd-plan", str(CAR), "--component", "rear", "--out", str(tmp_path / "p")]) == 0
    assert (tmp_path / "p" / "project.toml").exists()
    assert main(["auto-aero", str(CAR), str(TRACK), "--fix", "nosuch", "--dry-run"]) == 2
