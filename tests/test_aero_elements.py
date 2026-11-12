"""Per-element wings (main + flaps: chord, angle, gap, overlap) and the sidepod / radiator device."""

from pathlib import Path

import numpy as np
import pytest

from aerolap.aero.devices import Geometry, Sidepod, device_from_config
from aerolap.aero.elements import Element, ElementSection, naca4
from aerolap.analysis.aero_split import design_variables
from aerolap.config import ConfigError, load_vehicle_config, unknown_keys, with_overrides
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

ROOT = Path(__file__).resolve().parents[1]
CAR = ROOT / "data" / "vehicles" / "fsk2026.yaml"
TRACK = ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv"
FRONT_CSV = ROOT / "data" / "aero_sections" / "synthetic_front_summary.csv"
GEO = Geometry(35.0, 45.0, 1550.0)
AOA = np.arange(-5.0, 25.0, 0.5)


def section(defl=(15.0, 35.0), gaps=(6.0, 5.0), n=3):
    chords = (175.0, 80.0, 60.0)
    els = [Element("main", chords[0])]
    for i in range(1, n):
        els.append(Element(f"flap{i}", chords[i], defl[i - 1], gaps[i - 1], 4.0))
    return ElementSection(tuple(els))


def wing_cfg(**flaps):
    el = {"main": {"chord_mm": 175, "airfoil": "4412"},
          "flap1": {"chord_mm": 80, "deflection_deg": 15, "gap_mm": 6, "overlap_mm": 4},
          "flap2": {"chord_mm": 60, "deflection_deg": 35, "gap_mm": 5, "overlap_mm": 3}}
    for k, v in flaps.items():
        name, key = k.split("__")
        el[name][key] = v
    return {"type": "wing", "span_m": 1.0, "aoa_deg": 2.0, "x_mm": -650, "height_mm": 80, "elements": el}


def test_naca_codes():
    assert naca4("4412") == (0.04, 0.12) and naca4(4412.0) == (0.04, 0.12) and naca4("NACA 2412") == (0.02, 0.12)
    with pytest.raises(ValueError):
        naca4("e423")


def test_flap_angle_adds_lift_until_the_flap_separates():
    cl = [float(section((d, d + 20.0)).coefficients(2.0)[0]) for d in range(0, 45, 5)]
    ld = [float(np.divide(*section((d, d + 20.0)).coefficients(2.0))) for d in range(0, 45, 5)]
    k = int(np.argmax(cl))
    assert 0 < k < len(cl) - 1 and np.all(np.diff(cl[:k + 1]) > 0)  # more flap -> more lift, then separation
    assert ld[-1] < max(ld) * 0.7  # an over-deflected flap is mostly drag
    st = section((40.0, 80.0)).states(2.0)
    assert st[0]["ratio"] < 1.0 and st[1]["ratio"] >= 1.0 and st[2]["ratio"] >= 1.0  # flaps stall, main does not


def test_main_angle_stalls_the_main_element():
    s = section()
    cl = s.coefficients(AOA)[0]
    k = int(np.argmax(cl))
    assert 0 < k < len(AOA) - 1 and cl[-1] < cl[k]
    assert s.states(AOA[-1])[0]["ratio"] > 1.0


def test_more_elements_more_maximum_lift_and_slot_gap_matters():
    m1, m2, m3 = (float(section(n=n).coefficients(AOA)[0].max()) for n in (1, 2, 3))
    assert m1 < m2 < m3 and 2.3 < m2 < 3.2 and 3.0 < m3 < 4.2  # per main chord, typical 2-D values
    best = float(section(gaps=(6.0, 6.0)).coefficients(AOA)[0].max())
    assert float(section(gaps=(20.0, 20.0)).coefficients(AOA)[0].max()) < best
    assert float(section(gaps=(0.5, 0.5)).coefficients(AOA)[0].max()) < best


def test_wing_device_from_elements_and_its_design_variables():
    w = device_from_config("front", wing_cfg(), ".")
    assert w.model is not None and w.ground_effect and w.chord == pytest.approx(0.175)
    names = [v[0] for v in w.design_variables()]
    assert names == ["", ".flap1", ".flap2"]
    cla0 = float(w.coefficients(GEO, 35, 45, 0, 0)[0])
    more = device_from_config("front", wing_cfg(flap1__deflection_deg=22), ".")
    assert float(more.coefficients(GEO, 35, 45, 0, 0)[0]) > cla0
    assert float(w.coefficients(GEO, 25, 45, 0, 0)[0]) > cla0  # lower nose: more ground effect
    assert [st["name"] for st in w.element_states(GEO)] == ["main", "flap1", "flap2"]


def test_detail_variables_of_wings_and_sidepods():
    w = device_from_config("front", wing_cfg(), ".")
    detail = {v[0]: v for v in w.detail_variables()}
    assert list(detail) == [".span", ".height", ".x", ".main.chord", ".flap1.chord", ".flap1.gap", ".flap1.overlap",
                            ".flap2.chord", ".flap2.gap", ".flap2.overlap"]
    assert [v[0] for v in w.design_variables(detail=True)] == ["", ".flap1", ".flap2"] + list(detail)
    assert detail[".flap1.gap"][1] == "elements.flap1.gap_mm" and detail[".flap1.gap"][3] == "mm"
    lo, hi = detail[".flap1.gap"][2]
    assert lo == pytest.approx(0.01 * 175, abs=0.06) and hi == pytest.approx(0.07 * 175, abs=0.06)
    assert detail[".span"][2] == pytest.approx((0.8, 1.2)) and detail[".x"][2] == pytest.approx((-750, -550))
    # every detail variable moves the forces the way the geometry says
    cla0 = float(w.coefficients(GEO, 35, 45, 0, 0)[0])
    wide = dict(wing_cfg(), span_m=1.2)
    assert float(device_from_config("front", wide, ".").coefficients(GEO, 35, 45, 0, 0)[0]) > cla0
    long_flap = device_from_config("front", wing_cfg(flap1__chord_mm=100), ".")
    assert float(long_flap.coefficients(GEO, 35, 45, 0, 0)[0]) > cla0
    shut = device_from_config("front", wing_cfg(flap1__gap_mm=1.0, flap2__gap_mm=1.0), ".")
    assert float(shut.coefficients(GEO, 35, 45, 0, 0)[0]) < cla0
    # out of ground effect the height changes nothing: no height variable
    rear = device_from_config("rear", dict(wing_cfg(), height_mm=950, x_mm=1800), ".")
    assert not rear.ground_effect and ".height" not in [v[0] for v in rear.detail_variables()]
    # a simple wing has its total chord, a sidepod its radiator core
    simple = device_from_config("w", {"type": "wing", "span_m": 1.0, "chord_m": 0.3, "height_mm": 900}, ".")
    assert [v[0] for v in simple.detail_variables()] == [".span", ".x", ".chord"]
    pod = device_from_config("pod", {"type": "sidepod", "core_area_m2": 0.04, "exit_area_m2": 0.03}, ".")
    assert [(v[0], v[1], v[3]) for v in pod.design_variables(detail=True)] == [
        ("", "exit_area_m2", "m2"), (".core_area", "core_area_m2", "m2")]
    assert pod.design_variables() == pod.design_variables(detail=False) and len(pod.design_variables()) == 1


def test_detail_ranges_from_the_vehicle_file():
    c = dict(wing_cfg(), detail_ranges={"span": [0.9, 1.4], "flap1.gap": [3, 9]})
    w = device_from_config("front", c, ".")
    detail = {v[0]: v[2] for v in w.detail_variables()}
    assert detail[".span"] == (0.9, 1.4) and detail[".flap1.gap"] == (3.0, 9.0)
    assert detail[".flap2.gap"] != (3.0, 9.0)  # only the named variable
    with pytest.raises(ConfigError) as e:
        device_from_config("front", dict(wing_cfg(), detail_ranges={"flap9.gap": [1, 2]}), ".")
    assert e.value.path == "aero.components.front.detail_ranges.flap9.gap" and "flap1.gap" in str(e.value)
    with pytest.raises(ConfigError) as e:
        device_from_config("front", dict(wing_cfg(), detail_ranges={"span": [1.2, 1.0]}), ".")
    assert e.value.path == "aero.components.front.detail_ranges.span"
    cfg = load_vehicle_config(CAR)
    cfg["aero"]["components"]["rear"]["detail_ranges"] = {"span": [0.9, 1.1]}
    assert not unknown_keys(cfg)


def test_cfd_section_wing_keeps_its_section_in_the_detail_variables():
    c = wing_cfg()
    c["elements"].pop("flap2")
    c["section_data"] = str(FRONT_CSV)
    w = device_from_config("front", c, ".")
    assert w.cfd and [v[0] for v in w.detail_variables()] == [".span", ".height", ".x"]


def test_stall_constraint():
    from aerolap.analysis.objectives import Objective

    cfg = load_vehicle_config(CAR)
    spec = {"type": "skidpad_time", "level": "L2"}
    ok = Objective(spec, ".", {"max_stall_ratio": 0.98}, SolverOptions.coarse())(cfg)
    assert 0.5 < ok.metrics["wing_stall_ratio_max"] < 0.98 and ok.violations["wing_stall"] < 0
    stalled = with_overrides(cfg, {"aero.components.rear.elements.flap2.deflection_deg": 60})
    ev = Objective(spec, ".", {"max_stall_ratio": 0.98}, SolverOptions.coarse())(stalled)
    assert ev.metrics["wing_stall_ratio_max"] > 1.0 and ev.violations["wing_stall"] > 0 and not ev.feasible


def test_front_wing_clearance_constraint():
    from aerolap.analysis.objectives import Objective

    cfg = load_vehicle_config(CAR)
    spec = {"type": "lap_time", "track": str(TRACK), "level": "L4"}
    obj = Objective(spec, ".", {"min_wing_clearance_mm": 15}, SolverOptions.coarse())
    high = obj(cfg)
    # braking dive and roll take the wing below its static height
    assert 15 < high.metrics["wing_min_clearance_mm"] < 80 and high.violations["wing_clearance"] < 0
    low = obj(with_overrides(cfg, {"aero.components.front.height_mm": 25}))
    assert low.metrics["wing_min_clearance_mm"] == pytest.approx(high.metrics["wing_min_clearance_mm"] - 55, abs=1.0)
    assert low.violations["wing_clearance"] > 0 and not low.feasible


def test_element_input_errors_name_the_parameter():
    with pytest.raises(ConfigError) as e:
        device_from_config("front", wing_cfg(flap2__gap_mm=-1), ".")
    assert e.value.path == "aero.components.front.elements.flap2.gap_mm"
    bad = wing_cfg()
    bad["elements"] = {"main": {"chord_mm": 175}, "flap2": {"chord_mm": 60, "deflection_deg": 30}}
    with pytest.raises(ConfigError) as e:
        device_from_config("front", bad, ".")
    assert e.value.path == "aero.components.front.elements" and "flap1" in str(e.value)
    bad = wing_cfg()
    bad["elements"]["main"]["deflection_deg"] = 3
    with pytest.raises(ConfigError, match="aoa_deg"):
        device_from_config("front", bad, ".")
    bad = wing_cfg()
    del bad["elements"]["flap1"]["deflection_deg"]
    with pytest.raises(ConfigError) as e:
        device_from_config("front", bad, ".")
    assert e.value.path.endswith("flap1.deflection_deg")
    cfg = load_vehicle_config(CAR)
    cfg["aero"]["components"]["front"]["elements"]["flap1"]["gap"] = 5
    assert any("elements.flap1.gap'" in m and "gap_mm" in m for m in unknown_keys(cfg))


def test_element_values_pick_the_cfd_geometry():
    c = wing_cfg()
    c["elements"].pop("flap2")
    c["section_data"] = str(FRONT_CSV)
    w = device_from_config("front", c, ".")
    assert w.cfd and w.section.geometry_id == "4412_f80d15g6"
    c["elements"]["flap1"]["deflection_deg"] = 17
    with pytest.raises(ConfigError) as e:
        device_from_config("front", c, ".")
    assert e.value.path == "aero.components.front.elements" and "closest" in str(e.value)


def test_sidepod_cooling_drag_and_heat():
    p = Sidepod("pod", 0.04, 0.03)
    big = Sidepod("pod", 0.04, 0.06)
    assert big.flow_ratio > p.flow_ratio and big.cooling_cda > p.cooling_cda
    r = 0.04 / 0.03
    f = np.sqrt(0.9 / (8.0 + r * r))
    assert p.flow_ratio == pytest.approx(f) and p.cooling_cda == pytest.approx(2 * 0.04 * f * (1 - r * f))
    assert float(p.heat_rejection_kW(20.0)) == pytest.approx(2 * float(p.heat_rejection_kW(10.0)))
    assert float(big.heat_rejection_kW(15.0)) > float(p.heat_rejection_kW(15.0))
    cla, cda = p.coefficients(GEO, np.array([30.0, 40.0]), 45.0, 0.0, 0.0)
    assert cda.shape == (2,) and float(cda[0]) == pytest.approx(p.body_cda + p.cooling_cda)
    with pytest.raises(ConfigError) as e:
        device_from_config("pod", {"type": "sidepod", "core_area_m2": 0.04, "exit_area_m2": 0.0}, ".")
    assert e.value.path == "aero.components.pod.exit_area_m2"


def test_fsk2026_layout_totals_and_cooling_constraint():
    from aerolap.analysis.cooling import cooling_balance
    from aerolap.analysis.objectives import Objective

    cfg = load_vehicle_config(CAR)
    assert not unknown_keys(cfg)
    car = VehicleModel(cfg)
    cf, cr, cd, _ = car.aero.coefficients(0, 35.0, 45.0, 0.0, 0.0)
    assert float(cf + cr) == pytest.approx(0.997, rel=0.01) and float(cd) == pytest.approx(0.470, rel=0.01)
    dv = design_variables(cfg)
    assert {"front", "front.flap1", "front.flap2", "side", "side.flap1", "sidepod", "rear", "rear.flap1",
            "rear.flap2"} == set(dv)
    res = QSSSolver(SolverOptions.coarse()).solve(car, Track.from_file(TRACK), start="flying")
    cb = cooling_balance(car, res)
    assert cb is not None and cb.produced_kW > 0 and cb.rejected_kW > 0 and "margin" in cb.report()
    spec = {"type": "lap_time", "track": str(TRACK), "level": "L2"}
    ok = Objective(spec, ".", {"cooling_margin_kW": 0.0}, SolverOptions.coarse())(cfg)
    assert "cooling" in ok.violations and ok.metrics["cooling_margin_kW"] == pytest.approx(cb.margin_kW, rel=0.05)
    small = with_overrides(cfg, {"aero.components.sidepod.exit_area_m2": 0.004})
    ev = Objective(spec, ".", {"cooling_margin_kW": 0.0}, SolverOptions.coarse())(small)
    assert ev.metrics["cooling_margin_kW"] < ok.metrics["cooling_margin_kW"] and ev.violations["cooling"] > ok.violations["cooling"]
