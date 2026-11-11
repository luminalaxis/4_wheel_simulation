"""Battery capacity / power limits, lap energy breakdown and the energy-limited endurance."""

import numpy as np
import pytest

from aerolap.analysis.energy import endurance_energy, energy_breakdown, energy_sweep
from aerolap.analysis.objectives import Objective
from aerolap.config import with_overrides
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

COARSE = SolverOptions.coarse()


@pytest.fixture(scope="module")
def loop():
    """Short closed circuit with straights and slow corners (brakes, drag and acceleration all matter)."""
    segs = [("straight", 90.0, 0.0, "l"), ("corner", 25.0, 9.0, "l"), ("straight", 50.0, 0.0, "l"),
            ("corner", 20.0, 12.0, "l"), ("straight", 90.0, 0.0, "l"), ("corner", 25.0, 9.0, "l"),
            ("straight", 50.0, 0.0, "l"), ("corner", 20.0, 12.0, "l")]
    return Track.from_segments(segs, name="test loop", closed=True)


def _lap(cfg, track):
    car = VehicleModel(cfg)
    return car, QSSSolver(COARSE).solve(car, track, start="flying")


def test_battery_parameters(ev_cfg):
    car = VehicleModel(with_overrides(ev_cfg, {"battery.capacity_kWh": 7.0, "battery.usable_fraction": 0.9,
                                               "battery.max_power_kW": 60.0}))
    pt = car.powertrain
    assert pt.usable_energy == pytest.approx(7.0 * 0.9 * 3.6e6)
    assert pt.power_limit == pytest.approx(60e3)  # pack limit below the 80 kW rule limit
    none = VehicleModel(with_overrides(ev_cfg, {"battery.capacity_kWh": None}))
    assert none.powertrain.usable_energy is None


def test_pack_power_limit_caps_battery_power(ev_cfg):
    cfg = with_overrides(ev_cfg, {"battery.max_power_kW": 50.0})
    _, res = _lap(cfg, Track.straight(150.0))
    assert np.nanmax(res.channels["battery_power_kW"]) <= 50.0 * 1.01


def test_breakdown_balances_and_more_drag_costs_energy(ev_cfg, loop):
    car, res = _lap(ev_cfg, loop)
    b = energy_breakdown(car, res)
    assert b["battery_kWh"] == pytest.approx(b["wheel_work_kWh"] + b["powertrain_losses_kWh"]
                                             - b["regen_returned_kWh"], rel=1e-9)
    assert abs(b["kinetic_change_kWh"]) < 0.05 * b["wheel_work_kWh"]  # periodic flying lap
    assert b["aero_drag_kWh"] > 0 and b["powertrain_losses_kWh"] > 0
    car2, res2 = _lap(with_overrides(ev_cfg, {"aero.cda_scale": 1.5}), loop)
    b2 = energy_breakdown(car2, res2)
    assert b2["aero_drag_kWh"] > 1.3 * b["aero_drag_kWh"]
    assert b2["battery_kWh"] > b["battery_kWh"]


def test_regen_returns_energy_without_changing_the_lap(ev_cfg, loop):
    _, plain = _lap(ev_cfg, loop)
    _, regen = _lap(with_overrides(ev_cfg, {"battery.regen_power_kW": 30.0}), loop)
    assert regen.lap_time == pytest.approx(plain.lap_time, rel=1e-9)
    assert regen.energy_J < plain.energy_J
    assert np.nanmax(regen.channels["regen_power_kW"]) <= 30.0 + 1e-6


def test_endurance_derates_power_to_fit_the_battery(ev_cfg, loop):
    free = endurance_energy(with_overrides(ev_cfg, {"battery.capacity_kWh": None}), loop, 3.0, opts=COARSE)
    assert free.power_cap_kW is None and free.usable_kWh is None
    need = free.unlimited_required_kWh
    big = endurance_energy(with_overrides(ev_cfg, {"battery.capacity_kWh": 2 * need}), loop, 3.0, opts=COARSE)
    assert big.power_cap_kW is None and big.margin_kWh > 0
    small_cfg = with_overrides(ev_cfg, {"battery.capacity_kWh": 0.75 * need, "battery.usable_fraction": 1.0})
    en = endurance_energy(small_cfg, loop, 3.0, opts=COARSE)
    assert en.feasible and en.power_cap_kW < 80.0
    assert en.required_kWh <= 0.75 * need + 0.01
    assert en.lap_time_s > free.unlimited_lap_time_s
    check = endurance_energy(small_cfg, loop, 3.0, opts=COARSE, derate=False)
    assert not check.feasible and check.power_cap_kW is None
    assert "does NOT fit" in check.report()


def test_endurance_objective_and_battery_constraint(ev_cfg, loop, tmp_path):
    path = tmp_path / "loop.csv"
    rows = ["type,length_m,radius_m,direction", "straight,300,0,left"]
    path.write_text("\n".join(rows) + "\n")
    spec = {"type": "skidpad_time", "level": "L4"}
    need = endurance_energy(ev_cfg, loop, 3.0, opts=COARSE, derate=False).unlimited_required_kWh
    obj = Objective(spec, tmp_path, {"battery_energy": {"track": str(path), "distance_km": 3.0}}, COARSE)
    obj.energy_track = loop  # use the fixture circuit
    small = obj(with_overrides(ev_cfg, {"battery.capacity_kWh": 0.5 * need, "battery.usable_fraction": 1.0}))
    assert small.violations["battery_energy"] > 0 and not small.feasible
    large = obj(with_overrides(ev_cfg, {"battery.capacity_kWh": 2.0 * need, "battery.usable_fraction": 1.0}))
    assert large.violations["battery_energy"] < 0
    endu = Objective({"type": "endurance_time", "track": str(path), "distance_km": 3.0}, tmp_path, opts=COARSE)
    endu.track = loop
    ev = endu(with_overrides(ev_cfg, {"battery.capacity_kWh": 0.75 * need, "battery.usable_fraction": 1.0}))
    assert ev.metrics["endurance_power_cap_kW"] < 80.0
    assert ev.value == pytest.approx(ev.metrics["endurance_lap_s"] * round(3000.0 / loop.length))


def test_power_cap_is_tight(ev_cfg, loop):
    need = endurance_energy(ev_cfg, loop, 3.0, opts=COARSE, derate=False).unlimited_required_kWh
    cfg = with_overrides(ev_cfg, {"battery.capacity_kWh": 0.8 * need, "battery.usable_fraction": 1.0})
    en = endurance_energy(cfg, loop, 3.0, opts=COARSE)
    assert en.feasible and en.required_kWh <= en.usable_kWh + 0.01
    # a slightly higher cap would not fit any more
    over = endurance_energy(with_overrides(cfg, {"powertrain.ev.power_limit_kW": en.power_cap_kW + 1.0}), loop, 3.0,
                            opts=COARSE, derate=False)
    assert over.unlimited_required_kWh > en.usable_kWh


def test_relative_energy_sweep_keeps_individual_values(ev_cfg, loop):
    cfg = with_overrides(ev_cfg, {"aero.cla_scale": 0.8, "aero.cda_scale": 1.2, "battery.capacity_kWh": None})
    rel = energy_sweep(cfg, loop, "aero.cla_scale,aero.cda_scale", [1.0], 3.0, opts=COARSE, relative=True)
    same = endurance_energy(cfg, loop, 3.0, opts=COARSE)
    assert rel[0][1].total_time_s == pytest.approx(same.total_time_s, rel=1e-12)
    absolute = energy_sweep(cfg, loop, "aero.cla_scale,aero.cda_scale", [1.0], 3.0, opts=COARSE)
    assert absolute[0][1].total_time_s != pytest.approx(same.total_time_s, rel=1e-9)
