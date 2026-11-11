"""Cell-based accumulator model: voltage sag, current limits, I^2R loss, SOC stages."""

from pathlib import Path

import numpy as np
import pytest

from aerolap.analysis.energy import endurance_energy
from aerolap.battery import BatteryPack
from aerolap.config import load_vehicle_config, with_overrides
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

ROOT = Path(__file__).resolve().parents[1]
CAR = ROOT / "data" / "vehicles" / "fsk2026.yaml"
COARSE = SolverOptions.coarse()


@pytest.fixture(scope="module")
def cfg():
    return load_vehicle_config(CAR)


def test_pack_equations(cfg):
    pack = BatteryPack.from_config(cfg["battery"])
    v = float(pack.ocv(1.0))
    assert v == pytest.approx(132 * 4.18)
    assert pack.resistance == pytest.approx(132 * 0.013 / 4 + 0.03)
    i = pack.max_current(1.0)
    assert i == pytest.approx(4 * 45)
    assert pack.max_power(1.0) == pytest.approx(v * i - pack.resistance * i * i)
    p = 60e3
    i60 = float(pack.current(p, 1.0))
    assert v * i60 - pack.resistance * i60 ** 2 == pytest.approx(p, rel=1e-9)  # terminal power
    assert float(pack.terminal_voltage(p, 1.0)) == pytest.approx(v - pack.resistance * i60)
    assert float(pack.current(-20e3, 1.0)) < 0  # regen charges
    assert pack.usable_energy / 3.6e6 == pytest.approx(7.48, abs=0.05)


def test_current_limit_caps_power(cfg):
    weak = VehicleModel(with_overrides(cfg, {"battery.cell.max_current_A": 25})).powertrain
    assert weak.power_limit < 80e3 and weak.power_limit == pytest.approx(weak.pack.max_power())


def test_low_soc_lowers_voltage_power_and_speed(cfg):
    hi = VehicleModel(with_overrides(cfg, {"battery.soc": 1.0}))
    lo = VehicleModel(with_overrides(cfg, {"battery.soc": 0.15}))
    assert lo.powertrain.pack_voltage < hi.powertrain.pack_voltage
    assert lo.powertrain.motor.base_rpm < hi.powertrain.motor.base_rpm
    assert lo.powertrain.power_limit < hi.powertrain.power_limit
    straight = Track.straight(150.0)
    t_hi = QSSSolver(COARSE).solve(hi, straight, start="standing").lap_time
    t_lo = QSSSolver(COARSE).solve(lo, straight, start="standing").lap_time
    assert t_lo > t_hi


def test_lap_energy_includes_pack_loss(cfg):
    car = VehicleModel(cfg)
    res = QSSSolver(COARSE).solve(car, Track.straight(75.0), start="standing")
    ch = res.channels
    assert {"pack_current_A", "pack_voltage_V", "battery_loss_kW"} <= set(ch)
    terminal = np.sum(0.5 * (ch["battery_power_kW"][1:] + ch["battery_power_kW"][:-1]) * np.diff(res.t)) * 1e3
    assert res.energy_J > terminal and np.max(ch["battery_loss_kW"]) > 0
    assert np.min(ch["pack_voltage_V"]) < float(car.powertrain.pack.ocv())


def test_endurance_soc_stages(cfg):
    loop = Track.from_segments([("straight", 80.0, 0, ""), ("corner", 30.0, 10.0, "left")] * 2, closed=True)
    en = endurance_energy(cfg, loop, 6.0, opts=COARSE, derate=False)
    st = en.stages
    assert len(st) == 3 and st[0].soc_start == pytest.approx(1.0)
    assert all(a.soc_end == pytest.approx(b.soc_start) for a, b in zip(st, st[1:]))
    assert st[-1].soc_end < st[0].soc_end < 1.0
    assert st[-1].power_kW <= st[0].power_kW and st[-1].lap_time_s >= st[0].lap_time_s
    assert en.total_time_s == pytest.approx(sum(s.lap_time_s * s.laps for s in st))
    assert "SOC stages" in en.report()
    assert en.usable_kWh == pytest.approx(7.48, abs=0.05)  # from the start SOC, not a stage SOC


def test_without_cell_section_nothing_changes(ev_cfg):
    car = VehicleModel(ev_cfg)
    assert car.powertrain.pack is None and car.powertrain.power_limit == pytest.approx(80e3)


def test_small_pack_endurance_does_not_crash(cfg):
    """A pack too small for the run: the SOC stages go below soc_min - the cap search must still work."""
    loop = Track.from_segments([("straight", 80.0, 0, ""), ("corner", 30.0, 10.0, "left")] * 2, closed=True)
    small = with_overrides(cfg, {"battery.cell.capacity_Ah": 0.6})
    flat = endurance_energy(small, loop, 6.0, opts=COARSE, derate=False)
    assert not flat.feasible and flat.soc_end < small["battery"]["soc_min"]
    en = endurance_energy(small, loop, 6.0, opts=COARSE)
    assert en.power_cap_kW is not None and en.power_cap_kW < 80.0
