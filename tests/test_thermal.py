import math

import numpy as np
import pytest

from aerolap.analysis.energy import endurance_energy
from aerolap.analysis.energy_strategy import endurance_strategy
from aerolap.battery import BatteryPack
from aerolap.cli import main as cli_main
from aerolap.config import ConfigError, load_vehicle_config, unknown_keys, with_overrides
from aerolap.solvers.ggv import SolverOptions
from aerolap.thermal import LumpedThermal, motor_thermal
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

from .conftest import DATA

FSK_TRACK = DATA / "tracks" / "fsk2026_endurance_smoothed.csv"


@pytest.fixture(scope="module")
def thermal_cfg():
    return load_vehicle_config(DATA / "vehicles" / "fsk2026_thermal.yaml", warn=False)


def test_lumped_model_matches_the_analytic_solution():
    m = LumpedThermal("x", heat_capacity=40e3, conductance=25.0, ambient=25.0, start=30.0, limit=60.0)
    q, dt = np.full(1000, 800.0), np.full(1000, 1.5)
    end, peak = m.advance(30.0, q, dt)
    t = 1500.0
    exact = 25.0 + 800.0 / 25.0 + (30.0 - 25.0 - 800.0 / 25.0) * math.exp(-t * 25.0 / 40e3)
    assert end == pytest.approx(exact, rel=1e-12) and peak == pytest.approx(end)
    adiabatic = LumpedThermal("x", 40e3, 0.0, 25.0, 30.0, None)
    end, _ = adiabatic.advance(30.0, q, dt)
    assert end == pytest.approx(30.0 + 800.0 * 1500.0 / 40e3)
    # temperature-dependent heat (heat_fn) with a constant function equals the plain trace
    end2, _ = m.advance(30.0, q, dt, heat_fn=lambda i, temp: 800.0)
    assert end2 == pytest.approx(exact, rel=1e-12)


def test_cell_resistance_tables(thermal_cfg):
    plain = BatteryPack.from_config(load_vehicle_config(DATA / "vehicles" / "fsk2026.yaml", warn=False)["battery"])
    assert plain.r_factor() == 1.0
    assert plain.resistance == pytest.approx(132 * 0.013 / 4 + 0.030)
    hot = BatteryPack.from_config({**thermal_cfg["battery"], "temperature_C": 45.0, "soc": 0.8})
    cold = BatteryPack.from_config({**thermal_cfg["battery"], "temperature_C": 0.0, "soc": 0.8})
    assert hot.r_factor() == pytest.approx(0.8) and cold.r_factor() == pytest.approx(2.0)
    assert cold.resistance > plain.resistance > hot.resistance
    assert hot.cell_resistance(soc=0.1) > hot.cell_resistance(soc=0.8)
    with pytest.raises(ConfigError):
        BatteryPack.from_config({**thermal_cfg["battery"], "cell": {**thermal_cfg["battery"]["cell"],
                                                                    "resistance_temp": [[0, -1.0]]}})


def test_motor_thermal_resistance_comes_from_the_continuous_rating(thermal_cfg):
    car = VehicleModel(thermal_cfg)
    m = car.powertrain.motor
    th = motor_thermal(thermal_cfg["powertrain"]["ev"], m)
    rpm = m.cont_power / m.cont_torque * 60.0 / (2.0 * math.pi)
    q = float(m.losses(rpm, m.cont_torque))
    end, _ = th.advance(th.start, np.full(400, q), np.full(400, 30.0))  # long enough for steady state
    assert end == pytest.approx(th.limit, abs=0.05)
    assert 100.0 < th.time_constant < 900.0


def test_thermal_vehicle_file_is_clean(thermal_cfg):
    assert not unknown_keys(thermal_cfg)


def test_endurance_temperatures_and_temperature_limited_cap(thermal_cfg):
    track = Track.from_file(FSK_TRACK)
    opts = SolverOptions.coarse()
    en = endurance_energy(thermal_cfg, track, 22.0, opts=opts)
    th = en.thermal
    assert th is not None and th.cell_peak is not None and th.motor_peak is not None
    assert th.cell.start < th.cell_peak <= th.cell.limit + 0.05
    assert all(st.temp_start is not None for st in en.stages)
    assert en.stages[-1].temp_end > en.stages[0].temp_start
    assert "cell temperature" in en.report() and "motor winding" in en.report()
    # weak cooling: the cell temperature, not the battery energy, sets the power cap
    weak = with_overrides(thermal_cfg, {"battery.thermal.cooling_W_per_K": 5.0})
    en_w = endurance_energy(weak, track, 22.0, opts=opts)
    assert en_w.limited_by == "cells" and en_w.feasible
    assert en_w.thermal.cell_peak == pytest.approx(60.0, abs=0.5)
    assert en_w.required_kWh < en_w.usable_kWh
    assert en_w.power_cap_kW < en.power_cap_kW
    # without derating the run overheats and says so
    off = with_overrides(weak, {"battery.thermal.derate": False})
    en_o = endurance_energy(off, track, 22.0, opts=opts)
    assert en_o.thermal.over_limit == ["cells"] and "OVER" in en_o.report()


def test_energy_optimal_driving_respects_the_cell_limit(thermal_cfg):
    # a warm pack with weak cooling: the cell limit (not the battery energy) is what binds
    cfg = with_overrides(thermal_cfg, {"battery.thermal.cooling_W_per_K": 15.0, "battery.thermal.start_C": 45.0})
    st = endurance_strategy(cfg, Track.from_file(FSK_TRACK), 10.0, opts=SolverOptions.coarse(), pareto=False)
    assert st.cap.limited_by == "cells"
    assert st.mu > 0.0  # the cell heat had to be priced
    assert st.thermal.cell_peak <= st.thermal.cell.limit + 0.05
    assert st.total_s <= st.cap.total_time_s + 0.5  # not slower than the power cap (which also keeps the limit)
    assert "cell heat priced" in st.report()


def test_energy_cli_writes_the_temperature_plot(tmp_path):
    assert cli_main(["energy", str(DATA / "vehicles" / "fsk2026_thermal.yaml"), str(FSK_TRACK), "--coarse",
                     "--distance-km", "10", "--out", str(tmp_path)]) == 0
    assert (tmp_path / "thermal.png").exists()
    assert "cell temperature" in (tmp_path / "energy.txt").read_text()
