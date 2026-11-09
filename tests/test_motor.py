import math

import numpy as np
import pytest

from aerolap.config import with_overrides
from aerolap.motors import Motor
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.track import Track
from aerolap.vehicle import VehicleModel


def _emrax(variant="MV", v=630.0, **kw):
    return Motor.from_config({"motor": "emrax_228", "motor_variant": variant, "motor_cooling": "LC",
                              "dc_voltage_V": v, **kw})


def test_emrax_228_matches_datasheet_peak_points():
    mv = _emrax("MV", 630.0)
    p = lambda m, rpm: float(m.max_torque(rpm)) * rpm * 2 * math.pi / 60 / 1000  # noqa: E731
    assert float(mv.max_torque(1000)) == pytest.approx(220.0)
    assert p(mv, 5500) == pytest.approx(124.0, rel=0.02)  # "124 kW at 5500 rpm"
    hv = _emrax("HV", 830.0)
    assert p(hv, 4500) == pytest.approx(104.0, rel=0.02)  # "104 kW at 4500 rpm" (voltage limited)
    assert float(mv.max_torque(6600)) == 0.0  # limiting speed 6500 rpm
    assert mv.rotor_inertia == pytest.approx(0.0398)


def test_lower_bus_voltage_moves_base_speed_down():
    full, low = _emrax("MV", 630.0), _emrax("MV", 450.0)
    assert low.base_rpm == pytest.approx(8.68 * 450.0)
    assert float(low.max_torque(5000)) < float(full.max_torque(5000))
    assert float(low.max_torque(3000)) == float(full.max_torque(3000))


def test_controller_current_limits_torque():
    m = _emrax("MV", 630.0, controller_current_A=250.0)
    assert m.torque_limit == pytest.approx(0.64 * 250.0)


def test_efficiency_map_shape():
    m = _emrax("MV", 630.0)
    assert float(m.efficiency(3000, 110)) == pytest.approx(0.96, abs=0.005)  # datasheet peak 96 %
    assert float(m.efficiency(1000, 20)) < 0.91  # 86-90 % region
    assert float(m.efficiency(500, 20)) < 0.87
    assert float(m.efficiency(2500, 100)) > float(m.efficiency(2500, 30))


def test_battery_power_limit_and_rotor_inertia(ev_cfg):
    car = VehicleModel(ev_cfg)
    pt = car.powertrain
    v = np.array([25.0, 30.0])
    f, _ = pt.max_force(v)
    assert np.allclose(pt.battery_power(v, f), 80e3, rtol=1e-3)  # 80 kW accumulator limit
    assert pt.m_rot == pytest.approx(0.0398 * 3.6 ** 2 / 0.203 ** 2, rel=1e-6)
    # rotor inertia costs acceleration when the motor (not the tyres) is the limit
    track = Track.straight(75.0)
    no_inertia = VehicleModel(with_overrides(ev_cfg, {"powertrain.ev.rotor_inertia_kgm2": 0.0}))
    t_with = QSSSolver(SolverOptions.coarse()).solve(car, track, start="standing").lap_time
    t_without = QSSSolver(SolverOptions.coarse()).solve(no_inertia, track, start="standing").lap_time
    assert t_with > t_without


def test_lap_energy_uses_motor_efficiency(ev_cfg):
    car = VehicleModel(ev_cfg)
    res = QSSSolver(SolverOptions.coarse()).solve(car, Track.straight(75.0), start="standing")
    ch = res.channels
    assert {"battery_power_kW", "motor_rpm", "motor_torque_Nm", "motor_efficiency"} <= set(ch)
    assert np.nanmax(ch["battery_power_kW"]) <= 80.0 * 1.01
    mech = np.nansum(ch["drive_power_kW"][:-1] * np.diff(res.t)) * 1000.0
    assert res.energy_J > mech  # losses on top of the energy at the wheels
