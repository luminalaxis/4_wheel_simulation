import numpy as np
import pytest

from aerolap.analysis.energy_strategy import LapDP, endurance_strategy, lift_points
from aerolap.cli import main as cli_main
from aerolap.config import load_vehicle_config, with_overrides
from aerolap.gui_tasks import TASKS
from aerolap.solvers.ggv import SolverOptions
from aerolap.track import Track
from aerolap.vehicle import VehicleModel

from .conftest import DATA

ENDURANCE = DATA / "tracks" / "fsae_endurance_example.csv"


@pytest.fixture(scope="module")
def ev_dp():
    cfg = load_vehicle_config(DATA / "vehicles" / "fsae_ev_aero.yaml", warn=False)
    return LapDP(VehicleModel(cfg), Track.from_file(ENDURANCE), SolverOptions.coarse())


def test_zero_lambda_is_the_flat_out_lap(ev_dp):
    v, t, _ = ev_dp.solve(0.0)
    assert np.allclose(v, ev_dp.flat.v, atol=1e-9)
    assert t == pytest.approx(ev_dp.flat.lap_time, rel=1e-9)
    lap = ev_dp.lap(0.0)
    assert lap.lap_time == pytest.approx(ev_dp.flat.lap_time, rel=1e-9)


def test_lambda_trades_time_for_energy_and_profiles_stay_feasible(ev_dp):
    laps = [ev_dp.lap(lam) for lam in (0.0, 0.003, 0.01, 0.03)]
    times = [r.lap_time for r in laps]
    energy = [r.energy_J for r in laps]
    assert all(np.diff(times) > 0.0) and all(np.diff(energy) < 0.0)
    for r in laps:
        assert np.all(r.v <= ev_dp.flat.v + 1e-9)  # never above the flat-out (limit) profile
        assert r.v[0] == pytest.approx(r.v[-1], abs=0.15)  # periodic lap
        assert np.nanmax(r.channels["u_front"]) <= 1.0 + 1e-3 and np.nanmax(r.channels["u_rear"]) <= 1.0 + 1e-3
    lifts = lift_points(laps[2])
    assert len(lifts) >= 5 and all(p.coast_m >= 2.0 for p in lifts)
    # coasting is real coasting: no drive and no brake force on the stretch
    i = int(np.searchsorted(laps[2].s, lifts[0].s_lift + 0.5 * lifts[0].coast_m))
    assert laps[2].channels["drive_force_N"][i] < 30.0 and laps[2].channels["brake_force_N"][i] < 30.0


def test_energy_optimal_endurance_beats_the_power_cap(simple_cfg):
    cfg = with_overrides(simple_cfg, {"battery.capacity_kWh": 3.0, "battery.usable_fraction": 1.0,
                                      "tires.rolling_resistance": 0.015})
    track = Track.from_file(ENDURANCE)
    st = endurance_strategy(cfg, track, 10.0, opts=SolverOptions.coarse(), pareto=True)
    assert st.cap.power_cap_kW is not None  # flat out does not fit
    assert st.feasible and st.required_kWh <= 3.0 + 1e-6
    assert st.total_s < st.cap.total_time_s
    assert st.lam > 0.0 and st.lifts
    assert len(st.pareto_opt) >= 4 and len(st.pareto_cap) == 6
    text = st.report()
    assert "energy-optimal" in text and "lift-and-coast" in text


def test_no_battery_limit_means_flat_out(simple_cfg):
    st = endurance_strategy(simple_cfg, Track.from_file(ENDURANCE), 5.0, opts=SolverOptions.coarse(), pareto=False)
    assert st.lam == 0.0
    assert st.lap_s == pytest.approx(st.flat_lap_s)


def test_open_track_is_refused(simple_cfg):
    with pytest.raises(ValueError, match="closed"):
        LapDP(VehicleModel(simple_cfg), Track.from_file(DATA / "tracks" / "fsae_autocross_example.csv"))


def test_pack_model_uses_the_soc_stages(tmp_path):
    cfg = load_vehicle_config(DATA / "vehicles" / "fsk2026.yaml", warn=False)
    cfg = with_overrides(cfg, {"battery.series": 120})  # smaller pack: the endurance must save energy
    st = endurance_strategy(cfg, Track.from_file(DATA / "tracks" / "fsk2026_endurance_smoothed.csv"), 22.0,
                            opts=SolverOptions.coarse(), pareto=False)
    assert len(st.stages) == 3 and all(s.soc is not None for s in st.stages)
    assert st.total_s < st.cap.total_time_s
    assert "SOC stages" in st.report()


def test_energy_strategy_cli_and_gui_task(tmp_path, simple_cfg):
    car = DATA / "vehicles" / "fsae_ev_aero.yaml"
    assert cli_main(["energy-strategy", str(car), str(ENDURANCE), "--coarse", "--no-pareto", "--distance-km", "10",
                     "--set", "battery.capacity_kWh=3", "--out", str(tmp_path / "cli")]) == 0
    for f in ("energy_strategy.txt", "energy_strategy.png", "lift_points.csv", "trace_optimal.csv"):
        assert (tmp_path / "cli" / f).exists()
    cfg = with_overrides(simple_cfg, {"battery.capacity_kWh": 3.0})
    res = TASKS["strategy"][1](cfg, str(ENDURANCE), "L4", True, tmp_path / "gui", log=lambda m: None,
                               distance_km=10.0)
    assert res.images[0].exists() and "최적 주행" in res.summary
