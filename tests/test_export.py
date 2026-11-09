import csv

import numpy as np
import pytest
from scipy.io import loadmat

from aerolap.export import export_ggv, lap_to_mat
from aerolap.solvers.ggv import SolverOptions
from aerolap.solvers.qss import QSSSolver
from aerolap.track import Track
from aerolap.vehicle import VehicleModel


def test_ggv_export_csv_and_mat(ev_cfg, tmp_path):
    car = VehicleModel(ev_cfg)
    solver = QSSSolver(SolverOptions.coarse())
    ggvs = [solver.ggv(car, m) for m in range(len(car.mode_names))]
    files = export_ggv(ggvs, car.mode_names, tmp_path, {"vehicle": car.name})
    assert {f.name for f in files} == {"ggv.csv", "ggv.mat"}
    with open(tmp_path / "ggv.csv", newline="") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == sum(len(g.v) * len(g.fracs) for g in ggvs)
    S = loadmat(tmp_path / "ggv.mat", simplify_cells=True)
    g0 = ggvs[0]
    m = S["ggv"][car.mode_names[0]]
    assert m["ax_acc"].shape == (len(g0.v), len(g0.fracs))
    np.testing.assert_allclose(m["ay_max"], g0.ay_max)
    i = len(g0.v) // 2
    assert m["ax_brk"][i, 0] == pytest.approx(g0.brk_at(float(g0.v[i]), 0.0))
    assert S["meta"]["vehicle"] == car.name


def test_lap_to_mat(ev_cfg, tmp_path):
    car = VehicleModel(ev_cfg)
    res = QSSSolver(SolverOptions.coarse()).solve(car, Track.straight(75.0), start="standing")
    lap_to_mat(res, tmp_path / "trace.mat")
    S = loadmat(tmp_path / "trace.mat", simplify_cells=True)
    np.testing.assert_allclose(S["lap"]["v"], res.v)
    assert S["meta"]["lap_time_s"] == pytest.approx(res.lap_time)
    assert "battery_power_kW" in S["lap"]
