"""Per-device downforce from the aero variables."""

from pathlib import Path

import pytest

from aerolap.analysis.downforce import device_downforce
from aerolap.config import load_vehicle_config, with_overrides

CAR = Path(__file__).resolve().parents[1] / "data" / "vehicles" / "fsk2026.yaml"


def test_rows_add_up_and_scale_with_v2():
    res = device_downforce(load_vehicle_config(CAR), [50.0, 100.0])
    rows = {r["name"]: r for r in res["rows"]}
    assert {"front", "side", "rear"} <= set(rows)
    for i in range(2):
        assert sum(r["downforce"][i] for r in res["rows"]) == pytest.approx(res["total"]["downforce"][i])
    assert rows["rear"]["downforce"][1] == pytest.approx(4.0 * rows["rear"]["downforce"][0])
    assert res["total"]["cla"] == pytest.approx(0.997, abs=0.01)


def test_aero_variables_change_the_right_device():
    cfg = load_vehicle_config(CAR)
    base = {r["name"]: r["cla"] for r in device_downforce(cfg, [60.0])["rows"]}
    cfg2 = with_overrides(cfg, {"aero.components.rear.aoa_deg": 14.0})
    new = {r["name"]: r["cla"] for r in device_downforce(cfg2, [60.0])["rows"]}
    assert new["rear"] > base["rear"]
    assert new["front"] == pytest.approx(base["front"])
