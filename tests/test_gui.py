"""GUI task layer (no display needed) and, when tkinter + a display exist, the window itself."""

from pathlib import Path

import pytest

from aerolap.config import load_vehicle_config
from aerolap.gui_tasks import TASKS, editable_params

ROOT = Path(__file__).resolve().parents[1]
CAR = ROOT / "data" / "vehicles" / "fsk2026.yaml"
TRACK = ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv"


def test_editable_params_cover_aero_components():
    paths = [p for _, p in editable_params(load_vehicle_config(CAR))]
    assert "mass.total_kg" in paths and "battery.cell.capacity_Ah" in paths and "battery.soc" in paths
    assert {f"aero.components.{n}.scale" for n in ("front", "side", "rear")} <= set(paths)


@pytest.mark.parametrize("key", ["lap", "aero_split"])
def test_gui_tasks_write_text_and_graphs(key, tmp_path):
    _, fn = TASKS[key]
    res = fn(load_vehicle_config(CAR), str(TRACK), "L4", True, tmp_path / key, log=lambda m: None,
             objective="lap_time", bounds=(0.8, 1.2), max_evals=6, points=3)
    assert res.text and res.images and all(Path(p).exists() for p in res.images)


def test_window_builds():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display")
    from aerolap.gui import App

    app = App(root)
    assert app.param_vars and app.current_cfg()["mass"]["total_kg"] == 310.0
    root.destroy()
