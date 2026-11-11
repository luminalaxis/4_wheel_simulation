"""GUI task layer (no display needed) and, when tkinter + a display exist, the window itself."""

from pathlib import Path

import pytest

from aerolap.config import load_vehicle_config, with_overrides
from aerolap.gui_tasks import TASKS, TaskCancelled, changed_params, editable_params, save_vehicle_yaml

ROOT = Path(__file__).resolve().parents[1]
CAR = ROOT / "data" / "vehicles" / "fsk2026.yaml"
TRACK = ROOT / "data" / "tracks" / "fsk2026_endurance_smoothed.csv"
KW = dict(objective="lap_time", bounds=(0.8, 1.2), max_evals=6, points=3, sweep_path="mass.total_kg",
          sweep_lo=290.0, sweep_hi=330.0, sweep_n=3, sweep_objective="lap_time")


def test_editable_params_cover_aero_components():
    paths = [p for _, p in editable_params(load_vehicle_config(CAR))]
    assert "mass.total_kg" in paths and "battery.cell.capacity_Ah" in paths and "battery.soc" in paths
    assert {f"aero.components.{n}.scale" for n in ("front", "side", "rear")} <= set(paths)


@pytest.mark.parametrize("key", ["lap", "cornering", "sweep", "aero_split"])
def test_gui_tasks_write_text_graphs_and_a_summary(key, tmp_path):
    _, fn = TASKS[key]
    res = fn(load_vehicle_config(CAR), str(TRACK), "L4", True, tmp_path / key, log=lambda m: None, **KW)
    assert res.text and res.summary and res.images and all(Path(p).exists() for p in res.images)


def test_sweep_task_finds_the_lighter_car_faster(tmp_path):
    res = TASKS["sweep"][1](load_vehicle_config(CAR), str(TRACK), "L4", True, tmp_path, log=lambda m: None, **KW)
    assert "최적 290" in res.summary


def test_cancel_stops_a_long_task(tmp_path):
    def log(msg):
        raise TaskCancelled

    with pytest.raises(TaskCancelled):
        TASKS["sweep"][1](load_vehicle_config(CAR), str(TRACK), "L4", True, tmp_path, log=log, **KW)


def test_changed_params_and_yaml_header(tmp_path):
    base = load_vehicle_config(CAR)
    cfg = with_overrides(base, {"mass.total_kg": 300.0, "aero.components.front.scale": 1.5})
    text = changed_params(cfg, base)
    assert "mass.total_kg=300 (310)" in text and "aero.components.front.scale=1.5 (1)" in text
    assert changed_params(base, base) == ""
    out = tmp_path / "saved.yaml"
    save_vehicle_yaml(cfg, out, CAR)
    saved = out.read_text(encoding="utf-8")
    assert saved.startswith("# Imported from OptimumLap") and "ASSUMED" in saved
    again = load_vehicle_config(out)
    assert again["mass"]["total_kg"] == 300.0 and again["_warnings"] == []


def test_window_builds_and_records_history(tmp_path):
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display")
    from aerolap.gui import App

    app = App(root)
    assert app.param_vars and app.current_cfg()["mass"]["total_kg"] == 310.0
    app.param_vars["mass.total_kg"].set("300")
    assert app.param_entries["mass.total_kg"].cget("foreground") == "#0050c8"
    label = next(k for k, v in app.param_labels.items() if v == "mass.total_kg")
    app.sw_param_var.set(label)
    app._sweep_defaults()
    assert float(app.sw_lo_var.get()) == pytest.approx(240.0) and app.task_kwargs()["sweep_path"] == "mass.total_kg"
    res = TASKS["lap"][1](app.current_cfg(), str(TRACK), "L4", True, tmp_path / "lap", log=lambda m: None)
    app._show(res)
    app._add_history(res, {"task": "한 랩", "changes": "mass.total_kg=300 (310)", "setup": "x"})
    app.write_history(tmp_path / "h.csv")
    assert "mass.total_kg=300" in (tmp_path / "h.csv").read_text(encoding="utf-8-sig")
    assert len(app.tree.get_children()) == 1 and app.image_spans
    root.destroy()
