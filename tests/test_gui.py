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
    assert {"aero.components.front.aoa_deg", "aero.components.side.aoa_deg", "aero.components.rear.aoa_deg",
            "aero.components.front.elements.flap1.deflection_deg", "aero.components.front.elements.flap2.gap_mm",
            "aero.components.rear.elements.flap2.deflection_deg", "aero.components.sidepod.exit_area_m2"} <= set(paths)


def test_short_parameter_list_keeps_only_aero_design_knobs():
    cfg = load_vehicle_config(CAR)
    full = [p for _, p in editable_params(cfg)]
    short = [p for _, p in editable_params(cfg, detail=False)]
    assert set(short) < set(full) and len([p for p in short if p.startswith("aero")]) <= 16
    assert {"aero.components.rear.aoa_deg", "aero.components.rear.elements.flap1.deflection_deg",
            "aero.components.sidepod.exit_area_m2", "mass.total_kg"} <= set(short)
    assert not any(k in p for p in short if p.startswith("aero") for k in ("gap_mm", "overlap_mm", "x_mm", "height_mm", "chord_mm"))


def test_import_defaults_match_optimumlap(tmp_path):
    from aerolap.gui_tasks import import_file

    imp = ROOT / "data" / "import"
    car = load_vehicle_config(import_file(imp / "fsk2026_4.0.olveh", tmp_path))
    assert car["powertrain"]["ev"].get("motor_torque_curve")
    full = import_file(imp / "fsk2026_4.0.olveh", tmp_path, full=True)
    assert full.endswith("_full.yaml") and "motor_torque_curve" not in load_vehicle_config(full)["powertrain"]["ev"]
    from aerolap.track import Track

    raw = Track.from_file(import_file(imp / "FSK2026_GiantCorners_ConeMap_V4_851m.oltra", tmp_path))
    smooth = import_file(imp / "FSK2026_GiantCorners_ConeMap_V4_851m.oltra", tmp_path, smooth=15)
    assert raw.length == pytest.approx(851.0, abs=0.01) and smooth.endswith("_smooth15.csv")
    assert Track.from_file(smooth).length < 850


@pytest.mark.parametrize("key", ["lap", "cornering", "sweep", "aero_split"])
def test_gui_tasks_write_text_graphs_and_a_summary(key, tmp_path):
    _, fn = TASKS[key]
    res = fn(load_vehicle_config(CAR), str(TRACK), "L4", True, tmp_path / key, log=lambda m: None, **KW)
    assert res.text and res.summary and res.images and all(Path(p).exists() for p in res.images)


def test_aero_all_runs_every_aero_study_in_one_go(tmp_path):
    res = TASKS["aero_all"][1](load_vehicle_config(CAR), str(TRACK), "L4", True, tmp_path, log=lambda m: None, **KW)
    assert len(res.images) == 4 and all(Path(p).exists() for p in res.images)
    assert res.data.joint is not None  # the "apply optimum" button works on it
    assert (tmp_path / "aero_all.txt").exists() and (tmp_path / "optimized_vehicle.yaml").exists()
    assert "[4]" in res.summary


def test_aero_all_needs_aero_devices(tmp_path):
    with pytest.raises(ValueError, match="aero.components"):
        TASKS["aero_all"][1](load_vehicle_config(ROOT / "data" / "import" / "fsk2026_4.0.olveh", warn=False),
                             str(TRACK), "L0", True, tmp_path, log=lambda m: None, **KW)


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
    cfg = with_overrides(base, {"mass.total_kg": 300.0, "aero.components.front.aoa_deg": 5.0})
    text = changed_params(cfg, base)
    assert "mass.total_kg=300 (310)" in text and "aero.components.front.aoa_deg=5 (2)" in text
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


def test_error_text_and_params():
    from aerolap.config import ConfigError, with_overrides
    from aerolap.gui_tasks import error_params, error_text

    try:
        with_overrides(load_vehicle_config(CAR), {"battery.soc": 0.01}, "내구 SOC 단계 3/3")
    except ConfigError as exc:
        assert error_params(exc) == ["battery.soc", "battery.soc_min"]
        assert "내구 SOC 단계 3/3" in error_text(exc) and "해결:" in error_text(exc)
    assert error_text(KeyError("x")).startswith("차량 설정에 필요한 키가 없음")


def test_window_marks_the_bad_parameter(monkeypatch):
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display")
    from aerolap import gui
    from aerolap.gui import App

    shown = []
    monkeypatch.setattr(gui.messagebox, "showerror", lambda title, text: shown.append((title, text)))
    app = App(root)
    app.param_vars["battery.soc"].set("0.01")
    app.run_task("lap")
    assert shown and "battery.soc" in shown[0][1] and "빨간 칸" in shown[0][1]
    assert app.param_entries["battery.soc"].cget("background") == "#ffd6d6"
    root.destroy()


def test_window_hides_aero_detail_but_keeps_its_values():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display")
    from aerolap.gui import App

    app = App(root)
    gap = "aero.components.rear.elements.flap1.gap_mm"
    assert gap in app.param_vars and gap not in app.param_entries
    assert "aero.components.rear.aoa_deg" in app.param_entries
    app.param_vars[gap].set("12")
    assert get_cfg(app, gap) == 12.0
    app.aero_detail_var.set(True)
    app._layout_params()
    assert app.param_entries[gap].cget("foreground") == "#0050c8"
    root.destroy()


def get_cfg(app, path):
    from aerolap.config import get_path

    return get_path(app.current_cfg(), path)
