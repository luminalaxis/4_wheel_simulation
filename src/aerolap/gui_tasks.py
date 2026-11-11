"""GUI work items, independent of tkinter (so they can be tested without a display).

Every task takes the vehicle config dict, a track path, the model level and options, writes its
files to ``out_dir`` and returns a :class:`TaskResult` (text + PNG paths) for the window.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from . import plotting
from .config import get_path
from .features import features_for_level
from .solvers.ggv import SolverOptions
from .solvers.qss import QSSSolver
from .track import Track
from .vehicle import VehicleModel

ROOT = Path(__file__).resolve().parents[2]

# (label, dotted path) of the values shown in the parameter tab
COMMON_PARAMS = [
    ("질량 [kg]", "mass.total_kg"),
    ("앞 무게 배분 [-]", "mass.weight_dist_front"),
    ("무게중심 높이 [mm]", "mass.cg_height_mm"),
    ("휠베이스 [mm]", "geometry.wheelbase_mm"),
    ("앞 트레드 [mm]", "geometry.track_front_mm"),
    ("뒤 트레드 [mm]", "geometry.track_rear_mm"),
    ("타이어 μx", "tires.mu_x"),
    ("타이어 μy", "tires.mu_y"),
    ("타이어 반경 [mm]", "tires.radius_mm"),
    ("앞 스프링 [N/mm]", "suspension.front.spring_rate_N_per_mm"),
    ("뒤 스프링 [N/mm]", "suspension.rear.spring_rate_N_per_mm"),
    ("앞 ARB [Nm/deg]", "suspension.front.arb_rate_Nm_per_deg"),
    ("뒤 ARB [Nm/deg]", "suspension.rear.arb_rate_Nm_per_deg"),
    ("앞 정적 차고 [mm]", "suspension.front.static_ride_height_mm"),
    ("뒤 정적 차고 [mm]", "suspension.rear.static_ride_height_mm"),
    ("감속비", "powertrain.ev.reduction_ratio"),
    ("축전지 전압 [V]", "powertrain.ev.dc_voltage_V"),
    ("출력 한계 [kW]", "powertrain.ev.power_limit_kW"),
    ("배터리 용량 [kWh]", "battery.capacity_kWh"),
    ("사용 가능 비율", "battery.usable_fraction"),
    ("회생 한계 [kW]", "battery.regen_power_kW"),
    ("팩 직렬 수", "battery.series"),
    ("팩 병렬 수", "battery.parallel"),
    ("시작 SOC", "battery.soc"),
    ("최저 SOC", "battery.soc_min"),
    ("배선 저항 [mΩ]", "battery.interconnect_resistance_mOhm"),
    ("팩 최대 전류 [A]", "battery.max_current_A"),
    ("셀 용량 [Ah]", "battery.cell.capacity_Ah"),
    ("셀 내부저항 [mΩ]", "battery.cell.resistance_mOhm"),
    ("셀 최대 전류 [A]", "battery.cell.max_current_A"),
    ("셀 최저 전압 [V]", "battery.cell.v_min"),
]


def editable_params(cfg: dict) -> list[tuple[str, str]]:
    """Parameters present in ``cfg`` (aero components added when the car has them)."""
    out = [(label, p) for label, p in COMMON_PARAMS if isinstance(get_path(cfg, p, None), (int, float))]
    for name in ((cfg.get("aero") or {}).get("components") or {}):
        for key, label in (("scale", "크기 scale"), ("cla", "ClA [m²]"), ("cda", "CdA [m²]"),
                           ("cop_front", "앞축 분담")):
            out.append((f"공력 {name} {label}", f"aero.components.{name}.{key}"))
    for key, label in (("cla_scale", "전체 ClA 배율"), ("cda_scale", "전체 CdA 배율"), ("cop_shift", "CoP 이동")):
        out.append((f"공력 {label}", f"aero.{key}"))
    return out


class TaskCancelled(Exception):
    """Raised from the log callback when the user pressed cancel."""


@dataclass
class TaskResult:
    title: str
    text: str
    images: list[Path] = field(default_factory=list)
    out_dir: Path | None = None
    data: object = None  # task specific (e.g. the aero-split result for "apply")
    summary: str = ""  # one line of key numbers for the history table


def _opts(coarse: bool) -> SolverOptions:
    return SolverOptions.coarse() if coarse else SolverOptions()


def _start(track: Track) -> str:
    return "flying" if track.closed else "standing"


def task_lap(cfg, track_path, level, coarse, out_dir, log=print, **_) -> TaskResult:
    from .cli import _summary_text

    track = Track.from_file(track_path)
    car = VehicleModel(cfg, features_for_level(level))
    log(f"한 랩 계산: {track.name}")
    res = QSSSolver(_opts(coarse)).solve(car, track, start=_start(track))
    out_dir.mkdir(parents=True, exist_ok=True)
    res.to_csv(out_dir / "trace.csv")
    plotting.plot_lap(res, out_dir / "lap.png")
    plotting.plot_track_map(res, out_dir / "track_map.png")
    return TaskResult("한 랩", _summary_text(res), [out_dir / "lap.png", out_dir / "track_map.png"], out_dir,
                      summary=f"랩 {res.lap_time:.3f} s | 최고 {res.v.max() * 3.6:.1f} km/h | "
                              f"에너지 {res.energy_J / 3.6e6:.4f} kWh")


def task_compare(cfg, track_path, level, coarse, out_dir, log=print, **_) -> TaskResult:
    from .analysis.ladder import effect_ladder, ladder_table

    track = Track.from_file(track_path)
    log("효과 분해 L0 → L5 계산 중 ...")
    steps = effect_ladder(cfg, track, start=_start(track), opts=_opts(coarse))
    out_dir.mkdir(parents=True, exist_ok=True)
    plotting.plot_ladder(steps, out_dir / "ladder.png")
    plotting.plot_compare([s.result for s in steps], [s.level for s in steps], out_dir / "compare.png",
                          title=f"{track.name}: OptimumLap-equivalent -> full model")
    return TaskResult("효과 분해", ladder_table(steps), [out_dir / "ladder.png", out_dir / "compare.png"], out_dir,
                      summary=" | ".join(f"{st.level} {st.result.lap_time:.3f}" for st in steps) + " s")


def task_events(cfg, track_path, level, coarse, out_dir, log=print, distance_km=22.0, **_) -> TaskResult:
    from .analysis.events import events_table, run_events

    track = Track.from_file(track_path)
    car = VehicleModel(cfg, features_for_level(level))
    log("종목 계산 중 (가속, 스키드패드, 오토크로스, 내구) ...")
    endu = Track.from_file(track_path, closed=True)
    events = run_events(car, track, endu, distance_km, _opts(coarse))
    out_dir.mkdir(parents=True, exist_ok=True)
    imgs = []
    for e in events:
        e.result.to_csv(out_dir / f"trace_{e.name}.csv")
    plotting.plot_lap(events[-1].result, out_dir / "endurance_lap.png")
    imgs.append(out_dir / "endurance_lap.png")
    return TaskResult("종목", events_table(events), imgs, out_dir,
                      summary=" | ".join(f"{e.name} {e.time_s:.3f} s" for e in events)
                              + f" | 내구 {events[-1].energy_kWh:.2f} kWh")


def task_energy(cfg, track_path, level, coarse, out_dir, log=print, distance_km=22.0, wing_sweep=True,
                **_) -> TaskResult:
    from .analysis.energy import endurance_energy, energy_sweep, energy_sweep_table

    track = Track.from_file(track_path, closed=True)
    feats = features_for_level(level)
    log("내구 에너지 계산 중 ...")
    en = endurance_energy(cfg, track, distance_km, feats, _opts(coarse))
    text = en.report()
    imgs = []
    out_dir.mkdir(parents=True, exist_ok=True)
    if wing_sweep:
        comps = (cfg.get("aero") or {}).get("components")
        path = ",".join(f"aero.components.{n}.scale" for n in comps) if comps else "aero.cla_scale,aero.cda_scale"
        rows = energy_sweep(cfg, track, path, [0.6, 0.8, 1.0, 1.2, 1.4], distance_km, feats, _opts(coarse),
                            lambda k, n, x, e: log(f"  공력 패키지 x{x:g}: 내구 {e.total_time_s:.1f} s"), relative=True)
        text += "\n\n공력 패키지 전체 크기 스윕 (현재 값 x 배율):\n" + energy_sweep_table("x current", rows)
        plotting.plot_energy_sweep("aero package size (x current)", rows, out_dir / "energy_sweep.png")
        imgs.append(out_dir / "energy_sweep.png")
    cap = "출력 제한 없음" if en.power_cap_kW is None else f"출력 캡 {en.power_cap_kW:.1f} kW"
    usable = "" if en.usable_kWh is None else f" / 사용 가능 {en.usable_kWh:.2f}"
    return TaskResult("내구 에너지", text, imgs, out_dir,
                      summary=f"내구 {en.total_time_s:.1f} s | {cap} | 필요 {en.unlimited_required_kWh:.2f}{usable} kWh"
                              + ("" if en.feasible else " | 용량 부족"))


def task_aero_split(cfg, track_path, level, coarse, out_dir, log=print, objective="endurance_time",
                    distance_km=22.0, bounds=(0.5, 1.6), understeer=(1.0, 1.3), max_evals=100, points=7,
                    **_) -> TaskResult:
    from .analysis.aero_split import aero_split

    spec = {"type": objective, "level": level, "track": str(Path(track_path).resolve())}
    if objective == "endurance_time":
        spec["distance_km"] = distance_km
    constraints = {"understeer_index": list(understeer)} if understeer else {}
    res = aero_split(cfg, spec, ".", constraints, None, bounds, points, True, max_evals, SolverOptions.coarse(), log)
    out_dir.mkdir(parents=True, exist_ok=True)
    plotting.plot_aero_split(res, out_dir / "aero_split.png")
    j = res.joint
    best = ", ".join(f"{p.split('.')[-2]} {v:.2f}" for p, v in j.best.items())
    return TaskResult("공력 장치 최적화", res.report(cfg), [out_dir / "aero_split.png"], out_dir, res,
                      summary=f"{j.initial_eval.value:.3f} → {j.best_eval.value:.3f} {res.unit} | {best}"
                              + ("" if j.best_eval.feasible else " | 제약 위반 (유효한 최적값 아님)"))


def _objective(kind, track_path, level, distance_km, coarse, understeer=None):
    from .analysis.objectives import Objective

    spec = {"type": kind, "level": level}
    if kind != "skidpad_time":
        spec["track"] = str(Path(track_path).resolve())
    if kind == "endurance_time":
        spec["distance_km"] = distance_km
    constraints = {"understeer_index": list(understeer)} if understeer else {}
    return Objective(spec, ".", constraints, _opts(coarse))


def task_sweep(cfg, track_path, level, coarse, out_dir, log=print, sweep_path=None, sweep_lo=0.0, sweep_hi=1.0,
               sweep_n=7, sweep_objective="lap_time", distance_km=22.0, understeer=None, **_) -> TaskResult:
    import numpy as np

    from .analysis.sweep import sweep

    if not sweep_path:
        raise ValueError("스윕할 파라미터를 고르세요")
    if sweep_n < 2 or sweep_hi <= sweep_lo:
        raise ValueError("스윕 범위: 끝 값이 시작 값보다 커야 하고 점은 2개 이상")
    obj = _objective(sweep_objective, track_path, level, distance_km, coarse)  # no setup constraints here
    values = np.linspace(sweep_lo, sweep_hi, int(sweep_n))
    sw = sweep(cfg, [sweep_path], [values], obj,
               lambda k, n, over, ev: log(f"  [{k}/{n}] {sweep_path} = {over[sweep_path]:g} -> {ev.value:.3f} {obj.unit}"))
    out_dir.mkdir(parents=True, exist_ok=True)
    plotting.plot_sweep(sw, out_dir / "sweep.png", obj.unit)
    y, ok = sw.objective(), sw.feasible()
    i = int(np.nanargmin(np.where(ok, y, np.nan))) if ok.any() else int(np.nanargmin(y))
    cur = get_path(cfg, sweep_path, None)
    text = (f"objective: {obj.label} [{obj.unit}]\nparameter: {sweep_path} (now {cur})\n\n" + sw.table()
            + f"\n\nbest: {sweep_path} = {values[i]:g} -> {y[i]:.3f} {obj.unit}")
    return TaskResult("파라미터 스윕", text, [out_dir / "sweep.png"], out_dir,
                      summary=f"{sweep_path}: 최적 {values[i]:g} → {y[i]:.3f} {obj.unit} "
                              f"(범위 {y.min():.3f} .. {y.max():.3f})")


def task_cornering(cfg, track_path, level, coarse, out_dir, log=print, **_) -> TaskResult:
    import numpy as np

    from .analysis.cornering import (BRAKING_COLUMNS, CORNERING_COLUMNS, braking_sweep, cornering_sweep,
                                     format_table)

    car = VehicleModel(cfg, features_for_level(level))
    speeds = np.arange(30.0, 121.0, 10.0) / 3.6
    log("코너링·제동 한계 계산 중 ...")
    corner = cornering_sweep(car, speeds, None, _opts(coarse))
    brake = braking_sweep(car, speeds, None, _opts(coarse))
    ff, fr = car.ride_frequencies()
    text = (f"static weight front: {car.wd_front * 100:.1f} %   ride frequency front/rear: {ff:.2f} / {fr:.2f} Hz\n"
            "understeer index = front / rear lateral usage at the limit (> 1: understeer, < 1: oversteer)\n\n"
            "steady-state cornering limit (ax = 0):\n" + format_table(corner, CORNERING_COLUMNS)
            + "\n\nstraight-line braking limit:\n" + format_table(brake, BRAKING_COLUMNS))
    out_dir.mkdir(parents=True, exist_ok=True)
    plotting.plot_cornering(corner, out_dir / "cornering.png", title=f"{car.name} - steady-state cornering")
    usi = corner["understeer_index"]
    usi = usi[np.isfinite(usi)]
    k = int(np.argmin(np.abs(corner["v_kmh"] - 60.0)))
    us = f" | 언더스티어 지수 {usi.min():.3f} .. {usi.max():.3f}" if usi.size else ""
    return TaskResult("코너링 한계", text, [out_dir / "cornering.png"], out_dir,
                      summary=f"60 km/h 횡가속 {corner['ay_max_g'][k]:.2f} g{us} | "
                              f"최소 바닥 간극 {min(corner['clearance_mm'].min(), brake['clearance_mm'].min()):.1f} mm")


def changed_params(cfg: dict, base: dict) -> str:
    """Edited values vs the loaded file, e.g. ``mass.total_kg=300 (310)``."""
    out = []
    for _, path in editable_params(base):
        a, b = get_path(cfg, path, None), get_path(base, path, None)
        if a != b and not (isinstance(a, (int, float)) and isinstance(b, (int, float)) and abs(a - b) < 1e-12):
            out.append(f"{path}={a:g} ({b:g})" if isinstance(a, (int, float)) and isinstance(b, (int, float))
                       else f"{path}={a} ({b})")
    return ", ".join(out)


def save_vehicle_yaml(cfg: dict, path: str | Path, source: str | Path | None = None) -> None:
    """Write the vehicle YAML, keeping the leading comment block of the source file (ASSUMED notes ...)."""
    import yaml

    head = []
    if source and Path(source).exists():
        for line in Path(source).read_text(encoding="utf-8").splitlines():
            if not line.startswith("#"):
                break
            head.append(line)
        head.append(f"# (saved from the aerolap GUI; based on {Path(source).name} - inline comments are not kept)")
    clean = {k: v for k, v in cfg.items() if not k.startswith("_")}
    text = "\n".join(head) + ("\n" if head else "") + yaml.safe_dump(clean, sort_keys=False, allow_unicode=True)
    Path(path).write_text(text, encoding="utf-8")


TASKS = {
    "lap": ("한 랩", task_lap),
    "compare": ("효과 분해 (L0→L5)", task_compare),
    "events": ("FSAE 종목", task_events),
    "energy": ("내구 에너지 / 배터리", task_energy),
    "cornering": ("코너링 한계 · 밸런스", task_cornering),
    "sweep": ("파라미터 스윕", task_sweep),
    "aero_split": ("공력 장치 최적화 (front/side/rear)", task_aero_split),
}


def import_file(path: str | Path, out_dir: Path, full: bool = True, smooth: float | None = 15.0) -> str:
    """Convert an OptimumLap .oltra / .olveh or an Assetto Corsa .ai file; returns the written path."""
    from .cli import main

    path = Path(path)
    suf = path.suffix.lower()
    out_dir.mkdir(parents=True, exist_ok=True)
    if suf == ".olveh":
        out = out_dir / f"{path.stem}.yaml"
        args = ["import-vehicle", str(path), "--out", str(out)] + (["--full"] if full else [])
    elif suf in (".oltra", ".ai"):
        out = out_dir / f"{path.stem}.csv"
        args = ["import-track", str(path), "--out", str(out)]
        if suf == ".oltra" and smooth:
            args += ["--smooth", str(smooth)]
    else:
        raise ValueError("지원하는 파일: OptimumLap .olveh / .oltra, Assetto Corsa .ai")
    if main(args) != 0:
        raise ValueError(f"변환 실패: {path}")
    return str(out)
