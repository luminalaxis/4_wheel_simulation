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


@dataclass
class TaskResult:
    title: str
    text: str
    images: list[Path] = field(default_factory=list)
    out_dir: Path | None = None
    data: object = None  # task specific (e.g. the aero-split result for "apply")


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
    return TaskResult("한 랩", _summary_text(res), [out_dir / "lap.png", out_dir / "track_map.png"], out_dir)


def task_compare(cfg, track_path, level, coarse, out_dir, log=print, **_) -> TaskResult:
    from .analysis.ladder import effect_ladder, ladder_table

    track = Track.from_file(track_path)
    log("효과 분해 L0 → L5 계산 중 ...")
    steps = effect_ladder(cfg, track, start=_start(track), opts=_opts(coarse))
    out_dir.mkdir(parents=True, exist_ok=True)
    plotting.plot_ladder(steps, out_dir / "ladder.png")
    plotting.plot_compare([s.result for s in steps], [s.level for s in steps], out_dir / "compare.png",
                          title=f"{track.name}: OptimumLap-equivalent -> full model")
    return TaskResult("효과 분해", ladder_table(steps), [out_dir / "ladder.png", out_dir / "compare.png"], out_dir)


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
    return TaskResult("종목", events_table(events), imgs, out_dir)


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
                            lambda k, n, x, e: log(f"  윙 크기 {x:g}: 내구 {e.total_time_s:.1f} s"))
        text += "\n\n공력 패키지 전체 크기 스윕:\n" + energy_sweep_table(path, rows)
        plotting.plot_energy_sweep("aero package scale", rows, out_dir / "energy_sweep.png")
        imgs.append(out_dir / "energy_sweep.png")
    return TaskResult("내구 에너지", text, imgs, out_dir)


def task_aero_split(cfg, track_path, level, coarse, out_dir, log=print, objective="endurance_time",
                    distance_km=22.0, bounds=(0.5, 1.6), understeer=(1.05, 1.3), max_evals=100, points=7,
                    **_) -> TaskResult:
    from .analysis.aero_split import aero_split

    spec = {"type": objective, "level": level, "track": str(Path(track_path).resolve())}
    if objective == "endurance_time":
        spec["distance_km"] = distance_km
    constraints = {"understeer_index": list(understeer)} if understeer else {}
    res = aero_split(cfg, spec, ".", constraints, None, bounds, points, True, max_evals, SolverOptions.coarse(), log)
    out_dir.mkdir(parents=True, exist_ok=True)
    plotting.plot_aero_split(res, out_dir / "aero_split.png")
    return TaskResult("공력 장치 최적화", res.report(cfg), [out_dir / "aero_split.png"], out_dir, res)


TASKS = {
    "lap": ("한 랩", task_lap),
    "compare": ("효과 분해 (L0→L5)", task_compare),
    "events": ("FSAE 종목", task_events),
    "energy": ("내구 에너지 / 배터리", task_energy),
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
