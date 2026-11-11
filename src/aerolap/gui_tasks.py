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


# aero-device keys shown in the short parameter list (the design knobs); the rest is "detail"
AERO_BASIC_KEYS = {"wing": ("aoa_deg", "span_m"), "floor": ("scale",), "sidepod": ("exit_area_m2",),
                   "": ("cla", "cda")}
AERO_BASIC_ELEMENT_KEYS = ("deflection_deg",)
AERO_BASIC_GLOBALS = ("cla_scale", "cda_scale")


def editable_params(cfg: dict, detail: bool = True) -> list[tuple[str, str]]:
    """Parameters present in ``cfg`` (aero components added when the car has them).

    ``detail=False`` keeps only the aero design knobs (wing angles / span, floor size, sidepod exit,
    ClA·CdA of constant devices, overall scales); position, height, chord, gap, overlap, endplate and
    cooling details are left out.
    """
    out = [(label, p) for label, p in COMMON_PARAMS if isinstance(get_path(cfg, p, None), (int, float))]
    out += [("공력 차체 CdA [m²]", "aero.cda")] if isinstance(get_path(cfg, "aero.cda", None), (int, float)) else []
    labels = {"scale": "크기 scale", "cla": "ClA [m²]", "cda": "CdA [m²]", "cop_front": "앞축 분담",
              "aoa_deg": "메인 받음각 [deg]", "span_m": "스팬 [m]", "chord_m": "코드 [m]", "elements": "엘리먼트 수",
              "height_mm": "지상고 [mm]", "x_mm": "위치 x [mm]", "endplate_factor": "엔드플레이트 계수",
              "area_m2": "면적 [m²]", "cl_ref": "기준 CL", "stall_mm": "막힘 높이 [mm]", "rake_gain": "레이크 이득",
              "cd_area": "CdA [m²]", "core_area_m2": "라디에이터 코어 면적 [m²]", "exit_area_m2": "출구 면적 [m²]",
              "loss_coefficient": "코어 손실계수 K", "body_cda": "외형 CdA [m²]", "coolant_temp_C": "냉각수 온도 [°C]",
              "effectiveness": "라디에이터 유효도"}
    el_labels = {"chord_mm": "코드 [mm]", "deflection_deg": "각도 [deg, 메인 코드선 기준]", "gap_mm": "갭 [mm]",
                 "overlap_mm": "오버랩 [mm]"}
    for name, comp in ((cfg.get("aero") or {}).get("components") or {}).items():
        kind = str(comp.get("type", "")).lower()
        elements = comp.get("elements") if isinstance(comp.get("elements"), dict) else None
        keys = {"wing": ("aoa_deg", "span_m", "chord_m", "elements", "height_mm", "x_mm", "endplate_factor"),
                "floor": ("scale", "area_m2", "cl_ref", "height_mm", "x_mm", "stall_mm", "rake_gain", "cd_area"),
                "sidepod": ("exit_area_m2", "core_area_m2", "loss_coefficient", "body_cda", "cla", "coolant_temp_C",
                            "effectiveness", "x_mm")}.get(kind, ("scale", "cla", "cda", "cop_front"))
        basic = AERO_BASIC_KEYS.get(kind, AERO_BASIC_KEYS[""])
        for key in keys:
            if kind and key not in comp and key not in ("aoa_deg", "scale", "exit_area_m2"):
                continue
            if key == "elements" and elements is not None:
                continue
            if not detail and key not in basic:
                continue
            out.append((f"공력 {name} {labels[key]}", f"aero.components.{name}.{key}"))
        for el, ev in (elements or {}).items():
            for key in ("deflection_deg", "chord_mm", "gap_mm", "overlap_mm"):
                if not detail and key not in AERO_BASIC_ELEMENT_KEYS:
                    continue
                if isinstance(ev, dict) and isinstance(ev.get(key), (int, float)):
                    out.append((f"공력 {name}.{el} {el_labels[key]}", f"aero.components.{name}.elements.{el}.{key}"))
    for key, label in (("cla_scale", "전체 ClA 배율"), ("cda_scale", "전체 CdA 배율"), ("cop_shift", "CoP 이동")):
        if detail or key in AERO_BASIC_GLOBALS:
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
    from .analysis.cooling import cooling_balance

    cb = cooling_balance(car, res)
    return TaskResult("한 랩", _summary_text(res) + ("\n" + cb.report() if cb else ""), [out_dir / "lap.png", out_dir / "track_map.png"], out_dir,
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
                    distance_km=22.0, bounds=None, understeer=(1.0, 1.3), max_evals=100, points=7,
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


def task_exchange(cfg, track_path, level, coarse, out_dir, log=print, objective="endurance_time", distance_km=22.0,
                  **_) -> TaskResult:
    from .analysis.aero_exchange import aero_exchange

    obj = _objective(objective, track_path, level, distance_km, coarse)
    ex = aero_exchange(cfg, obj, log)
    out_dir.mkdir(parents=True, exist_ok=True)
    plotting.plot_aero_exchange(ex, out_dir / "aero_exchange.png")
    cla, cda = ex.get("ClA"), ex.get("CdA")
    rate = f"ClA +0.1 = CdA +{0.1 * -cla.delta / cda.delta:.3f}" if cla and cda and cda.delta > 0 > cla.delta else ""
    return TaskResult("공력 교환비", ex.report(), [out_dir / "aero_exchange.png"], out_dir,
                      summary=f"ClA +0.1: {cla.delta:+.3f} {ex.unit} | CdA +0.1: {cda.delta:+.3f} {ex.unit} | {rate}")


def task_envelope(cfg, track_path, level, coarse, out_dir, log=print, **_) -> TaskResult:
    from .analysis.aero_envelope import aero_envelope

    track = Track.from_file(track_path)
    car = VehicleModel(cfg, features_for_level(level))
    log("공력 작동 영역 계산 중 ...")
    res = QSSSolver(_opts(coarse)).solve(car, track, start=_start(track))
    env = aero_envelope(res, car)
    out_dir.mkdir(parents=True, exist_ok=True)
    plotting.plot_aero_envelope(res, env, car, out_dir / "aero_envelope.png")
    b = env.balance
    flagged = ", ".join(f"{k} {v * 100:.1f} %" for k, v in env.shares.items() if v > 0) or "경고 없음"
    return TaskResult("공력 작동 영역", env.report(), [out_dir / "aero_envelope.png"], out_dir,
                      summary=f"밸런스 제동 {b['braking']:.3f} / 코너 {b['cornering']:.3f} / 가속 {b['accelerating']:.3f}"
                              f" | 앞 차고 {env.ranges['front ride height [mm]'][0]:.0f}-"
                              f"{env.ranges['front ride height [mm]'][2]:.0f} mm | {flagged}")


def task_aero_all(cfg, track_path, level, coarse, out_dir, log=print, objective="endurance_time",
                  distance_km=22.0, bounds=None, understeer=(1.0, 1.3), max_evals=100, points=7,
                  **_) -> TaskResult:
    """All aero studies in one run: operating envelope -> exchange rates -> device optimisation of every
    device -> envelope of the optimised car. One report (``aero_all.txt``) with every graph; the
    optimum can be applied with the usual "apply" button."""
    if not ((cfg.get("aero") or {}).get("components") or {}):
        raise ValueError("이 차량에는 공력 장치(aero.components)가 없습니다 - 상수 Cl·Cd 차량(OptimumLap 동등)은 "
                         "장치별 계산을 할 수 없습니다. fsk2026.yaml처럼 장치 모델이 있는 차량을 고르세요")
    from .config import dump_config

    out_dir = Path(out_dir)
    kw = dict(objective=objective, distance_km=distance_km)
    steps = []

    def step(k, title, fn, sub, **extra):
        log(f"=== [{k}/4] {title} ===")
        res = fn(cfg if k < 4 else extra.pop("car"), track_path, level, coarse, out_dir / sub, log=log, **extra)
        steps.append(res)
        return res

    step(1, "공력 작동 영역 (현재 차)", task_envelope, "1_envelope")
    step(2, "공력 교환비", task_exchange, "2_exchange", **kw)
    split = step(3, "공력 장치 최적화 (모든 장치 동시)", task_aero_split, "3_aero_split", bounds=bounds,
                 understeer=understeer, max_evals=max_evals, points=points, **kw)
    best_cfg = getattr(getattr(split.data, "joint", None), "best_cfg", None)
    if best_cfg is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        dump_config(best_cfg, out_dir / "optimized_vehicle.yaml")
        step(4, "공력 작동 영역 (최적화된 차)", task_envelope, "4_envelope_optimized", car=best_cfg)
    else:
        log("=== [4/4] 최적값이 없어 최적화된 차의 작동 영역은 건너뜀 ===")
    titles = ["현재 차 공력 작동 영역", "공력 교환비", "공력 장치 최적화", "최적화된 차 공력 작동 영역"]
    lines = ["공력 전체 계산 요약", ""] + [f"[{i + 1}] {titles[i]}: {r.summary}" for i, r in enumerate(steps)]
    if best_cfg is not None:
        lines.append(f"\n최적화된 차량: {out_dir / 'optimized_vehicle.yaml'}")
    text = "\n".join(lines) + "\n\n" + "\n\n".join(
        f"{'=' * 70}\n[{i + 1}] {titles[i]}\n{'=' * 70}\n{r.text}" for i, r in enumerate(steps))
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "aero_all.txt").write_text(text, encoding="utf-8")
    images = [p for r in steps for p in r.images]
    return TaskResult("공력 전체 계산", text, images, out_dir, split.data,
                      summary=" | ".join(f"[{i + 1}] {r.summary.split(' | ')[0]}" for i, r in enumerate(steps)))


def task_rank_sections(cfg, track_path, level, coarse, out_dir, log=print, section_csv=None, section_component="front",
                       objective="lap_time", distance_km=22.0, understeer=(1.0, 1.3), max_geometries=None,
                       **_) -> TaskResult:
    from .analysis.section_rank import rank_sections
    from .config import dump_config

    if not section_csv:
        raise ValueError("AERO_AUTOMATION summary.csv 파일을 고르세요")
    spec = {"type": objective, "level": level, "track": str(Path(track_path).resolve())}
    if objective == "endurance_time":
        spec["distance_km"] = distance_km
    rk = rank_sections(cfg, section_csv, section_component, spec,
                       {"understeer_index": list(understeer)} if understeer else {}, cfg.get("_base_dir", "."),
                       SolverOptions.coarse(), max_geometries, progress=log)
    out_dir.mkdir(parents=True, exist_ok=True)
    plotting.plot_section_rank(rk, out_dir / "section_rank.png")
    if rk.best_cfg is not None:
        dump_config(rk.best_cfg, out_dir / "best_vehicle.yaml")
    best = rk.ordered()[0]
    return TaskResult("CFD 단면 순위", rk.report(), [out_dir / "section_rank.png"], out_dir, rk,
                      summary=f"{section_component} 1위 {best.geometry_id} @ {best.aoa:.1f}° → {best.value:.3f} {rk.unit} "
                              f"({best.value - rk.current_search.value:+.3f})"
                              + (" [합성 데이터]" if rk.synthetic else ""))


def error_params(exc: Exception) -> list[str]:
    """Parameter paths an error is about (``ConfigError.paths``), for marking them in the GUI."""
    return list(getattr(exc, "paths", None) or ([exc.path] if getattr(exc, "path", None) else []))


def error_text(exc: Exception) -> str:
    """Message for the user: the structured ConfigError text, otherwise the type and the message."""
    from .config import ConfigError

    if isinstance(exc, ConfigError):
        return str(exc)
    if isinstance(exc, FileNotFoundError):
        return f"파일을 찾을 수 없음: {exc.filename or exc}"
    if isinstance(exc, KeyError):
        return f"차량 설정에 필요한 키가 없음: {exc}"
    return f"{type(exc).__name__}: {exc}"


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
    if source and Path(source).exists() and Path(source).suffix.lower() in (".yaml", ".yml"):
        for line in Path(source).read_text(encoding="utf-8").splitlines():
            if not line.startswith("#"):
                break
            head.append(line)
        head.append(f"# (saved from the aerolap GUI; based on {Path(source).name} - inline comments are not kept)")
    from .config import relocate_paths

    clean = relocate_paths(cfg, Path(path).resolve().parent)
    text = "\n".join(head) + ("\n" if head else "") + yaml.safe_dump(clean, sort_keys=False, allow_unicode=True)
    Path(path).write_text(text, encoding="utf-8")


TASKS = {
    "lap": ("한 랩", task_lap),
    "compare": ("효과 분해 (L0→L5)", task_compare),
    "events": ("FSAE 종목", task_events),
    "energy": ("내구 에너지 / 배터리", task_energy),
    "cornering": ("코너링 한계 · 밸런스", task_cornering),
    "envelope": ("공력 작동 영역 · 맵 점검", task_envelope),
    "exchange": ("공력 교환비 (ClA·CdA·밸런스·질량)", task_exchange),
    "sweep": ("파라미터 스윕", task_sweep),
    "aero_split": ("공력 장치 최적화 (front/side/rear)", task_aero_split),
    "aero_all": ("공력 전체 계산 (작동 영역·교환비·최적화)", task_aero_all),
    "rank_sections": ("CFD 단면 순위 (AERO_AUTOMATION)", task_rank_sections),
}


def import_file(path: str | Path, out_dir: Path, full: bool = False, smooth: float | None = None) -> str:
    """Convert an OptimumLap .oltra / .olveh or an Assetto Corsa .ai file; returns the written path.

    The defaults reproduce the OptimumLap inputs as they are (the file's torque curve, the segment
    track unchanged), so an L0 run is comparable with OptimumLap. ``full=True``: EMRAX 228 motor model
    and battery instead of the torque curve. ``smooth`` [m]: .oltra re-drawn as a smoothed x-y line
    (removes cone-map curvature noise, but changes length and radii). The variant is added to the
    file name (``_full``, ``_smooth15``) so different imports do not overwrite each other.
    """
    from .cli import main

    path = Path(path)
    suf = path.suffix.lower()
    out_dir.mkdir(parents=True, exist_ok=True)
    if suf == ".olveh":
        out = out_dir / f"{path.stem}{'_full' if full else ''}.yaml"
        args = ["import-vehicle", str(path), "--out", str(out)] + (["--full"] if full else [])
    elif suf in (".oltra", ".ai"):
        tag = f"_smooth{smooth:g}" if suf == ".oltra" and smooth else ""
        out = out_dir / f"{path.stem}{tag}.csv"
        args = ["import-track", str(path), "--out", str(out)]
        if suf == ".oltra" and smooth:
            args += ["--smooth", str(smooth)]
    else:
        raise ValueError("지원하는 파일: OptimumLap .olveh / .oltra, Assetto Corsa .ai")
    if main(args) != 0:
        raise ValueError(f"변환 실패: {path}")
    return str(out)
