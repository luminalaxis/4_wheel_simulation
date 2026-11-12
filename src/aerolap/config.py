"""Vehicle configuration: YAML loading, defaults, validation and dotted-path access.

The raw configuration is kept as a nested ``dict`` in *engineering units* (mm, N/mm,
deg, kg, kW). That dict is the single source of truth: sweeps and the optimizer modify
it through dotted paths such as ``"suspension.front.spring_rate_N_per_mm"`` and a new
:class:`~aerolap.vehicle.VehicleModel` is built from it (unit conversion to SI happens
there).
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    """Missing or invalid input, reported with the parameter, its value and the violated condition.

    ``str(err)`` is a multi-line message::

        파라미터 battery.soc = 0.031
        조건: battery.soc_min (0.05) < battery.soc <= 1 위반
        원인: 내부 계산 - 내구 SOC 단계 3/3 (적용한 값: battery.soc=0.031)
        해결: ...

    ``paths`` lists the parameters involved (the first one is the main one) so a GUI can mark them.
    """

    def __init__(self, message: str = "", *, path: str | None = None, value: Any = None, condition: str = "",
                 hint: str = "", context: str = "", paths: list[str] | None = None, has_value: bool = True):
        self.path = path
        self.value = value
        self.condition = condition
        self.hint = hint
        self.context = context
        self.message = message
        self.has_value = has_value and path is not None
        self.paths = list(paths or ([path] if path else []))
        super().__init__(self.text())

    def text(self) -> str:
        lines = []
        if self.path:
            lines.append(f"파라미터 {self.path}" + (f" = {self.value!r}" if self.has_value else ""))
        if self.condition:
            lines.append(f"조건: {self.condition}")
        if self.message:
            lines.append(self.message)
        if self.context:
            lines.append(f"원인: {self.context}")
        if self.hint:
            lines.append(f"해결: {self.hint}")
        return "\n".join(lines) or "invalid configuration"

    def with_context(self, context: str) -> "ConfigError":
        self.context = f"{self.context}; {context}" if self.context else context
        self.args = (self.text(),)
        return self


def _fmt(v: Any) -> str:
    return f"{v:g}" if isinstance(v, float) else repr(v)


AXLE_DEFAULTS: dict[str, Any] = {
    "motion_ratio": 1.0,  # spring travel / wheel travel -> wheel rate = k_spring * MR^2
    "arb_rate_Nm_per_deg": 0.0,  # anti-roll bar roll stiffness referred to the wheels
    "heave_spring_N_per_mm": 0.0,  # third (heave) element, referred to the wheel
    "roll_center_mm": 30.0,
    "static_camber_deg": 0.0,  # negative = top of the wheel leans towards the chassis
    "camber_gain_deg_per_mm": 0.0,  # camber change per mm of bump (negative = gains negative camber)
    "anti_dive": 0.0,  # front: fraction of braking pitch transfer carried by the links
    "anti_lift": 0.0,  # rear: fraction of braking pitch transfer carried by the links
    "anti_squat": 0.0,  # rear: fraction of driving pitch transfer carried by the links
    # transient solver only
    "damper_rate_Ns_per_mm": None,  # damper rate at the damper [N/(mm/s)]; null -> from damping_ratio
    "damping_ratio": 0.65,  # corner damping ratio used when damper_rate_Ns_per_mm is null
}

STRATEGY_DEFAULTS: dict[str, Any] = {
    "type": "fixed",  # fixed | rule | ideal
    "open_mode": None,  # low-drag mode (default: first mode that is not default_mode)
    "closed_mode": None,  # high-downforce mode (default: default_mode)
    "open_radius_m": 50.0,  # flap may open where the corner radius is larger than this
    "min_length_m": 10.0,  # ... over at least this distance
    "min_speed_kmh": 0.0,  # ... and only above this speed (low speed: rear grip is worth more)
    "open_delay_s": 0.2,  # actuation time to open
    "close_delay_s": 0.15,  # actuation time to close
    "close_anticipation": False,  # True: controller closes the flap before the brake point
    "max_iterations": 6,
}

DEFAULTS: dict[str, Any] = {
    "name": "unnamed vehicle",
    "environment": {
        "temperature_C": 20.0,
        "pressure_kPa": None,  # None -> from altitude (ISA)
        "altitude_m": 0.0,
        "relative_humidity": 0.5,
        "air_density_kg_m3": None,  # set -> used directly instead of the conditions above
    },
    "mass": {"unsprung_front_kg": 0.0, "unsprung_rear_kg": 0.0,
             "pitch_inertia_kgm2": None,  # sprung mass, transient solver only (null -> estimate)
             "roll_inertia_kgm2": None,
             "yaw_inertia_kgm2": None},  # whole car, transient lateral dynamics (null -> m (0.4 L)^2)
    "tires": {
        "ref_load_N": 700.0,
        "load_sensitivity": 0.0,  # relative change of mu per +100 % load
        "min_mu_factor": 0.4,
        "vertical_rate_N_per_mm": 100.0,
        "peak_slip_deg": 6.0,
        "camber_optimum_deg": -1.0,  # best camber of the loaded (outer) wheel for lateral grip
        "camber_sensitivity_per_deg2": 0.0,
        "rolling_resistance": 0.015,
    },
    "suspension": {},
    "brakes": {"bias_front": None},  # None -> ideal brake balance
    "battery": {
        "capacity_kWh": None,  # nominal pack energy; null -> no energy check
        "usable_fraction": 0.95,  # usable share of the nominal energy (SOC window)
        "max_power_kW": None,  # pack discharge capability; null -> only the powertrain limit
        "regen_power_kW": 0.0,  # recuperation power limit at the battery (0 = no regen)
        "regen_axle_share": 0.35,  # share of the braking force on the driven (rear) axle
    },
    "aero": {
        "type": "constant",
        "cla_scale": 1.0,
        "cda_scale": 1.0,
        "cop_shift": 0.0,  # added to the front downforce fraction
        "drag_height_mm": 400.0,
        "floor_width_mm": 800.0,
        "bottoming_clearance_mm": 5.0,
        "cop_lateral_shift_mm_per_deg": 0.0,
        "strategy": {},
    },
    "powertrain": {
        "drive": "rwd",  # rwd | awd
        "differential": "lsd",  # lsd | open
        "efficiency": 0.9,
        "awd_front_fraction": None,  # None -> ideal torque split for awd
    },
}

REQUIRED: tuple[str, ...] = (
    "mass.total_kg",
    "mass.weight_dist_front",
    "mass.cg_height_mm",
    "geometry.wheelbase_mm",
    "geometry.track_front_mm",
    "geometry.track_rear_mm",
    "tires.radius_mm",
    "tires.mu_x",
    "tires.mu_y",
    "suspension.front.spring_rate_N_per_mm",
    "suspension.front.static_ride_height_mm",
    "suspension.rear.spring_rate_N_per_mm",
    "suspension.rear.static_ride_height_mm",
    "powertrain.type",
)


_AXLE_KEYS = set(AXLE_DEFAULTS) | {"spring_rate_N_per_mm", "static_ride_height_mm"}
_COMPONENT_KEYS = {"cla", "cda", "cop_front", "scale", "drag_exponent", "points", "type", "span_m", "chord_m",
                   "elements", "aoa_deg", "x_mm", "height_mm", "endplate_factor", "e", "cl0", "cl_max", "cd0",
                   "lift_slope_2d", "ground_effect", "k_ground", "h_stall_mm", "range", "area_m2", "cl_ref",
                   "h_ref_mm", "height_exponent", "stall_mm", "rake_gain", "roll_loss", "yaw_loss", "cd_area",
                   "induced_k", "section_data", "section", "section_speed_kmh", "cl_factor",
                   "cd_add", "flap_ranges", "core_area_m2", "exit_area_m2", "loss_coefficient", "inlet_recovery",
                   "effectiveness", "coolant_temp_C", "body_cda", "heat_share", "detail_ranges"}
_ELEMENT_KEYS = {"chord_mm", "deflection_deg", "gap_mm", "overlap_mm", "airfoil", "camber", "thickness", "range"}
_AERO_MODE_KEYS = {"cl", "cd", "cla", "cda", "frontal_area_m2", "cop_front"}

# keys the model reads, per section (anything else in a vehicle file is reported as a possible typo)
KNOWN_KEYS: dict[str, set[str]] = {
    "": {"name", "environment", "mass", "geometry", "tires", "suspension", "brakes", "battery", "aero",
         "powertrain"},
    "environment": {"temperature_C", "pressure_kPa", "altitude_m", "relative_humidity", "air_density_kg_m3"},
    "mass": {"total_kg", "weight_dist_front", "cg_height_mm", "unsprung_front_kg", "unsprung_rear_kg",
             "pitch_inertia_kgm2", "roll_inertia_kgm2", "yaw_inertia_kgm2"},
    "geometry": {"wheelbase_mm", "track_front_mm", "track_rear_mm"},
    "tires": {"radius_mm", "mu_x", "mu_y", "ref_load_N", "load_sensitivity", "min_mu_factor",
              "vertical_rate_N_per_mm", "peak_slip_deg", "camber_optimum_deg", "camber_sensitivity_per_deg2",
              "rolling_resistance"},
    "suspension": {"front", "rear"},
    "suspension.front": _AXLE_KEYS,
    "suspension.rear": _AXLE_KEYS,
    "brakes": {"bias_front"},
    "battery": {"capacity_kWh", "usable_fraction", "max_power_kW", "regen_power_kW", "regen_axle_share", "series",
                "parallel", "soc", "soc_min", "soc_stages", "interconnect_resistance_mOhm", "max_current_A", "cell"},
    "battery.cell": {"capacity_Ah", "resistance_mOhm", "max_current_A", "v_min", "ocv"},
    "aero": {"type", "file", "default_mode", "cl", "cd", "cla", "cda", "frontal_area_m2", "cop_front", "modes",
             "cla_scale", "cda_scale", "cop_shift", "drag_height_mm", "floor_width_mm", "bottoming_clearance_mm",
             "cop_lateral_shift_mm_per_deg", "strategy", "components"},
    "aero.strategy": set(STRATEGY_DEFAULTS),
    "powertrain": {"type", "drive", "differential", "efficiency", "awd_front_fraction", "ev", "ice"},
    "powertrain.ev": {"motor", "motor_variant", "motor_cooling", "dc_voltage_V", "controller_current_A", "n_motors",
                      "reduction_ratio", "drivetrain_efficiency", "inverter_efficiency", "power_limit_kW",
                      "rotor_inertia_kgm2", "motor_torque_curve", "max_rpm"},
    "powertrain.ice": {"torque_curve", "gear_ratios", "final_drive", "primary_ratio", "rev_limit_rpm",
                       "shift_time_s"},
}


def unknown_keys(cfg: dict) -> list[str]:
    """Messages for keys the model does not read (typos are otherwise ignored silently)."""
    import difflib

    out: list[str] = []

    def check(node: dict, section: str, known: set[str]):
        for key, value in node.items():
            if section == "" and str(key).startswith("_"):
                continue
            path = f"{section}.{key}" if section else str(key)
            if key not in known:
                near = difflib.get_close_matches(str(key), sorted(known), n=1, cutoff=0.6)
                out.append(f"unknown key '{path}'" + (f" - did you mean '{near[0]}'?" if near else "")
                           + " (ignored)")
            elif isinstance(value, dict):
                if path in ("aero.components", "aero.modes"):
                    sub = _COMPONENT_KEYS if path == "aero.components" else _AERO_MODE_KEYS
                    for name, comp in value.items():
                        if isinstance(comp, dict):
                            check(comp, f"{path}.{name}", sub)
                            if path == "aero.components" and isinstance(comp.get("elements"), dict):
                                for el, ev in comp["elements"].items():
                                    if isinstance(ev, dict):
                                        check(ev, f"{path}.{name}.elements.{el}", _ELEMENT_KEYS)
                elif path in KNOWN_KEYS:
                    check(value, path, KNOWN_KEYS[path])

    check(cfg, "", KNOWN_KEYS[""])
    return out


def deep_merge(base: dict, override: dict) -> dict:
    """Return a new dict with ``override`` merged recursively on top of ``base``."""
    out = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def apply_defaults(raw: dict) -> dict:
    cfg = deep_merge(DEFAULTS, raw)
    susp = cfg.setdefault("suspension", {})
    for axle in ("front", "rear"):
        susp[axle] = deep_merge(AXLE_DEFAULTS, susp.get(axle, {}) or {})
    cfg["aero"]["strategy"] = deep_merge(STRATEGY_DEFAULTS, cfg["aero"].get("strategy") or {})
    return cfg


def _number(cfg: dict, path: str) -> float:
    value = get_path(cfg, path, None)
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ConfigError(path=path, value=value, condition="숫자여야 함",
                          hint=f"차량 YAML(또는 GUI 파라미터 탭)의 {path}에 숫자를 넣으세요") from None


def _choice(cfg: dict, path: str, allowed: tuple[str, ...]) -> str:
    value = get_path(cfg, path, None)
    if value not in allowed:
        raise ConfigError(path=path, value=value, condition=f"{' | '.join(allowed)} 중 하나",
                          hint=f"{path}를 {', '.join(repr(a) for a in allowed)} 중 하나로 바꾸세요")
    return value


def validate(cfg: dict) -> None:
    """Check the configuration; every error names the parameter, its value and the condition."""
    missing = [p for p in REQUIRED if not has_path(cfg, p)]
    if missing:
        raise ConfigError(path=missing[0], has_value=False, paths=missing,
                          condition="필수 값 (비어 있거나 없음)",
                          message="빠진 값: " + ", ".join(missing),
                          hint="차량 YAML에 위 값을 모두 넣으세요 (예: data/vehicles/fsk2026.yaml 참고)")
    wd = _number(cfg, "mass.weight_dist_front")
    if not 0.0 < wd < 1.0:
        raise ConfigError(path="mass.weight_dist_front", value=wd, condition="0 < 값 < 1 (앞축 무게 비율)",
                          hint="예: 앞 47 %면 0.47")
    for path in ("mass.total_kg", "geometry.wheelbase_mm", "tires.radius_mm",
                 "suspension.front.spring_rate_N_per_mm", "suspension.rear.spring_rate_N_per_mm"):
        v = _number(cfg, path)
        if v <= 0.0:
            raise ConfigError(path=path, value=v, condition="값 > 0", hint=f"{path}에 양수를 넣으세요")
    pt = _choice(cfg, "powertrain.type", ("ev", "ice"))
    if not isinstance(cfg["powertrain"].get(pt), dict):
        raise ConfigError(path=f"powertrain.{pt}", has_value=False,
                          condition=f"powertrain.type = {pt!r}이면 powertrain.{pt}: 섹션이 있어야 함",
                          hint=f"powertrain 아래에 {pt}: 섹션을 넣거나 powertrain.type을 바꾸세요")
    _choice(cfg, "powertrain.drive", ("rwd", "awd"))
    _choice(cfg, "powertrain.differential", ("lsd", "open"))
    _choice(cfg, "aero.strategy.type", ("fixed", "rule", "ideal"))
    bat = cfg.get("battery") or {}
    cell = bat.get("cell")
    if cell:
        for key in ("series", "parallel"):
            v = bat.get(key)
            if v is None or _number(cfg, f"battery.{key}") < 1:
                raise ConfigError(path=f"battery.{key}", value=v, condition="셀 팩 모델(battery.cell)에서는 1 이상",
                                  hint=f"팩의 {'직렬' if key == 'series' else '병렬'} 셀 수를 넣으세요")
        cap = cell.get("capacity_Ah")
        if cap is None or _number(cfg, "battery.cell.capacity_Ah") <= 0:
            raise ConfigError(path="battery.cell.capacity_Ah", value=cap, condition="값 > 0",
                              hint="셀 데이터시트의 용량 [Ah]을 넣으세요")
        soc = _number(cfg, "battery.soc") if bat.get("soc") is not None else 1.0
        soc_min = _number(cfg, "battery.soc_min") if bat.get("soc_min") is not None else 0.05
        if not 0.0 <= soc_min < 1.0:
            raise ConfigError(path="battery.soc_min", value=soc_min, condition="0 <= battery.soc_min < 1",
                              paths=["battery.soc_min", "battery.soc"], hint="보통 0.05 정도")
        if not soc_min < soc <= 1.0:
            raise ConfigError(path="battery.soc", value=soc,
                              condition=f"battery.soc_min ({_fmt(soc_min)}) < battery.soc <= 1 위반",
                              paths=["battery.soc", "battery.soc_min"],
                              hint="시작 SOC(battery.soc)를 최저 SOC(battery.soc_min)보다 크고 1 이하로 넣으세요")
        ocv = cell.get("ocv")
        if ocv is not None:
            try:
                pts = sorted((float(a), float(b)) for a, b in ocv)
            except (TypeError, ValueError):
                raise ConfigError(path="battery.cell.ocv", value=ocv, condition="[SOC, 전압] 쌍의 목록",
                                  hint="예: [[0, 3.0], [0.5, 3.68], [1.0, 4.18]]") from None
            bad = next(((a, b) for a, b in zip(pts, pts[1:]) if b[1] < a[1]), None)
            if len(pts) < 2 or bad:
                raise ConfigError(path="battery.cell.ocv", value=ocv,
                                  condition="점이 2개 이상이고 SOC가 커질수록 전압이 같거나 커야 함"
                                            + (f" (SOC {bad[0][0]:g}: {bad[0][1]:g} V > SOC {bad[1][0]:g}: "
                                               f"{bad[1][1]:g} V)" if bad else ""),
                                  hint="셀 OCV 곡선을 [SOC, V]로 넣으세요")


def load_yaml(path: str | Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: YAML 파일의 맨 위가 key: value 형식(매핑)이 아님")
    return data


def load_vehicle_config(path: str | Path, warn: bool = True) -> dict:
    """Load a vehicle YAML file, apply defaults and validate it.

    Keys the model does not read are listed in ``cfg["_warnings"]`` (and printed to stderr with
    ``warn=True``) - a misspelt key would otherwise silently keep its default.
    """
    path = Path(path)
    cfg = apply_defaults(load_yaml(path))
    cfg["_base_dir"] = str(path.resolve().parent)
    cfg["_source"] = str(path)
    validate(cfg)
    cfg["_warnings"] = unknown_keys(cfg)
    if warn:
        import sys

        for msg in cfg["_warnings"]:
            print(f"warning: {path.name}: {msg}", file=sys.stderr)
    return cfg


def vehicle_config_from_dict(raw: dict, base_dir: str | Path = ".") -> dict:
    cfg = apply_defaults(raw)
    cfg["_base_dir"] = str(Path(base_dir).resolve())
    validate(cfg)
    return cfg


def resolve_file(cfg: dict, file_path: str) -> Path:
    p = Path(file_path)
    if not p.is_absolute():
        p = Path(cfg.get("_base_dir", ".")) / p
    return p


_MISSING = object()


def get_path(cfg: dict, path: str, default: Any = _MISSING) -> Any:
    node: Any = cfg
    for key in path.split("."):
        if isinstance(node, dict) and key in node:
            node = node[key]
        else:
            if default is _MISSING:
                raise KeyError(path)
            return default
    return node


def has_path(cfg: dict, path: str) -> bool:
    return get_path(cfg, path, None) is not None


def set_path(cfg: dict, path: str, value: Any) -> None:
    """Set ``value`` at dotted ``path`` in place (intermediate dicts are created)."""
    keys = path.split(".")
    node = cfg
    for key in keys[:-1]:
        nxt = node.get(key)
        if not isinstance(nxt, dict):
            nxt = {}
            node[key] = nxt
        node = nxt
    node[keys[-1]] = value


def with_overrides(cfg: dict, overrides: dict[str, Any] | None, context: str = "") -> dict:
    """Deep-copied config with dotted-path overrides applied and re-validated.

    If the result is invalid the error says which applied value(s) caused it and, with
    ``context``, in which internal step (e.g. "내구 SOC 단계 3/3").
    """
    out = copy.deepcopy(cfg)
    for path, value in (overrides or {}).items():
        set_path(out, path, value)
    try:
        validate(out)
    except ConfigError as err:
        applied = ", ".join(f"{p}={_fmt(v)}" for p, v in (overrides or {}).items())
        where = f"내부 계산 - {context}" if context else "덮어쓴 값"
        raise err.with_context(f"{where} (적용한 값: {applied})" if applied else where) from None
    return out


def parse_override(text: str) -> tuple[str, Any]:
    """Parse ``"a.b.c=value"``; the value is interpreted as YAML (numbers, bools, lists)."""
    if "=" not in text:
        raise ConfigError(path="--set", value=text, condition="경로=값 형식 (예: mass.total_kg=300)")
    path, raw = text.split("=", 1)
    return path.strip(), yaml.safe_load(raw)


def relocate_paths(cfg: dict, target_dir: str | Path) -> dict:
    """Copy of ``cfg`` without the private keys and with its data-file paths (aero map, wing section
    data) rewritten relative to ``target_dir``, so a YAML saved elsewhere still finds its files."""
    import os

    out = copy.deepcopy({k: v for k, v in cfg.items() if not k.startswith("_")})
    base = Path(cfg.get("_base_dir", "."))
    target = Path(target_dir).resolve()

    def fix(node: dict, key: str):
        val = node.get(key)
        if isinstance(val, str) and val:
            src = Path(val) if Path(val).is_absolute() else (base / val)
            try:
                node[key] = Path(os.path.relpath(src.resolve(), target)).as_posix()
            except ValueError:  # other drive on Windows
                node[key] = src.resolve().as_posix()

    aero = out.get("aero") or {}
    fix(aero, "file")
    for comp in (aero.get("components") or {}).values():
        if isinstance(comp, dict):
            fix(comp, "section_data")
    return out


def dump_config(cfg: dict, path: str | Path) -> None:
    clean = relocate_paths(cfg, Path(path).resolve().parent)
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(clean, fh, sort_keys=False, allow_unicode=True)
