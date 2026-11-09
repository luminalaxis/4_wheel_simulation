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
    """Raised for missing or invalid configuration values."""


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


def validate(cfg: dict) -> None:
    missing = [p for p in REQUIRED if not has_path(cfg, p)]
    if missing:
        raise ConfigError("missing required vehicle parameters: " + ", ".join(missing))
    wd = float(get_path(cfg, "mass.weight_dist_front"))
    if not 0.0 < wd < 1.0:
        raise ConfigError("mass.weight_dist_front must be between 0 and 1")
    for path in ("mass.total_kg", "geometry.wheelbase_mm", "tires.radius_mm",
                 "suspension.front.spring_rate_N_per_mm", "suspension.rear.spring_rate_N_per_mm"):
        if float(get_path(cfg, path)) <= 0.0:
            raise ConfigError(f"{path} must be positive")
    pt = cfg["powertrain"]["type"]
    if pt not in ("ev", "ice"):
        raise ConfigError("powertrain.type must be 'ev' or 'ice'")
    if pt not in cfg["powertrain"]:
        raise ConfigError(f"powertrain.{pt} section is required for powertrain.type: {pt}")
    if cfg["powertrain"]["drive"] not in ("rwd", "awd"):
        raise ConfigError("powertrain.drive must be 'rwd' or 'awd'")
    if cfg["powertrain"]["differential"] not in ("lsd", "open"):
        raise ConfigError("powertrain.differential must be 'lsd' or 'open'")
    if cfg["aero"]["strategy"]["type"] not in ("fixed", "rule", "ideal"):
        raise ConfigError("aero.strategy.type must be fixed, rule or ideal")


def load_yaml(path: str | Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: top level of a YAML config must be a mapping")
    return data


def load_vehicle_config(path: str | Path) -> dict:
    """Load a vehicle YAML file, apply defaults and validate it."""
    path = Path(path)
    cfg = apply_defaults(load_yaml(path))
    cfg["_base_dir"] = str(path.resolve().parent)
    cfg["_source"] = str(path)
    validate(cfg)
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


def with_overrides(cfg: dict, overrides: dict[str, Any] | None) -> dict:
    """Deep-copied config with dotted-path overrides applied and re-validated."""
    out = copy.deepcopy(cfg)
    for path, value in (overrides or {}).items():
        set_path(out, path, value)
    validate(out)
    return out


def parse_override(text: str) -> tuple[str, Any]:
    """Parse ``"a.b.c=value"``; the value is interpreted as YAML (numbers, bools, lists)."""
    if "=" not in text:
        raise ConfigError(f"override must look like path=value, got {text!r}")
    path, raw = text.split("=", 1)
    return path.strip(), yaml.safe_load(raw)


def dump_config(cfg: dict, path: str | Path) -> None:
    clean = {k: v for k, v in cfg.items() if not k.startswith("_")}
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(clean, fh, sort_keys=False, allow_unicode=True)
