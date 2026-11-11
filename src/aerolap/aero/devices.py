"""Physics-based aero devices: wings (per-element geometry or simple), sidepods with radiators, floor.

Used for the entries of ``aero.components`` that have a ``type`` (a component without ``type``
keeps the simple ``cla`` x ``scale`` model of :class:`aerolap.aero.maps.AeroComponent`). The
devices respond to the car's attitude: ride heights (heave and pitch at the device position),
roll and yaw, so suspension movement changes their downforce and drag even without a CFD map.

``type: wing``  (front / rear wing; downforce positive)::

    span_m, chord_m (sum of the elements), elements (1..3), aoa_deg (main-element angle, design
    variable), x_mm (centre of pressure behind the front axle; negative = ahead of it),
    height_mm (lowest point above the ground at the static ride heights), endplate_factor (effective
    aspect-ratio gain from end plates), ground_effect (default: true when height < chord),
    optional cl0, cl_max, cd0, e (span efficiency), lift_slope_2d (1/rad), range [lo, hi] (deg)

    AR_eff = endplate_factor * span / chord,  a = a0 / (1 + a0 / (pi e AR_eff))
    CL = soft_stall(cl0 + a alpha) * ground(h) * cos^2(yaw),  CD = cd0 (1 + (alpha/15)^2) + CL^2 / (pi e AR_eff)
    ground(h) = 1 + k_ground * chord / h, falling below h_stall_mm (ground stall); the two wing tips
    sit at h -/+ span/2 sin(roll) and the effect is averaged

    With ``elements: {main: {...}, flap1: {...}, ...}`` the section comes from the per-element model
    of :mod:`aerolap.aero.elements` (main aoa = ``aoa_deg``, flap angles to the main chord line,
    gaps, overlaps) and goes through the same 3-D path as 2-D CFD data (``section_data``).

``type: sidepod``  (radiator duct; see :class:`Sidepod`)::

    core_area_m2, exit_area_m2 (design variable), loss_coefficient, inlet_recovery, effectiveness,
    coolant_temp_C, heat_share, body_cda, cla, x_mm

``type: floor``  (undertray + diffuser)::

    area_m2, cl_ref (at h_ref_mm), height_mm (static clearance at x), height_exponent, stall_mm,
    rake_gain (per deg of extra rake), roll_loss / yaw_loss (per deg), cd_area (CdA of the
    sidepods / floor at scale 1), induced_k, scale (design variable: floor / diffuser size)

    ClA = scale cl_ref area (h_ref / h)^n (1 + rake_gain d_rake) (1 - roll_loss |roll|) (1 - yaw_loss |yaw|)
    below stall_mm the flow chokes: ClA(h) = ClA(stall_mm) (h / stall_mm)^2

All lengths in the configuration are in mm except span / chord (m) and areas (m^2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# per number of elements: zero-angle lift coefficient, maximum lift coefficient, profile drag (typical
# FSAE values incl. end-plate and mount drag; override with cl0 / cl_max / cd0 from your CFD)
WING_DEFAULTS = {1: (0.6, 1.6, 0.08), 2: (1.2, 2.6, 0.20), 3: (1.6, 3.2, 0.30)}  # cd0 incl. end plates, mounts


@dataclass
class Geometry:
    """Car reference needed to place devices: static ride heights (mm) and wheelbase (mm)."""

    rh_f0: float = 30.0
    rh_r0: float = 40.0
    wheelbase: float = 1550.0

    def local_change(self, x_mm: float, rh_f, rh_r):
        """Change of the ground clearance at ``x_mm`` behind the front axle (heave + pitch)."""
        df = np.asarray(rh_f, dtype=float) - self.rh_f0
        dr = np.asarray(rh_r, dtype=float) - self.rh_r0
        return df + (dr - df) * (x_mm / self.wheelbase)


def _soft_stall(cl_lin, cl_max):
    """Linear up to 0.85 cl_max, smooth roll-over to cl_max, then a drop past the stall."""
    x = np.asarray(cl_lin, dtype=float) / cl_max
    knee = 0.85
    y = np.where(x <= knee, x, knee + (1.0 - knee) * np.tanh((x - knee) / (1.0 - knee)))
    post = np.maximum(x - 1.15, 0.0)
    return cl_max * (y - 0.6 * post), np.maximum(x - 1.0, 0.0)


@dataclass
class Wing:
    name: str
    span: float  # m
    chord: float  # m, all elements
    elements: int
    aoa: float  # deg
    x: float  # mm behind the front axle
    height: float  # mm, static
    endplate: float = 1.5
    e: float = 0.9
    cl0: float | None = None
    cl_max: float | None = None
    cd0: float | None = None
    a0: float = 2.0 * math.pi
    ground_effect: bool | None = None
    k_ground: float = 0.1
    h_stall: float = 30.0  # mm, ground stall of a front wing
    cop_front: float | None = None
    range: tuple[float, float] = (-5.0, 15.0)
    design_key: str = "aoa_deg"
    section: object = None  # SectionPolar from 2-D CFD (AERO_AUTOMATION); replaces cl0 / slope / stall / cd0 / ground
    section_speed: float | None = None  # km/h at which the section data are read (Re reference)
    cl_factor: float = 1.0  # 3-D calibration of the section lift
    cd_add: float = 0.03  # section mode: end plates, mounts, gurney / tip losses (per reference area)
    model: object = None  # ElementSection (per-element geometry, aerolap.aero.elements); builds ``section``
    flap_ranges: dict | None = None  # flap name -> (lo, hi) deg for the optimisers

    def __post_init__(self):
        if self.model is not None and self.section is None:
            from .elements import element_polar

            if self.ground_effect is None:
                self.ground_effect = self.height < sum(e.chord_mm for e in self.model.elements)
            self.section = element_polar(self.model, bool(self.ground_effect), round(float(self.height), 1))
            self.elements = len(self.model.elements)
            rng = self.range
        else:
            rng = None
        if self.section is not None:
            self.chord = self.section.c_ref
            self.ground_effect = self.section.has_ground
            self.range = (float(self.section.aoa[0]), float(self.section.aoa[-1]))
            if self.section_speed is None and self.section.speeds is not None:
                self.section_speed = float(np.median(self.section.speeds))
        n = min(max(int(self.elements), 1), 3)
        d = WING_DEFAULTS[n]
        self.cl0 = d[0] if self.cl0 is None else self.cl0
        self.cl_max = d[1] if self.cl_max is None else self.cl_max
        self.cd0 = d[2] if self.cd0 is None else self.cd0
        if self.ground_effect is None:
            self.ground_effect = self.height < 1000.0 * self.chord
        if rng is not None:  # element model: the design range, not the table's aoa grid
            self.range = rng

    @property
    def cfd(self) -> bool:
        """Section data from CFD (not the element model)."""
        return self.section is not None and getattr(self.section, "model", None) is None

    @property
    def design_value(self) -> float:
        return self.aoa

    def design_variables(self) -> list[tuple[str, str, tuple[float, float], str, float]]:
        """(suffix, config key, range, unit, value): main aoa, then each flap angle (element model)."""
        out = [("", "aoa_deg", tuple(self.range), "deg", self.aoa)]
        if self.model is not None and getattr(self.section, "model", None) is not None:
            for f in self.model.flaps:
                rng = (self.flap_ranges or {}).get(f.name) or (max(f.deflection - 15.0, 0.0), min(f.deflection + 15.0, 80.0))
                out.append((f".{f.name}", f"elements.{f.name}.deflection_deg", tuple(rng), "deg", f.deflection))
        return out

    def element_states(self, geo: Geometry | None = None) -> list[dict]:
        """Per element at the wing's effective (induced-angle corrected) angle, static attitude."""
        if self.model is None:
            return []
        return self.model.states(self._alpha_eff(geo))

    def _alpha_eff(self, geo: Geometry | None = None) -> float:
        cla, _ = self._section_coefficients(geo or Geometry(0.0, 0.0, 1.0), (geo.rh_f0 if geo else 0.0),
                                            (geo.rh_r0 if geo else 0.0), 0.0, 0.0)
        return self.aoa - math.degrees(float(cla) / self.area / (math.pi * self.e * self.ar_eff))

    @property
    def area(self) -> float:
        return self.span * self.chord

    @property
    def ar_eff(self) -> float:
        return self.endplate * self.span / self.chord

    @property
    def lift_slope(self) -> float:
        """3-D lift slope per rad (finite wing)."""
        return self.a0 / (1.0 + self.a0 / (math.pi * self.e * self.ar_eff))

    def stall_ratio(self) -> float:
        """Free-stream lift over the maximum lift (>= 1: stalled; ground effect not included).

        With section data: aoa over the aoa of maximum section lift at the static clearance; with the
        element model: the most loaded element (its own lift over its own maximum)."""
        if self.model is not None and not self.cfd:
            return max(st["ratio"] for st in self.model.states(self._alpha_eff()))
        if self.section is not None:  # effective (induced-angle corrected) aoa vs the section's max-lift aoa
            h = self.height if self.section.has_ground else None
            a_max, _ = self.section.max_cl(h, self.section_speed)
            cla, _ = self._section_coefficients(Geometry(0.0, 0.0, 1.0), 0.0, 0.0, 0.0, 0.0)
            a_eff = self.aoa - math.degrees(float(cla) / self.area / (math.pi * self.e * self.ar_eff))
            return 1.0 if a_eff >= a_max else max(0.0, 1.0 - (a_max - a_eff) / 10.0)
        return (self.cl0 + self.lift_slope * math.radians(self.aoa)) / self.cl_max

    def _section_curve(self, h):
        """3-D CL, CD of the section at this wing's angle (lifting-line fixed point) at clearance(s) h."""
        sec = self.section
        k = math.pi * self.e * self.ar_eff
        cl = self.cl_factor * sec.cl_at(np.full(np.shape(h) if h is not None else (), self.aoa), h, self.section_speed)
        for _ in range(40):  # induced angle from the 3-D lift (relaxed fixed point)
            a_eff = self.aoa - np.degrees(cl / k)
            new = 0.5 * cl + 0.5 * self.cl_factor * sec.cl_at(a_eff, h, self.section_speed)
            done = np.max(np.abs(new - cl)) < 1e-7
            cl = new
            if done:
                break
        a_eff = self.aoa - np.degrees(cl / k)
        return cl, sec.cd_at(a_eff, h, self.section_speed) + self.cd_add + cl ** 2 / k

    def _section_table(self):
        """Pre-computed (h grid, CL, CD) - the angle does not change during a lap."""
        key = (self.aoa, self.section_speed, self.cl_factor, self.cd_add, self.ar_eff, self.e)
        if getattr(self, "_tab_key", None) != key:
            if self.section.has_ground:
                hs = self.section.heights
                grid = np.linspace(min(hs[0], 1.0), hs[-1] + 0.5 * (hs[-1] - hs[0]), 241)
                cl, cd = self._section_curve(grid)
            else:
                grid = None
                cl, cd = (float(x) for x in self._section_curve(None))
            self._tab, self._tab_key = (grid, cl, cd), key
        return self._tab

    def _section_coefficients(self, geo: Geometry, rh_f, rh_r, roll_deg, yaw_deg):
        """2-D CFD section + lifting-line correction: alpha_eff = alpha - CL / (pi e AR), CDi = CL^2 / (pi e AR)."""
        shape = np.broadcast(np.asarray(rh_f), np.asarray(rh_r), np.asarray(roll_deg), np.asarray(yaw_deg)).shape
        grid, cl_t, cd_t = self._section_table()
        if grid is None:  # free stream (rear wing): independent of the ride heights
            cl, cd = np.full(shape, cl_t), np.full(shape, cd_t)
        else:
            h = self.height + geo.local_change(self.x, rh_f, rh_r)
            dz = 500.0 * self.span * np.sin(np.radians(np.abs(roll_deg)))  # the two tips
            lo, hi = np.broadcast_to(h - dz, shape), np.broadcast_to(h + dz, shape)
            cl = 0.5 * (np.interp(lo, grid, cl_t) + np.interp(hi, grid, cl_t))
            cd = 0.5 * (np.interp(lo, grid, cd_t) + np.interp(hi, grid, cd_t))
        cy = np.cos(np.radians(yaw_deg)) ** 2
        k = math.pi * self.e * self.ar_eff
        cl_y = cl * cy
        cd = cd - (cl ** 2 - cl_y ** 2) / k  # less induced drag with the lift lost in yaw
        return cl_y * self.area, cd * self.area

    def section_out_of_range(self, geo: Geometry, rh_f, rh_r) -> np.ndarray:
        if self.section is None:
            return np.zeros(np.shape(rh_f), dtype=bool)
        h = (self.height + geo.local_change(self.x, rh_f, rh_r)) if self.section.has_ground else None
        return self.section.out_of_range(np.full(np.shape(rh_f), self.aoa), h, self.section_speed)

    def _ground(self, h_mm):
        h = np.maximum(np.asarray(h_mm, dtype=float), 1.0)
        g = 1.0 + self.k_ground * (1000.0 * self.chord) / h
        return g * np.where(h < self.h_stall, (h / self.h_stall) ** 2, 1.0)  # ground stall

    def coefficients(self, geo: Geometry, rh_f, rh_r, roll_deg, yaw_deg):
        """ClA, CdA (m^2) at the given attitude (arrays broadcast)."""
        if self.section is not None:
            return self._section_coefficients(geo, rh_f, rh_r, roll_deg, yaw_deg)
        alpha = math.radians(self.aoa)
        cl, over = _soft_stall(self.cl0 + self.lift_slope * alpha, self.cl_max)
        cl = cl * np.ones(np.broadcast(np.asarray(rh_f), np.asarray(rh_r), np.asarray(roll_deg)).shape)
        if self.ground_effect:
            h = self.height + geo.local_change(self.x, rh_f, rh_r)
            dz = 500.0 * self.span * np.sin(np.radians(np.abs(roll_deg)))  # tip height change, mm
            g = 0.5 * (self._ground(h - dz) + self._ground(h + dz))
            cl = cl * g
        cl = cl * np.cos(np.radians(yaw_deg)) ** 2
        cd = (self.cd0 * (1.0 + (self.aoa / 15.0) ** 2) + 0.5 * over ** 2
              + cl ** 2 / (math.pi * self.e * self.ar_eff))
        return cl * self.area, cd * self.area

    def front_share(self, geo: Geometry) -> float:
        return self.cop_front if self.cop_front is not None else 1.0 - self.x / geo.wheelbase


@dataclass
class Floor:
    name: str
    area: float  # m^2
    cl_ref: float
    height: float  # mm, static clearance at x
    x: float  # mm behind the front axle
    h_ref: float | None = None
    exponent: float = 0.6
    stall: float = 12.0  # mm
    rake_gain: float = 0.05  # per deg of extra rake
    roll_loss: float = 0.05  # per deg
    yaw_loss: float = 0.02  # per deg
    cd_area: float = 0.04
    induced_k: float = 0.02
    scale: float = 1.0
    cop_front: float | None = None
    range: tuple[float, float] = (0.5, 1.6)
    design_key: str = "scale"

    def __post_init__(self):
        if self.h_ref is None:
            self.h_ref = self.height

    @property
    def design_value(self) -> float:
        return self.scale

    def design_variables(self):
        return [("", "scale", tuple(self.range), "x", self.scale)]

    def _height_factor(self, h_mm):
        h = np.maximum(np.asarray(h_mm, dtype=float), 0.5)
        above = (self.h_ref / np.maximum(h, self.stall)) ** self.exponent
        return above * np.where(h < self.stall, (h / self.stall) ** 2, 1.0)

    def coefficients(self, geo: Geometry, rh_f, rh_r, roll_deg, yaw_deg):
        h = self.height + geo.local_change(self.x, rh_f, rh_r)
        rake0 = math.degrees(math.atan((geo.rh_r0 - geo.rh_f0) / geo.wheelbase))
        rake = np.degrees(np.arctan((np.asarray(rh_r, dtype=float) - np.asarray(rh_f, dtype=float)) / geo.wheelbase))
        f = self._height_factor(h) * np.maximum(1.0 + self.rake_gain * (rake - rake0), 0.3)
        f = f * np.maximum(1.0 - self.roll_loss * np.abs(roll_deg), 0.2) * np.maximum(1.0 - self.yaw_loss * np.abs(yaw_deg), 0.2)
        s = max(self.scale, 0.0)
        cla = s * self.cl_ref * self.area * f
        cda = s * self.cd_area + self.induced_k * cla ** 2 / max(self.area * s, 1e-6)
        return cla, cda

    def front_share(self, geo: Geometry) -> float:
        return self.cop_front if self.cop_front is not None else 1.0 - self.x / geo.wheelbase


CP_AIR = 1005.0  # J/(kg K)


@dataclass
class Sidepod:
    """Sidepod with a radiator: cooling drag from the momentum the cooling air loses, heat rejection.

    Inlet -> radiator core (area ``core_area``, pressure-loss coefficient ``loss_k`` on the core
    dynamic pressure) -> exit (``exit_area``, back to ambient pressure). Bernoulli + continuity:

        v_core / V = sqrt(eta_in / (K + (A_core / A_exit)^2))
        CdA_cool   = 2 A_core (v_core / V) (1 - (A_core / A_exit) (v_core / V))     (momentum deficit)
        Q          = effectiveness rho A_core v_core c_p (T_coolant - T_air)           (proportional to V)

    A bigger exit pulls more air through the core: more cooling, more drag. ``exit_area_m2`` is the
    design variable; ``cooling_margin_kW`` in the constraints keeps the heat balance.
    """

    name: str
    core_area: float  # m^2 (both sides)
    exit_area: float  # m^2
    loss_k: float = 8.0
    inlet_recovery: float = 0.9
    effectiveness: float = 0.5
    coolant_temp: float = 60.0  # degC
    body_cda: float = 0.06  # m^2, outer shape of the sidepods
    cla: float = 0.0  # m^2, sidepod / side-wing-less downforce (e.g. from the pod's top surface)
    x: float = 800.0
    cop_front: float | None = None
    range: tuple[float, float] | None = None
    design_key: str = "exit_area_m2"
    height: float = 0.0

    def __post_init__(self):
        if self.range is None:
            self.range = (round(0.3 * self.core_area, 4), round(1.5 * self.core_area, 4))

    @property
    def design_value(self) -> float:
        return self.exit_area

    def design_variables(self):
        return [("", "exit_area_m2", tuple(self.range), "m2", self.exit_area)]

    @property
    def flow_ratio(self) -> float:
        """v_core / V."""
        r = self.core_area / max(self.exit_area, 1e-9)
        return math.sqrt(self.inlet_recovery / (self.loss_k + r * r))

    @property
    def cooling_cda(self) -> float:
        f = self.flow_ratio
        r = self.core_area / max(self.exit_area, 1e-9)
        return 2.0 * self.core_area * f * (1.0 - r * f)

    def heat_rejection_kW(self, v_mps, rho: float = 1.225, t_air: float = 25.0):
        """Heat the radiator gives off at speed ``v_mps`` (kW)."""
        v_core = self.flow_ratio * np.asarray(v_mps, dtype=float)
        return self.effectiveness * rho * self.core_area * v_core * CP_AIR * (self.coolant_temp - t_air) / 1000.0

    def coefficients(self, geo: Geometry, rh_f, rh_r, roll_deg, yaw_deg):
        shape = np.broadcast(np.asarray(rh_f), np.asarray(rh_r), np.asarray(roll_deg), np.asarray(yaw_deg)).shape
        return np.full(shape, self.cla), np.full(shape, self.body_cda + self.cooling_cda)

    def front_share(self, geo: Geometry) -> float:
        return self.cop_front if self.cop_front is not None else 1.0 - self.x / geo.wheelbase


REQUIRED_DEVICE_KEYS = {"wing": ("span_m", "chord_m"), "wing+section": ("span_m",), "wing+elements": ("span_m",),
                        "floor": ("area_m2", "cl_ref"), "sidepod": ("core_area_m2", "exit_area_m2")}


def device_from_config(name: str, c: dict, base_dir: str | Path = "."):
    """Device from ``aero.components.<name>``; every input error names the parameter and the condition."""
    from ..config import ConfigError

    pre = f"aero.components.{name}"
    kind = str(c.get("type", "")).lower()
    if kind not in ("wing", "floor", "sidepod"):
        raise ConfigError(path=f"{pre}.type", value=c.get("type"),
                          condition="'wing' | 'floor' | 'sidepod' (또는 type 생략 = 단순 모델)",
                          hint="장치 종류를 wing(윙, 사이드 윙 포함), floor(바닥), sidepod(사이드포드·라디에이터)로 쓰세요")
    has_el = kind == "wing" and isinstance(c.get("elements"), (dict, list))
    need = REQUIRED_DEVICE_KEYS["wing+section" if kind == "wing" and c.get("section_data")
                                else "wing+elements" if has_el else kind]
    missing = [k for k in need if c.get(k) is None]
    if missing:
        raise ConfigError(path=f"{pre}.{missing[0]}", has_value=False, paths=[f"{pre}.{k}" for k in missing],
                          condition=f"type: {kind} 장치의 필수 값", message="빠진 값: " + ", ".join(f"{pre}.{k}" for k in missing))
    for k, v in c.items():
        if k in ("type", "section", "section_data", "range", "ground_effect", "cop_front", "cl0", "cl_max", "cd0",
                 "h_ref_mm", "section_speed_kmh", "flap_ranges") or (k == "elements" and has_el):
            continue
        try:
            float(v)
        except (TypeError, ValueError):
            raise ConfigError(path=f"{pre}.{k}", value=v, condition="숫자여야 함") from None
    try:
        return _device(name, kind, c, base_dir)
    except ConfigError:
        raise
    except ValueError as err:
        if has_el and c.get("section_data") and not c.get("section"):
            raise ConfigError(str(err), path=f"{pre}.elements", value="(엘리먼트 값)", paths=[f"{pre}.elements", f"{pre}.section_data"],
                              condition="section_data 파일에 이 엘리먼트 값(코드·각·갭·오버랩)의 형상이 있어야 함",
                              hint="엘리먼트 값을 CFD 형상에 맞추거나, section: <형상 id>로 직접 고르거나, section_data를 지워 "
                                   "엘리먼트 모델을 쓰세요") from None
        key = "section" if "section" in str(err) else ("section_data" if c.get("section_data") else "type")
        raise ConfigError(str(err), path=f"{pre}.{key}", value=c.get(key),
                          condition="CFD 단면 파일에서 형상 하나를 고를 수 있어야 함" if key == "section" else "") from None
    except FileNotFoundError:
        raise ConfigError(path=f"{pre}.section_data", value=c.get("section_data"), condition="파일이 있어야 함",
                          hint=f"경로는 차량 YAML 폴더({base_dir}) 기준입니다") from None


def _flap_ranges(c: dict) -> dict:
    out = {}
    for fname, e in (c.get("elements") or {}).items() if isinstance(c.get("elements"), dict) else ():
        if isinstance(e, dict) and e.get("range") is not None:
            out[fname] = (float(e["range"][0]), float(e["range"][1]))
    return out


def _device(name: str, kind: str, c: dict, base_dir):
    rng = c.get("range")
    rng = None if rng is None else (float(rng[0]), float(rng[1]))
    model = None
    if kind == "wing" and isinstance(c.get("elements"), (dict, list)):
        from .elements import parse_elements

        el = {k: ({kk: vv for kk, vv in v.items() if kk != "range"} if isinstance(v, dict) else v)
              for k, v in c["elements"].items()} if isinstance(c["elements"], dict) else c["elements"]
        model = parse_elements(el, f"aero.components.{name}.elements")
    if kind == "wing" and c.get("section_data"):
        from .section_data import load_sections

        path = Path(c["section_data"])
        if not path.is_absolute():
            path = Path(base_dir) / path
        sset = load_sections(path)
        if model is not None and not c.get("section"):
            from .elements import select_cfd

            sec = select_cfd(sset, model)
        else:
            sec = sset.select(c.get("section"))
        n_el = len(model.elements) if model is not None else int(c.get("elements", 2))
        w = Wing(name, float(c["span_m"]), sec.c_ref, n_el, float(c.get("aoa_deg", 0.0)),
                 float(c.get("x_mm", 0.0)), float(c.get("height_mm", 100.0)), float(c.get("endplate_factor", 1.5)),
                 float(c.get("e", 0.9)), cop_front=c.get("cop_front"), section=sec,
                 section_speed=None if c.get("section_speed_kmh") is None else float(c["section_speed_kmh"]),
                 cl_factor=float(c.get("cl_factor", 1.0)), cd_add=float(c.get("cd_add", 0.03)), model=model)
        if rng:
            w.range = rng
        return w
    if kind == "wing" and model is not None:
        ge = c.get("ground_effect")
        return Wing(name, float(c["span_m"]), model.c0 / 1000.0, len(model.elements), float(c.get("aoa_deg", 0.0)),
                    float(c.get("x_mm", 0.0)), float(c.get("height_mm", 100.0)), float(c.get("endplate_factor", 1.5)),
                    float(c.get("e", 0.9)), ground_effect=None if ge is None else bool(ge), cop_front=c.get("cop_front"),
                    range=rng or (-5.0, 15.0), cl_factor=float(c.get("cl_factor", 1.0)),
                    cd_add=float(c.get("cd_add", 0.03)), model=model, flap_ranges=_flap_ranges(c))
    if kind == "wing":
        w = Wing(name, float(c["span_m"]), float(c["chord_m"]), int(c.get("elements", 1)), float(c.get("aoa_deg", 0.0)),
                 float(c.get("x_mm", 0.0)), float(c.get("height_mm", 100.0)), float(c.get("endplate_factor", 1.5)),
                 float(c.get("e", 0.9)), c.get("cl0"), c.get("cl_max"), c.get("cd0"),
                 float(c.get("lift_slope_2d", 2.0 * math.pi)), c.get("ground_effect"),
                 float(c.get("k_ground", 0.1)), float(c.get("h_stall_mm", 30.0)), c.get("cop_front"))
        if rng:
            w.range = rng
        return w
    if kind == "floor":
        f = Floor(name, float(c["area_m2"]), float(c["cl_ref"]), float(c.get("height_mm", 40.0)),
                  float(c.get("x_mm", 800.0)), c.get("h_ref_mm"), float(c.get("height_exponent", 0.6)),
                  float(c.get("stall_mm", 12.0)), float(c.get("rake_gain", 0.05)), float(c.get("roll_loss", 0.05)),
                  float(c.get("yaw_loss", 0.02)), float(c.get("cd_area", 0.04)), float(c.get("induced_k", 0.02)),
                  float(c.get("scale", 1.0)), c.get("cop_front"))
        if rng:
            f.range = rng
        return f
    if kind == "sidepod":
        from ..config import ConfigError

        pre = f"aero.components.{name}"
        for k, cond in (("core_area_m2", 0.0), ("exit_area_m2", 0.0), ("loss_coefficient", -1e-12)):
            if k in c and not float(c[k]) > cond:
                raise ConfigError(path=f"{pre}.{k}", value=c[k], condition="> 0" if cond == 0.0 else ">= 0")
        if not 0.0 < float(c.get("inlet_recovery", 0.9)) <= 1.0:
            raise ConfigError(path=f"{pre}.inlet_recovery", value=c.get("inlet_recovery"), condition="0 < 값 <= 1")
        if not 0.0 < float(c.get("effectiveness", 0.5)) <= 1.0:
            raise ConfigError(path=f"{pre}.effectiveness", value=c.get("effectiveness"), condition="0 < 값 <= 1")
        return Sidepod(name, float(c["core_area_m2"]), float(c["exit_area_m2"]), float(c.get("loss_coefficient", 8.0)),
                       float(c.get("inlet_recovery", 0.9)), float(c.get("effectiveness", 0.5)),
                       float(c.get("coolant_temp_C", 60.0)), float(c.get("body_cda", 0.06)), float(c.get("cla", 0.0)),
                       float(c.get("x_mm", 800.0)), c.get("cop_front"), rng)
    raise ValueError(f"unknown device type {kind!r}")


DEVICE_KEYS = {"type", "span_m", "chord_m", "elements", "aoa_deg", "x_mm", "height_mm", "endplate_factor", "e", "cl0",
               "cl_max", "cd0", "lift_slope_2d", "ground_effect", "k_ground", "h_stall_mm", "cop_front", "range",
               "area_m2", "cl_ref", "h_ref_mm", "height_exponent", "stall_mm", "rake_gain", "roll_loss", "yaw_loss",
               "cd_area", "induced_k", "scale", "section_data", "section", "section_speed_kmh", "cl_factor",
               "cd_add", "flap_ranges", "core_area_m2", "exit_area_m2", "loss_coefficient", "inlet_recovery",
               "effectiveness", "coolant_temp_C", "body_cda", "heat_share", "cla"}


def geometry_from_cfg(cfg: dict) -> Geometry:
    try:
        sus = cfg["suspension"]
        return Geometry(float(sus["front"]["static_ride_height_mm"]), float(sus["rear"]["static_ride_height_mm"]),
                        float(cfg["geometry"]["wheelbase_mm"]))
    except (KeyError, TypeError):
        return Geometry()
