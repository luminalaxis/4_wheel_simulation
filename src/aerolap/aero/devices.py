"""Physics-based aero devices: multi-element wings and an underbody (floor / diffuser).

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

``type: floor``  (undertray + diffuser + sidepods)::

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

    def __post_init__(self):
        n = min(max(int(self.elements), 1), 3)
        d = WING_DEFAULTS[n]
        self.cl0 = d[0] if self.cl0 is None else self.cl0
        self.cl_max = d[1] if self.cl_max is None else self.cl_max
        self.cd0 = d[2] if self.cd0 is None else self.cd0
        if self.ground_effect is None:
            self.ground_effect = self.height < 1000.0 * self.chord

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
        """Free-stream lift over the maximum lift (>= 1: stalled; ground effect not included)."""
        return (self.cl0 + self.lift_slope * math.radians(self.aoa)) / self.cl_max

    def _ground(self, h_mm):
        h = np.maximum(np.asarray(h_mm, dtype=float), 1.0)
        g = 1.0 + self.k_ground * (1000.0 * self.chord) / h
        return g * np.where(h < self.h_stall, (h / self.h_stall) ** 2, 1.0)  # ground stall

    def coefficients(self, geo: Geometry, rh_f, rh_r, roll_deg, yaw_deg):
        """ClA, CdA (m^2) at the given attitude (arrays broadcast)."""
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


def device_from_config(name: str, c: dict):
    kind = str(c.get("type", "")).lower()
    rng = c.get("range")
    rng = None if rng is None else (float(rng[0]), float(rng[1]))
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
    raise ValueError(f"aero.components.{name}.type must be 'wing' or 'floor' (or omitted for the simple model)")


DEVICE_KEYS = {"type", "span_m", "chord_m", "elements", "aoa_deg", "x_mm", "height_mm", "endplate_factor", "e", "cl0",
               "cl_max", "cd0", "lift_slope_2d", "ground_effect", "k_ground", "h_stall_mm", "cop_front", "range",
               "area_m2", "cl_ref", "h_ref_mm", "height_exponent", "stall_mm", "rake_gain", "roll_loss", "yaw_loss",
               "cd_area", "induced_k", "scale"}


def geometry_from_cfg(cfg: dict) -> Geometry:
    try:
        sus = cfg["suspension"]
        return Geometry(float(sus["front"]["static_ride_height_mm"]), float(sus["rear"]["static_ride_height_mm"]),
                        float(cfg["geometry"]["wheelbase_mm"]))
    except (KeyError, TypeError):
        return Geometry()
