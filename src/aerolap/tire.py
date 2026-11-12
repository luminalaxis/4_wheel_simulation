"""Tyre grip models: load sensitivity, camber sensitivity, friction ellipse, slip curve.

Two models share one interface (used by :class:`~aerolap.vehicle.VehicleModel` and the transient solver):

* :class:`Tire` (``tires:`` without ``mf_file``) - the simple engineering model: ``mu(Fz)`` linear in
  load, a parabolic camber loss around an optimum, one peak slip angle;
* :class:`~aerolap.tire_mf.MagicFormulaTire` (``tires.mf_file`` = Pacejka ``.tir``) - Magic Formula
  5.2 / 6.1 peak forces, load-dependent cornering stiffness and slip curve, camber thrust, inflation
  pressure, combined-slip envelope (see :mod:`aerolap.tire_mf`).

Interface:

* ``capacity(fz, inclination, camber)`` - peak longitudinal / lateral force of each tyre,
* ``axle_slip_angle(u, fz_a, fz_b)`` - slip angle of an axle at lateral utilisation ``u``,
* ``axle_lateral_force(alpha, caps, fzs)`` / ``axle_peak_slip(fz_a, fz_b)`` - transient lateral DOF,
* ``combined_exponent`` - exponent ``n`` of the friction "ellipse" ``|ux|^n + |uy|^n <= 1``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Tire:
    radius: float  # m
    mu_x: float  # longitudinal friction coefficient at the reference load
    mu_y: float  # lateral friction coefficient at the reference load
    fz_ref: float  # N
    load_sensitivity: float  # relative change of mu per +100 % load (negative = degressive)
    min_mu_factor: float
    vertical_rate: float  # N/m
    peak_slip: float  # rad, slip angle of peak lateral force
    camber_optimum: float  # rad, best camber (wheel frame) of a wheel carrying lateral load
    camber_sensitivity: float  # 1/rad^2, relative grip loss per rad^2 away from the optimum
    rolling_resistance: float
    combined_exponent: float = 2.0  # friction ellipse exponent (2 = ellipse)

    model = "simple"

    @classmethod
    def from_config(cls, t: dict) -> "Tire":
        return cls(
            radius=float(t["radius_mm"]) / 1000.0,
            mu_x=float(t["mu_x"]),
            mu_y=float(t["mu_y"]),
            fz_ref=float(t["ref_load_N"]),
            load_sensitivity=float(t["load_sensitivity"]),
            min_mu_factor=float(t["min_mu_factor"]),
            vertical_rate=float(t["vertical_rate_N_per_mm"]) * 1000.0,
            peak_slip=math.radians(float(t["peak_slip_deg"])),
            camber_optimum=math.radians(float(t["camber_optimum_deg"])),
            camber_sensitivity=float(t["camber_sensitivity_per_deg2"]) * (180.0 / math.pi) ** 2,
            rolling_resistance=float(t["rolling_resistance"]),
            combined_exponent=_exponent(t.get("combined_exponent"), 2.0),
        )

    @property
    def uses_camber(self) -> bool:
        return self.camber_sensitivity > 0.0

    def load_factor(self, fz):
        """mu(Fz)/mu0 = 1 + k (Fz - Fz0)/Fz0, clamped from below."""
        f = 1.0 + self.load_sensitivity * (np.asarray(fz) - self.fz_ref) / self.fz_ref
        return np.maximum(f, self.min_mu_factor)

    def camber_factors(self, inclination, camber):
        """Grip factors (longitudinal, lateral) from camber.

        ``inclination`` is the wheel lean towards the turn centre (positive helps
        lateral grip); the optimum is ``-camber_optimum`` (e.g. -1 deg camber on the
        outer wheel = 1 deg inclination towards the turn centre). Longitudinal grip
        is best upright.
        """
        c = self.camber_sensitivity
        fy = 1.0 - c * (np.asarray(inclination) + self.camber_optimum) ** 2
        fx = 1.0 - c * np.asarray(camber) ** 2
        return np.clip(fx, 0.5, 1.0), np.clip(fy, 0.5, 1.0)

    def capacity(self, fz, inclination=None, camber=None):
        """Peak longitudinal and lateral force of each tyre (friction-ellipse semi-axes)."""
        fz = np.asarray(fz)
        base = self.load_factor(fz) * fz
        fx = self.mu_x * base
        fy = self.mu_y * base
        if inclination is not None and camber is not None and self.camber_sensitivity > 0.0:
            cx, cy = self.camber_factors(inclination, camber)
            fx = fx * cx
            fy = fy * cy
        return fx, fy

    def slip_angle(self, utilization):
        """Slip angle needed for a lateral utilization ``u`` = Fy/Fy_max.

        Normalised curve Fy/Fy_max = 2x/(1+x^2), x = alpha/alpha_peak (stable branch).
        """
        u = np.clip(np.asarray(utilization, dtype=float), 0.0, 0.999999)
        with np.errstate(divide="ignore", invalid="ignore"):
            x = np.where(u > 1e-9, (1.0 - np.sqrt(1.0 - u * u)) / np.where(u > 1e-9, u, 1.0), 0.5 * u)
        return self.peak_slip * x

    # ---- axle-level helpers (the Magic Formula tyre makes them load dependent) ----------------
    def axle_slip_angle(self, utilization, fz_a=None, fz_b=None):
        """Common slip angle of an axle's two tyres at lateral utilisation ``u``."""
        return self.slip_angle(utilization)

    def axle_peak_slip(self, fz_a=None, fz_b=None) -> float:
        return self.peak_slip

    def axle_lateral_force(self, alpha: float, caps, fzs=None) -> float:
        """Lateral force of an axle (two tyres with peak forces ``caps``) at slip angle ``alpha``."""
        return float(caps[0] + caps[1]) * lat_shape(alpha / self.peak_slip)

    def describe(self) -> str:
        return (f"simple tyre: mu_x {self.mu_x:g}, mu_y {self.mu_y:g} at {self.fz_ref:g} N, load sensitivity "
                f"{self.load_sensitivity:+g}/100 %, peak slip {math.degrees(self.peak_slip):.1f} deg")


def lat_shape(x: float) -> float:
    """Normalised lateral force vs slip ratio x = alpha / alpha_peak (odd function).

    ``2x / (1 + x^2)`` up to the peak, then a gentle fall to 85 % of the peak when sliding.
    """
    ax = abs(x)
    if ax <= 1.0:
        return 2.0 * x / (1.0 + x * x)
    return math.copysign(1.0 - 0.15 * (1.0 - 1.0 / ax), x)


def _exponent(value, default: float) -> float:
    if value is None:
        return default
    n = float(value)
    if not 1.0 <= n <= 10.0:
        from .config import ConfigError

        raise ConfigError(path="tires.combined_exponent", value=value, condition="1 <= 값 <= 10 (2 = 마찰 타원)",
                          hint="종·횡 동시 사용 한계 |ux|^n + |uy|^n <= 1 의 지수입니다. 모르면 비워 두세요 (2)")
    return n


def tire_from_config(t: dict, base_dir: str | Path = "."):
    """Simple tyre, or a Magic Formula tyre when ``tires.mf_file`` / ``tires.mf`` is given."""
    if t.get("mf_file") or t.get("mf"):
        from .tire_mf import MagicFormulaTire

        return MagicFormulaTire.from_config(t, base_dir)
    return Tire.from_config(t)
