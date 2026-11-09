"""Tyre grip model: load sensitivity, camber sensitivity, friction ellipse, slip curve."""

from __future__ import annotations

import math
from dataclasses import dataclass

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
        )

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
