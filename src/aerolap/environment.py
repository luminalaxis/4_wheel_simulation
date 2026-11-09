"""Ambient air properties."""

from __future__ import annotations

import math

R_DRY = 287.058  # J/(kg K)
R_VAPOUR = 461.495  # J/(kg K)
P0 = 101.325  # kPa, ISA sea level


def isa_pressure_kpa(altitude_m: float) -> float:
    """Static pressure of the International Standard Atmosphere (troposphere)."""
    return P0 * (1.0 - 2.25577e-5 * altitude_m) ** 5.25588


def saturation_vapour_pressure_kpa(temperature_c: float) -> float:
    """Buck (1996) saturation vapour pressure over water."""
    t = temperature_c
    return 0.61121 * math.exp((18.678 - t / 234.5) * (t / (257.14 + t)))


def air_density(temperature_c: float = 15.0, pressure_kpa: float | None = None,
                altitude_m: float = 0.0, relative_humidity: float = 0.0) -> float:
    """Moist-air density in kg/m^3.

    ``pressure_kpa`` is the station pressure; when omitted it is derived from
    ``altitude_m`` with the ISA model.
    """
    p = (pressure_kpa if pressure_kpa is not None else isa_pressure_kpa(altitude_m)) * 1000.0
    t = temperature_c + 273.15
    rh = min(max(relative_humidity, 0.0), 1.0)
    p_v = rh * saturation_vapour_pressure_kpa(temperature_c) * 1000.0
    p_d = p - p_v
    return p_d / (R_DRY * t) + p_v / (R_VAPOUR * t)


def density_from_config(env: dict) -> float:
    return air_density(
        temperature_c=float(env.get("temperature_C", 15.0)),
        pressure_kpa=env.get("pressure_kPa"),
        altitude_m=float(env.get("altitude_m", 0.0)),
        relative_humidity=float(env.get("relative_humidity", 0.0)),
    )
