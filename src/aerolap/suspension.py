"""Linear suspension description of one axle (stiffnesses referred to the wheel centre)."""

from __future__ import annotations

import math
from dataclasses import dataclass


def series(k1: float, k2: float) -> float:
    """Two springs in series (inf-safe)."""
    if math.isinf(k1):
        return k2
    if math.isinf(k2):
        return k1
    if k1 <= 0.0 or k2 <= 0.0:
        return 0.0
    return k1 * k2 / (k1 + k2)


@dataclass(frozen=True)
class Axle:
    track: float  # m
    wheel_rate: float  # N/m per corner (spring * MR^2)
    heave_rate: float  # N/m, third element at the wheel (heave only)
    arb_rate: float  # N m/rad, anti-roll bar at the wheels
    roll_center: float  # m
    static_ride_height: float  # m
    static_camber: float  # rad
    camber_gain: float  # rad per m of bump
    tire_rate: float  # N/m per tyre
    anti_brake: float  # anti-dive (front) / anti-lift (rear), fraction
    anti_drive: float  # anti-squat (rear), fraction
    unsprung_mass: float  # kg, both corners
    motion_ratio: float = 1.0
    damper_rate: float | None = None  # N s/m at the damper (None -> damping ratio)
    damping_ratio: float = 0.65

    @classmethod
    def from_config(cls, a: dict, track_mm: float, tire_rate: float, unsprung_kg: float,
                    is_front: bool) -> "Axle":
        mr = float(a["motion_ratio"])
        return cls(
            track=track_mm / 1000.0,
            wheel_rate=float(a["spring_rate_N_per_mm"]) * 1000.0 * mr * mr,
            heave_rate=float(a["heave_spring_N_per_mm"]) * 1000.0,
            arb_rate=float(a["arb_rate_Nm_per_deg"]) * 180.0 / math.pi,
            roll_center=float(a["roll_center_mm"]) / 1000.0,
            static_ride_height=float(a["static_ride_height_mm"]) / 1000.0,
            static_camber=math.radians(float(a["static_camber_deg"])),
            camber_gain=math.radians(float(a["camber_gain_deg_per_mm"])) * 1000.0,
            tire_rate=tire_rate,
            anti_brake=float(a["anti_dive"] if is_front else a["anti_lift"]),
            anti_drive=0.0 if is_front else float(a["anti_squat"]),
            unsprung_mass=unsprung_kg,
            motion_ratio=mr,
            damper_rate=None if a.get("damper_rate_Ns_per_mm") is None
            else float(a["damper_rate_Ns_per_mm"]) * 1000.0,
            damping_ratio=float(a.get("damping_ratio", 0.65)),
        )

    @property
    def heave_stiffness(self) -> float:
        """Axle heave stiffness of the springs (both corners + third element), N/m."""
        return 2.0 * self.wheel_rate + self.heave_rate

    @property
    def roll_stiffness_springs(self) -> float:
        """Suspension roll stiffness (springs + ARB), N m/rad."""
        return 0.5 * self.wheel_rate * self.track ** 2 + self.arb_rate

    @property
    def roll_stiffness(self) -> float:
        """Roll stiffness including tyre compliance in series, N m/rad."""
        return series(self.roll_stiffness_springs, 0.5 * self.tire_rate * self.track ** 2)

    def ride_rate(self) -> float:
        """Corner ride rate in heave (spring + half third element in series with the tyre)."""
        return series(self.wheel_rate + 0.5 * self.heave_rate, self.tire_rate)

    def ride_frequency(self, corner_sprung_mass: float) -> float:
        return math.sqrt(self.ride_rate() / corner_sprung_mass) / (2.0 * math.pi)

    def wheel_damping(self, corner_sprung_mass: float) -> float:
        """Damping per corner at the wheel, N s/m."""
        if self.damper_rate is not None:
            return self.damper_rate * self.motion_ratio ** 2
        k = self.wheel_rate + 0.5 * self.heave_rate
        return 2.0 * self.damping_ratio * math.sqrt(k * corner_sprung_mass)
