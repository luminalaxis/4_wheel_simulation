"""Model feature switches used to build the OptimumLap -> full-model "effect ladder"."""

from __future__ import annotations

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class ModelFeatures:
    """Which physical effects are active in :class:`~aerolap.vehicle.VehicleModel`.

    All ``False`` reproduces an OptimumLap-style point mass: tyre loads are the
    static axle loads plus downforce split in the static weight ratio, aero
    coefficients are frozen at the static ride heights.
    """

    load_transfer: bool = True  # longitudinal + lateral load transfer (4 individual tyre loads)
    aero_balance: bool = True  # downforce split by the aero map / CoP (else static weight ratio)
    drag_height: bool = True  # drag acting above the ground -> rearward load transfer
    heave_pitch_aero: bool = True  # ride heights from heave & pitch feed the aero map
    roll_yaw_aero: bool = True  # body roll & sideslip (yaw) feed the aero map
    camber: bool = True  # camber change with heave/roll affects tyre grip
    active_aero: bool = True  # use the configured active-aero / DRS strategy
    label: str = "full"

    def with_label(self, label: str) -> "ModelFeatures":
        return replace(self, label=label)


_ALL_OFF = ModelFeatures(
    load_transfer=False, aero_balance=False, drag_height=False, heave_pitch_aero=False,
    roll_yaw_aero=False, camber=False, active_aero=False, label="L0",
)

LEVELS: dict[str, tuple[str, ModelFeatures]] = {
    "L0": ("OptimumLap 동등 (점질량, 정적 차고 공력 상수)", _ALL_OFF),
    "L1": ("+ 4륜 하중이동 / 타이어 하중 민감도", replace(_ALL_OFF, load_transfer=True, label="L1")),
    "L2": ("+ 공력 밸런스(CoP) / 항력 작용 높이",
           replace(_ALL_OFF, load_transfer=True, aero_balance=True, drag_height=True, label="L2")),
    "L3": ("+ 히브·피치 차고 변화 -> aero map",
           replace(_ALL_OFF, load_transfer=True, aero_balance=True, drag_height=True,
                   heave_pitch_aero=True, label="L3")),
    "L4": ("+ 롤·요(코너링 자세) -> aero map, 캠버",
           replace(_ALL_OFF, load_transfer=True, aero_balance=True, drag_height=True,
                   heave_pitch_aero=True, roll_yaw_aero=True, camber=True, label="L4")),
    "L5": ("+ 액티브 에어로 / DRS", ModelFeatures(label="L5")),
}


def features_for_level(level: str) -> ModelFeatures:
    key = level.upper()
    if key in ("FULL", "L5"):
        return LEVELS["L5"][1]
    if key not in LEVELS:
        raise ValueError(f"unknown model level {level!r}; use one of {', '.join(LEVELS)} or 'full'")
    return LEVELS[key][1]
