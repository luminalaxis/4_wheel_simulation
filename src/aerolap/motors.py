"""Electric motor models built from datasheet values.

``EMRAX 228`` (datasheet v1.7, EMRAX d.o.o.):

* peak torque 220 Nm, limiting speed 6500 rpm, peak power 124 kW (S2 30 s)
* winding variants HV / MV / LV with their KV at peak load, KT, peak current, phase resistance
* continuous rating by cooling: AC 83 Nm / 48 kW, LC 118 Nm / 68 kW, CC 130 Nm / 75 kW
* rotor inertia 0.0398 kg m^2 (LC), 0.0393 (AC / CC)

Torque-speed envelope::

    n_base = KV_peak_load * V_dc                        (back-EMF reaches the DC bus voltage)
    T(n)   = min(T_peak, KT * I_controller)             for n <= n_base
           = T(n_base) * n_base / n                     for n_base < n <= n_max (field weakening)
    P      <= P_peak

Check against the datasheet: MV at 630 V gives n_base = 5468 rpm -> 126 kW (datasheet: 124 kW at
5500 rpm); HV at 830 V gives 4690 rpm -> 108 kW (datasheet: 104 kW at 4500 rpm).

Efficiency: ``eta = P / (P + P_cu + P_0)`` with the copper loss ``3 (T / KT)^2 R_phase`` from the
datasheet values and a speed-dependent iron / mechanical / fixed loss
``P_0 = c0 + c1 n + c2 n^2`` fitted to the datasheet efficiency map (96 % peak around
3000 rpm / 110 Nm, 94-96 % island over 2000-4500 rpm and 60-170 Nm, 86-90 % at
1000 rpm / 20 Nm, below 86 % at 500 rpm). Approximate - replace with a measured map if
you have one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

MOTOR_DATABASE: dict[str, dict] = {
    "emrax_228": {
        "name": "EMRAX 228",
        "source": "EMRAX 228 datasheet v1.7",
        "peak_torque_Nm": 220.0,
        "max_rpm": 6500.0,
        "peak_power_kW": 124.0,
        "variants": {
            # KV at peak load [rpm/V], KT [Nm/A_rms], peak current [A_rms], phase resistance [ohm],
            # DC voltage needed for peak power [V]
            "HV": {"kv_peak": 5.65, "kt": 0.98, "i_peak": 235.0, "r_phase": 16.09e-3, "v_peak_power": 830.0},
            "MV": {"kv_peak": 8.68, "kt": 0.64, "i_peak": 360.0, "r_phase": 6.95e-3, "v_peak_power": 630.0},
            "LV": {"kv_peak": 21.91, "kt": 0.24, "i_peak": 920.0, "r_phase": 1.36e-3, "v_peak_power": 250.0},
        },
        "continuous": {"AC": (83.0, 48.0), "LC": (118.0, 68.0), "CC": (130.0, 75.0)},  # Nm, kW
        "rotor_inertia_kgm2": {"AC": 0.0393, "LC": 0.0398, "CC": 0.0393},
        "mass_kg": {"AC": 12.9, "LC": 13.5, "CC": 13.2},
        "speed_loss_W": (120.0, 0.04, 6.0e-5),  # c0 [W], c1 [W/rpm], c2 [W/rpm^2], fitted to the map
    },
}


@dataclass(frozen=True)
class Motor:
    name: str
    variant: str
    cooling: str
    peak_torque: float  # Nm (also limited by KT * controller current)
    max_rpm: float
    peak_power: float  # W
    kt: float
    kv_peak: float
    r_phase: float
    v_dc: float
    i_max: float
    cont_torque: float  # Nm
    cont_power: float  # W
    rotor_inertia: float  # kg m^2
    mass: float
    speed_loss: tuple[float, float, float]  # c0, c1, c2

    @classmethod
    def from_config(cls, ev: dict) -> "Motor":
        key = str(ev["motor"]).lower()
        if key not in MOTOR_DATABASE:
            from .config import ConfigError

            raise ConfigError(path="powertrain.ev.motor", value=ev["motor"],
                              condition=f"모터 데이터베이스에 있는 이름: {', '.join(MOTOR_DATABASE)}",
                              hint="다른 모터는 motor 대신 motor_torque_curve + max_rpm으로 넣으세요")
        db = MOTOR_DATABASE[key]
        variant = str(ev.get("motor_variant", "MV")).upper()
        cooling = str(ev.get("motor_cooling", "LC")).upper()
        if variant not in db["variants"]:
            from .config import ConfigError

            raise ConfigError(path="powertrain.ev.motor_variant", value=variant,
                              condition=f"{', '.join(db['variants'])} 중 하나")
        if cooling not in db["continuous"]:
            from .config import ConfigError

            raise ConfigError(path="powertrain.ev.motor_cooling", value=cooling,
                              condition=f"{', '.join(db['continuous'])} 중 하나")
        var = db["variants"][variant]
        v_dc = float(ev.get("dc_voltage_V") or var["v_peak_power"])
        i_ctrl = ev.get("controller_current_A")
        i_max = min(float(i_ctrl), var["i_peak"]) if i_ctrl is not None else var["i_peak"]
        inertia = ev.get("rotor_inertia_kgm2")
        cont_t, cont_p = db["continuous"][cooling]
        return cls(
            name=f"{db['name']} {variant} {cooling}", variant=variant, cooling=cooling,
            peak_torque=db["peak_torque_Nm"], max_rpm=float(ev.get("max_rpm") or db["max_rpm"]),
            peak_power=db["peak_power_kW"] * 1000.0, kt=var["kt"], kv_peak=var["kv_peak"],
            r_phase=var["r_phase"], v_dc=v_dc, i_max=i_max, cont_torque=cont_t, cont_power=cont_p * 1000.0,
            rotor_inertia=float(inertia) if inertia is not None else db["rotor_inertia_kgm2"][cooling],
            mass=db["mass_kg"][cooling], speed_loss=tuple(db["speed_loss_W"]),
        )

    @property
    def base_rpm(self) -> float:
        """Speed where the back-EMF at peak load reaches the DC bus voltage."""
        return min(self.kv_peak * self.v_dc, self.max_rpm)

    @property
    def torque_limit(self) -> float:
        return min(self.peak_torque, self.kt * self.i_max)

    def max_torque(self, rpm):
        n = np.maximum(np.asarray(rpm, dtype=float), 0.0)
        t0 = self.torque_limit
        nb = self.base_rpm
        t = np.where(n <= nb, t0, t0 * nb / np.maximum(n, 1e-9))
        omega = n * 2.0 * math.pi / 60.0
        t = np.minimum(t, self.peak_power / np.maximum(omega, 1e-9))
        return np.where(n <= self.max_rpm, t, 0.0)

    def losses(self, rpm, torque):
        """Copper + speed (iron, friction, fixed) losses at (rpm, torque), W."""
        n = np.maximum(np.asarray(rpm, dtype=float), 0.0)
        t = np.abs(np.asarray(torque, dtype=float))
        c0, c1, c2 = self.speed_loss
        return 3.0 * (t / self.kt) ** 2 * self.r_phase + c0 + c1 * n + c2 * n * n

    def efficiency(self, rpm, torque):
        """Motor efficiency at (rpm, torque >= 0); clipped to [0.5, 0.97]."""
        n = np.maximum(np.asarray(rpm, dtype=float), 0.0)
        t = np.abs(np.asarray(torque, dtype=float))
        p = t * n * 2.0 * math.pi / 60.0
        c0, c1, c2 = self.speed_loss
        loss = 3.0 * (t / self.kt) ** 2 * self.r_phase + c0 + c1 * n + c2 * n * n
        with np.errstate(divide="ignore", invalid="ignore"):
            eta = np.where(p > 1.0, p / (p + loss), 0.5)
        return np.clip(eta, 0.5, 0.97)
