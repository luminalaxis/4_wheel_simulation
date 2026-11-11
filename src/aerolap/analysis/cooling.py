"""Radiator heat balance of a lap: heat the sidepods reject vs heat the powertrain makes.

The powertrain losses of the lap (battery energy at the terminals - work at the wheels - pack
R I^2, from :func:`aerolap.analysis.energy.energy_breakdown`), times ``heat_share`` (share that goes
into the coolant: motor + inverter; default 0.85), averaged over the lap time, must be carried away
by the radiators. Their heat rejection is proportional to the speed (see
:class:`aerolap.aero.devices.Sidepod`), so its lap average is the value at the lap-average speed.
``margin = rejected - produced`` (kW); negative = the coolant heats up over the run.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class CoolingBalance:
    produced_kW: float
    rejected_kW: float
    v_avg_kmh: float
    pods: list[tuple[str, float]]  # (name, rejected kW)

    @property
    def margin_kW(self) -> float:
        return self.rejected_kW - self.produced_kW

    def report(self) -> str:
        pods = ", ".join(f"{n} {q:.2f} kW" for n, q in self.pods)
        return (f"cooling: powertrain heat {self.produced_kW:.2f} kW (lap average) vs radiators {self.rejected_kW:.2f} kW "
                f"at {self.v_avg_kmh:.1f} km/h ({pods}) -> margin {self.margin_kW:+.2f} kW"
                + ("  NOT ENOUGH COOLING" if self.margin_kW < 0 else ""))


def sidepods(vehicle) -> list:
    return [c.dev for c in getattr(vehicle.aero, "components", []) if hasattr(getattr(c, "dev", None), "cooling_cda")]


def cooling_balance(vehicle, res, heat_share: float | None = None) -> CoolingBalance | None:
    """None when the car has no ``type: sidepod`` device."""
    from .energy import KWH, energy_breakdown

    pods = sidepods(vehicle)
    if not pods:
        return None
    b = energy_breakdown(vehicle, res)
    share = heat_share if heat_share is not None else float(
        next((c.get("heat_share") for c in vehicle.cfg["aero"].get("components", {}).values()
              if isinstance(c, dict) and c.get("heat_share") is not None), 0.85))
    produced = max(b["powertrain_losses_kWh"], 0.0) * KWH / max(res.lap_time, 1e-9) / 1000.0 * share
    v_avg = float(res.s[-1] / max(res.lap_time, 1e-9))
    t_air = float(vehicle.cfg.get("environment", {}).get("temperature_C", 20.0))
    q = [(p.name, float(np.asarray(p.heat_rejection_kW(v_avg, vehicle.rho, t_air)))) for p in pods]
    return CoolingBalance(produced, sum(x for _, x in q), v_avg * 3.6, q)
