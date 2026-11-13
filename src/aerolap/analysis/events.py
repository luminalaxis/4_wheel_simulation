"""FSAE dynamic events: acceleration, skidpad, autocross, endurance."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..solvers.base import LapResult
from ..solvers.ggv import SolverOptions
from ..solvers.qss import QSSSolver
from ..track import Track
from ..vehicle import VehicleModel

SKIDPAD_RADIUS = 9.125  # m, centre line of the 15.25 m inner circle + half the 3 m lane
ACCEL_LENGTH = 75.0  # m


@dataclass
class EventResult:
    name: str
    time_s: float
    energy_kWh: float
    result: LapResult
    note: str = ""
    energy: object = None  # EnduranceEnergy for the endurance event


def run_events(vehicle: VehicleModel, autocross: Track | None = None, endurance: Track | None = None,
               endurance_distance_km: float = 22.0, opts: SolverOptions | None = None) -> list[EventResult]:
    solver = QSSSolver(opts)
    out = []
    acc = solver.solve(vehicle, Track.straight(ACCEL_LENGTH, name="acceleration 75 m"), start="standing")
    acc_note = "75 m, standing start"
    pt = vehicle.powertrain
    if pt.pack is not None:
        acc_note += f"; pack {pt.pack.describe()}, bus {pt.pack_voltage:.0f} V at {pt.power_limit / 1000:.1f} kW"
    out.append(EventResult("acceleration", acc.lap_time, acc.energy_J / 3.6e6, acc, acc_note))
    skid = solver.solve(vehicle, Track.circle(SKIDPAD_RADIUS, name="skidpad"), start="flying")
    out.append(EventResult("skidpad", skid.lap_time, skid.energy_J / 3.6e6, skid,
                           f"one steady-state lap, R = {SKIDPAD_RADIUS} m"))
    if autocross is not None:
        r = solver.solve(vehicle, autocross, start="standing")
        out.append(EventResult("autocross", r.lap_time, r.energy_J / 3.6e6, r,
                               f"{autocross.name}, standing start"))
    if endurance is not None:
        from .energy import endurance_energy

        en = endurance_energy(vehicle.cfg, endurance, endurance_distance_km, vehicle.features, opts)
        r = en.result
        note = f"{en.laps} flying laps x {en.lap_time_s:.3f} s ({endurance.name})"
        if en.usable_kWh is not None:
            if en.power_cap_kW is None:
                note += f"; battery {en.usable_kWh:.2f} kWh usable, margin {en.margin_kWh:+.2f} kWh"
            else:
                note += (f"; battery {en.usable_kWh:.2f} kWh usable -> power cap {en.power_cap_kW:.1f} kW "
                         f"(flat out would need {en.unlimited_required_kWh:.2f} kWh)")
        if en.stages:
            st = en.stages
            note += ("\n" + " " * 41 + f"pack: SOC {st[0].soc_start:.2f} -> {st[-1].soc_end:.2f}, bus "
                     f"{min(x.v_terminal_min for x in st):.0f} V min, {max(x.i_max for x in st):.0f} A max, "
                     f"R I^2 {sum(x.loss_kWh for x in st):.2f} kWh, laps {st[0].lap_time_s:.3f} -> "
                     f"{st[-1].lap_time_s:.3f} s")
        thermal = motor_thermal_note(VehicleModel(vehicle.cfg, vehicle.features), r)
        if thermal:
            note += "\n" + " " * 41 + thermal
        if en.thermal is not None:
            for line in en.thermal.report_lines():
                note += "\n" + " " * 41 + line.strip()
        ev = EventResult("endurance", en.total_time_s, en.required_kWh, r, note)
        ev.energy = en
        out.append(ev)
    return out


def motor_thermal_note(vehicle: VehicleModel, res: LapResult) -> str:
    """RMS motor torque and mean motor power over a lap vs the continuous (S1) rating."""
    m = vehicle.powertrain.motor
    ch = res.channels
    if m is None or "motor_torque_Nm" not in ch:
        return ""
    dt = np.diff(res.t)
    tq = 0.5 * (ch["motor_torque_Nm"][1:] + ch["motor_torque_Nm"][:-1])
    p_mech = ch["motor_torque_Nm"] * ch["motor_rpm"] * 2.0 * np.pi / 60.0
    pm = 0.5 * (p_mech[1:] + p_mech[:-1])
    t_rms = float(np.sqrt((tq ** 2 * dt).sum() / dt.sum()))
    p_mean = float((pm * dt).sum() / dt.sum()) / 1000.0
    flag = "OK" if t_rms <= m.cont_torque and p_mean <= m.cont_power / 1000.0 else "ABOVE continuous rating"
    return (f"motor {m.name}: RMS torque {t_rms:.1f} Nm (S1 {m.cont_torque:.0f} Nm), mean power "
            f"{p_mean:.1f} kW (S1 {m.cont_power / 1000.0:.0f} kW) -> {flag}")


def events_table(events: list[EventResult]) -> str:
    lines = [f"{'event':13s} {'time [s]':>10s} {'energy [kWh]':>13s}  note", "-" * 80]
    for e in events:
        lines.append(f"{e.name:13s} {e.time_s:10.3f} {e.energy_kWh:13.3f}  {e.note}")
    for e in events:
        if e.energy is not None:
            lines += ["", e.energy.report()]
    return "\n".join(lines)
