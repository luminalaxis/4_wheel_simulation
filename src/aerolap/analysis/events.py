"""FSAE dynamic events: acceleration, skidpad, autocross, endurance."""

from __future__ import annotations

from dataclasses import dataclass

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


def run_events(vehicle: VehicleModel, autocross: Track | None = None, endurance: Track | None = None,
               endurance_distance_km: float = 22.0, opts: SolverOptions | None = None) -> list[EventResult]:
    solver = QSSSolver(opts)
    out = []
    acc = solver.solve(vehicle, Track.straight(ACCEL_LENGTH, name="acceleration 75 m"), start="standing")
    out.append(EventResult("acceleration", acc.lap_time, acc.energy_J / 3.6e6, acc, "75 m, standing start"))
    skid = solver.solve(vehicle, Track.circle(SKIDPAD_RADIUS, name="skidpad"), start="flying")
    out.append(EventResult("skidpad", skid.lap_time, skid.energy_J / 3.6e6, skid,
                           f"one steady-state lap, R = {SKIDPAD_RADIUS} m"))
    if autocross is not None:
        r = solver.solve(vehicle, autocross, start="standing")
        out.append(EventResult("autocross", r.lap_time, r.energy_J / 3.6e6, r,
                               f"{autocross.name}, standing start"))
    if endurance is not None:
        r = solver.solve(vehicle, endurance, start="flying")
        laps = max(1, round(endurance_distance_km * 1000.0 / endurance.length))
        out.append(EventResult("endurance", r.lap_time * laps, r.energy_J * laps / 3.6e6, r,
                               f"{laps} flying laps x {r.lap_time:.3f} s ({endurance.name})"))
    return out


def events_table(events: list[EventResult]) -> str:
    lines = [f"{'event':13s} {'time [s]':>10s} {'energy [kWh]':>13s}  note", "-" * 80]
    for e in events:
        lines.append(f"{e.name:13s} {e.time_s:10.3f} {e.energy_kWh:13.3f}  {e.note}")
    return "\n".join(lines)
