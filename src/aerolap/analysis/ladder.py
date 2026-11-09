"""Effect ladder: OptimumLap-equivalent point mass -> full coupled model, one effect at a time."""

from __future__ import annotations

from dataclasses import dataclass

from ..features import LEVELS
from ..solvers.base import LapResult
from ..solvers.ggv import SolverOptions
from ..solvers.qss import QSSSolver
from ..vehicle import VehicleModel


@dataclass
class LadderStep:
    level: str
    description: str
    result: LapResult


def effect_ladder(cfg: dict, track, start: str = "flying", opts: SolverOptions | None = None,
                  levels: list[str] | None = None) -> list[LadderStep]:
    steps = []
    for level in levels or list(LEVELS):
        desc, feats = LEVELS[level]
        car = VehicleModel(cfg, feats)
        res = QSSSolver(opts).solve(car, track, start=start)
        steps.append(LadderStep(level, desc, res))
    return steps


def ladder_table(steps: list[LadderStep]) -> str:
    base = steps[0].result.lap_time
    lines = [f"{'level':5s} {'lap [s]':>9s} {'step [s]':>9s} {'vs L0 [s]':>10s} "
             f"{'vmax[km/h]':>10s} {'E [Wh]':>8s}  description",
             "-" * 96]
    prev = base
    for st in steps:
        r = st.result
        sm = r.summary()
        lines.append(f"{st.level:5s} {r.lap_time:9.3f} {r.lap_time - prev:+9.3f} {r.lap_time - base:+10.3f} "
                     f"{sm['v_max_kmh']:10.1f} {sm['drive_energy_kWh'] * 1000:8.1f}  {st.description}")
        prev = r.lap_time
    return "\n".join(lines)
