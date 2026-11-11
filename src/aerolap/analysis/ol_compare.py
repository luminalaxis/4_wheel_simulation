"""Compare the OptimumLap-equivalent model (L0) with an OptimumLap result (.OLRes).

The result file carries the track and vehicle OptimumLap used, so the same car is rebuilt with
:func:`aerolap.importers.vehicle_cfg_from_optimumlap`, run on the same track and compared lap time,
speed trace and energy. What OptimumLap does was read from its own channels (checked on a real
result, FSK 2026 track, 8510 points):

* normal load ``Fz = m g + downforce`` (one point mass), ``downforce = 0.5 rho Cl A v^2``
* corner limit ``m v^2 / R = mu_lat Fz``
* braking ``m a = -(mu_long Fz + drag + C_rr Fz)``; in corners the tyre forces (brake force without
  drag / rolling resistance, and ``m a_y``) stay on the friction circle ``mu Fz``
* traction ``m a + drag + C_rr Fz <= weightdrivenwheel * mu_long * Fz`` (downforce shared like the
  weight) and the torque curve x final drive x efficiency / tyre radius
* a corner followed by a corner in the same direction changes its radius linearly to the next one
  (:meth:`aerolap.importers.OlTrack.segments`)
* ``Energy Spent`` = integral of ``max(m a + drag, 0) ds / efficiency``: rolling resistance is left
  out (aerolap's lap energy includes it, so it is a few per cent higher)
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..config import vehicle_config_from_dict
from ..features import features_for_level
from ..importers import OlResult, vehicle_cfg_from_optimumlap
from ..solvers.base import LapResult
from ..solvers.ggv import SolverOptions
from ..solvers.qss import QSSSolver
from ..track import Track
from ..vehicle import VehicleModel

G = 9.80665


@dataclass
class OlComparison:
    ol: OlResult
    lap: LapResult
    cfg: dict
    rows: list[tuple[str, float, float, str]] = field(default_factory=list)  # name, OptimumLap, aerolap, unit
    s: np.ndarray | None = None  # OptimumLap distance [m]
    v_ol: np.ndarray | None = None
    v_ours: np.ndarray | None = None  # aerolap speed at the OptimumLap points
    dt_cum: np.ndarray | None = None  # aerolap - OptimumLap elapsed time [s]
    phases: list[tuple[str, float, float, float]] = field(default_factory=list)  # name, distance, t_OL, t_ours

    @property
    def lap_diff_pct(self) -> float:
        return (self.lap.lap_time / self.ol.lap_time - 1.0) * 100.0

    def report(self) -> str:
        v = self.ol.vehicle
        lines = [f"OptimumLap result: {self.ol.name}",
                 f"track:   {self.ol.track.name} ({len(self.ol.track.lengths)} segments, "
                 f"{self.ol.track.length:.1f} m, {'closed' if self.ol.track.closed else 'open'})",
                 f"vehicle: {v.name}  m {v.get('mass', 0):g} kg, mu {v.get('longfriction', 0):g}/"
                 f"{v.get('latfriction', 0):g}, ClA {v.get('liftcoefficient', 0) * v.get('frontarea', 0):.3f}, "
                 f"CdA {v.get('dragcoefficient', 0) * v.get('frontarea', 0):.3f}, final drive "
                 f"{v.get('finaldriveratio', 0):g}, efficiency {v.get('driveefficiency', 0):g}, "
                 f"driven wheels {v.get('weightdrivenwheel', 0):.0%} of the load",
                 f"aerolap: model {self.lap.level}, {self.lap.solver}", "",
                 f"{'':34s} {'OptimumLap':>11s} {'aerolap':>11s} {'diff':>9s}"]
        for name, a, b, unit in self.rows:
            diff = f"{(b / a - 1) * 100:+8.2f}%" if a else ""
            lines.append(f"{name + ' [' + unit + ']':34s} {a:11.3f} {b:11.3f} {diff}")
        dv = (self.v_ours - self.v_ol) * 3.6
        lines += ["", f"speed difference aerolap - OptimumLap: mean {dv.mean():+.2f} km/h, "
                      f"rms {np.sqrt(np.mean(dv ** 2)):.2f} km/h, 5..95 % {np.percentile(dv, 5):+.2f} .. "
                      f"{np.percentile(dv, 95):+.2f} km/h"]
        if self.phases:
            lines += ["", "time by OptimumLap phase (same distance in both):",
                      f"  {'phase':30s} {'dist [m]':>9s} {'OL [s]':>8s} {'aerolap':>8s} {'diff':>8s}"]
            for name, dist, a, b in self.phases:
                lines.append(f"  {name:30s} {dist:9.1f} {a:8.3f} {b:8.3f} {b - a:+8.3f}")
        lines += ["", "Energy: OptimumLap leaves rolling resistance out of 'Energy Spent'; the row "
                      "'energy, OptimumLap definition' applies the same definition to the aerolap lap."]
        return "\n".join(lines)


# OptimumLap 'VehicleState' channel; meaning inferred from throttle / brake / lateral acceleration
_STATES = {1: "accelerating (corner exit)", 2: "braking in a corner", 3: "accelerating on a straight",
           4: "braking on a straight", 5: "corner limit (apex)"}


def compare_with_optimumlap(ol: OlResult, base_raw: dict, drive: str = "rwd", level: str = "L0",
                            opts: SolverOptions | None = None, transitions: bool = True) -> OlComparison:
    raw, _, _, _ = vehicle_cfg_from_optimumlap(ol.vehicle, base_raw, drive=drive)
    cfg = vehicle_config_from_dict(raw)
    car = VehicleModel(cfg, features_for_level(level))
    trk = Track.from_segments(ol.track.segments(transitions), name=ol.track.name, closed=ol.track.closed)
    ch = ol.channels
    v_ol = ch["speed"]
    start = "flying" if ol.track.closed else ("standing" if v_ol[0] < 1.0 else "flying")
    lap = QSSSolver(opts or SolverOptions(ds=0.1)).solve(car, trk, start=start)

    s = ch["elapsedDistance"]
    v_ours = np.interp(s, lap.s, lap.v)
    t_ol = ch["elapsedTime"]
    t_ours = np.interp(s, lap.s, lap.t)
    out = OlComparison(ol, lap, cfg, s=s, v_ol=v_ol, v_ours=v_ours, dt_cum=t_ours - t_ol)

    mass = float(raw["mass"]["total_kg"])
    eff = float(raw["powertrain"].get("efficiency", 1.0))
    # aerolap lap energy, total and with OptimumLap's definition (no rolling resistance)
    f_ol = mass * lap.ax + lap.channels["drag_N"]
    e_ol_def = float(np.sum(np.maximum(f_ol[:-1], 0.0) * np.diff(lap.s))) / eff
    ay_ol, ax_ol = ch.get("lateralAcceleration"), ch.get("longitudinalAcceleration")
    rows = [("lap time", ol.lap_time, lap.lap_time, "s"),
            ("lowest speed", v_ol.min() * 3.6, lap.v.min() * 3.6, "km/h"),
            ("highest speed", v_ol.max() * 3.6, lap.v.max() * 3.6, "km/h"),
            ("average speed", ol.track.length / ol.lap_time * 3.6, lap.s[-1] / lap.lap_time * 3.6, "km/h")]
    if ay_ol is not None:
        rows.append(("max lateral acceleration", np.abs(ay_ol).max() / G, np.abs(lap.ay).max() / G, "g"))
    if ax_ol is not None:
        rows += [("max longitudinal acceleration", ax_ol.max() / G, lap.ax.max() / G, "g"),
                 ("max deceleration", -ax_ol.min() / G, -lap.ax.min() / G, "g")]
    if "energySpent" in ol.kpis:
        rows += [("energy, OptimumLap definition", ol.kpis["energySpent"] / 1e3, e_ol_def / 1e3, "kJ"),
                 ("energy, aerolap (with rolling res.)", ol.kpis["energySpent"] / 1e3, lap.energy_J / 1e3, "kJ")]
    out.rows = rows

    state = ch.get("VehicleState")
    if state is not None and len(state) == len(s):
        ds = np.diff(np.concatenate([[0.0], s]))
        dt_ol = np.diff(np.concatenate([[0.0], t_ol]))
        dt_ours = np.diff(np.concatenate([[0.0], t_ours]))
        for k in sorted(set(int(x) for x in state)):
            m = state.astype(int) == k
            out.phases.append((f"{k}: {_STATES.get(k, '?')}", float(ds[m].sum()), float(dt_ol[m].sum()),
                               float(dt_ours[m].sum())))
    return out


def plot_ol_compare(cmp: OlComparison, path):
    from ..plotting import _save, plt

    fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axs[0].plot(cmp.s, cmp.v_ol * 3.6, color="tab:orange", lw=1.2, label=f"OptimumLap {cmp.ol.lap_time:.3f} s")
    axs[0].plot(cmp.s, cmp.v_ours * 3.6, color="k", lw=0.9,
                label=f"aerolap {cmp.lap.level} {cmp.lap.lap_time:.3f} s ({cmp.lap_diff_pct:+.2f} %)")
    axs[0].set_ylabel("speed [km/h]")
    axs[0].legend(loc="lower right")
    axs[1].plot(cmp.s, (cmp.v_ours - cmp.v_ol) * 3.6, color="tab:blue", lw=0.8)
    axs[1].axhline(0, color="0.5", lw=0.6)
    axs[1].set_ylabel("aerolap - OL [km/h]")
    axs[2].plot(cmp.s, cmp.dt_cum, color="tab:red")
    axs[2].axhline(0, color="0.5", lw=0.6)
    axs[2].set_ylabel("time delta [s]\n(+ = aerolap slower)")
    axs[2].set_xlabel("distance [m]")
    for a in axs:
        a.grid(alpha=0.3)
    fig.suptitle(f"OptimumLap vs aerolap {cmp.lap.level}: {cmp.ol.track.name}")
    return _save(fig, path)
