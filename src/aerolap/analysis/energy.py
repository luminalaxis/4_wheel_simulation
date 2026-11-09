"""Battery energy: where the energy of a lap goes, and an energy-limited endurance.

``energy_breakdown`` splits the work at the wheels into aerodynamic drag, rolling resistance,
braking (heat) and the change of kinetic energy, and the battery energy into that work plus the
motor / inverter / drivetrain losses (minus what regeneration returns).

``endurance_energy`` drives the endurance distance flat out; if that needs more than the usable
battery energy (``battery.capacity_kWh`` x ``usable_fraction``) it lowers the battery power cap
until the run fits - the usual way an electric car is managed in an endurance - and reports the
lap time at that cap. More drag (a bigger wing) therefore costs twice: more energy per lap and,
when the battery is the limit, a lower power cap.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field

import numpy as np

from ..config import with_overrides
from ..features import ModelFeatures
from ..solvers.base import LapResult
from ..solvers.ggv import SolverOptions
from ..solvers.qss import QSSSolver
from ..track import Track
from ..vehicle import VehicleModel

KWH = 3.6e6


def energy_breakdown(vehicle: VehicleModel, res: LapResult) -> dict[str, float]:
    """Energy terms of one lap in kWh (positive = consumed)."""
    ch = res.channels
    dt = np.diff(res.t)

    def integral(power_w):
        p = np.asarray(power_w, dtype=float)
        return float((0.5 * (p[1:] + p[:-1]) * dt).sum()) / KWH

    v = res.v
    fz = sum(ch[k] for k in ("fz_fl_N", "fz_fr_N", "fz_rl_N", "fz_rr_N"))
    drag = integral(ch["drag_N"] * v)
    rolling = integral(vehicle.tire.rolling_resistance * fz * v)
    braking = integral(ch["brake_force_N"] * v)
    wheel = integral(ch["drive_force_N"] * v)
    regen = integral(ch["regen_power_kW"] * 1000.0) if "regen_power_kW" in ch else 0.0
    battery = res.energy_J / KWH
    gross = battery + regen
    return {
        "battery_kWh": battery,
        "wheel_work_kWh": wheel,
        "aero_drag_kWh": drag,
        "rolling_kWh": rolling,
        "braking_heat_kWh": braking - regen,
        "regen_returned_kWh": regen,
        "kinetic_change_kWh": wheel - drag - rolling - braking,
        "powertrain_losses_kWh": gross - wheel,
        "aero_drag_share_of_battery": drag / battery if battery > 0 else float("nan"),
    }


@dataclass
class EnduranceEnergy:
    track_name: str
    laps: int
    distance_km: float
    usable_kWh: float | None
    unlimited_lap_time_s: float
    unlimited_required_kWh: float
    lap_time_s: float
    required_kWh: float
    power_cap_kW: float | None  # None: no derate needed
    feasible: bool
    breakdown: dict[str, float] = field(default_factory=dict)  # per lap, at the final power cap
    result: LapResult | None = None

    @property
    def total_time_s(self) -> float:
        return self.lap_time_s * self.laps

    @property
    def margin_kWh(self) -> float | None:
        return None if self.usable_kWh is None else self.usable_kWh - self.required_kWh

    def report(self) -> str:
        lines = [f"endurance energy - {self.track_name}: {self.laps} laps = {self.distance_km:.1f} km"]
        if self.usable_kWh is None:
            lines.append("  battery.capacity_kWh not set -> no energy check")
        lines.append(f"  flat out:        lap {self.unlimited_lap_time_s:8.3f} s   needs "
                     f"{self.unlimited_required_kWh:6.2f} kWh")
        if self.usable_kWh is not None:
            lines.append(f"  usable battery:  {self.usable_kWh:6.2f} kWh")
            if self.power_cap_kW is None and self.feasible:
                lines.append(f"  -> fits, margin {self.margin_kWh:+.2f} kWh")
            elif self.power_cap_kW is None:
                lines.append(f"  -> does NOT fit flat out: short by {-self.margin_kWh:.2f} kWh")
            elif self.feasible:
                lines.append(f"  -> too much: power cap {self.power_cap_kW:.1f} kW -> lap {self.lap_time_s:8.3f} s, "
                             f"{self.required_kWh:6.2f} kWh, endurance time +"
                             f"{(self.lap_time_s - self.unlimited_lap_time_s) * self.laps:.1f} s")
            else:
                lines.append(f"  -> does not fit even at {self.power_cap_kW:.1f} kW ({self.required_kWh:.2f} kWh)")
        b = self.breakdown
        if b:
            lines.append("  per lap [kWh]:  battery {battery_kWh:.4f} = wheel work {wheel_work_kWh:.4f} + powertrain losses "
                         "{powertrain_losses_kWh:.4f} - regen {regen_returned_kWh:.4f}".format(**b))
            lines.append("                  wheel work = aero drag {aero_drag_kWh:.4f} + rolling {rolling_kWh:.4f} + braking "
                         "{braking_heat_kWh:.4f} + regen {regen_returned_kWh:.4f} + kinetic {kinetic_change_kWh:+.4f}"
                         .format(**b))
            lines.append(f"  aero drag = {b['aero_drag_share_of_battery'] * 100:.0f} % of the battery energy")
        return "\n".join(lines)


def _solve(cfg, features, track, opts):
    car = VehicleModel(cfg, features)
    return car, QSSSolver(opts).solve(car, track, start="flying")


def endurance_energy(cfg: dict, track: Track, distance_km: float = 22.0, features: ModelFeatures | None = None,
                     opts: SolverOptions | None = None, search_opts: SolverOptions | None = None,
                     iterations: int = 12, tol_kWh: float = 0.01, derate: bool = True) -> EnduranceEnergy:
    """Flat-out endurance energy and, if it does not fit the battery, the power cap that makes it fit.

    ``derate=False`` only evaluates the flat-out run (``feasible`` then says whether it fits).
    """
    opts = opts or SolverOptions()
    search_opts = search_opts or SolverOptions.coarse()
    car, res = _solve(cfg, features, track, opts)
    laps = max(1, round(distance_km * 1000.0 / track.length))
    e_lap = res.energy_J / KWH
    usable = car.powertrain.usable_energy
    usable_kwh = None if usable is None else usable / KWH
    out = EnduranceEnergy(track.name, laps, laps * track.length / 1000.0, usable_kwh, res.lap_time, e_lap * laps,
                          res.lap_time, e_lap * laps, None, True, energy_breakdown(car, res), res)
    if usable_kwh is None or e_lap * laps <= usable_kwh:
        return out
    if not derate or car.powertrain.type != "ev":
        out.feasible = False
        return out
    pt = car.powertrain
    if pt.power_limit:
        p_hi = pt.power_limit / 1000.0
    elif pt.motor is not None:
        p_hi = pt.motor.peak_power / 1000.0
    else:  # plain torque curve: peak shaft power over the battery-side efficiency
        p_hi = float((pt.torque * pt.rpm).max() * 2.0 * np.pi / 60.0) * pt.n_motors / pt.efficiency / 1000.0
    p_lo = 5.0

    def energy_at(p_kw, o):
        c = with_overrides(cfg, {"powertrain.ev.power_limit_kW": p_kw, "battery.max_power_kW": None})
        _, r = _solve(c, features, track, o)
        return r.energy_J / KWH * laps

    if energy_at(p_lo, search_opts) > usable_kwh:
        c = with_overrides(cfg, {"powertrain.ev.power_limit_kW": p_lo, "battery.max_power_kW": None})
        car_l, r = _solve(c, features, track, opts)
        out.lap_time_s, out.required_kWh, out.power_cap_kW, out.feasible = r.lap_time, r.energy_J / KWH * laps, p_lo, False
        out.breakdown, out.result = energy_breakdown(car_l, r), r
        return out
    for _ in range(iterations):  # largest cap whose energy fits
        mid = 0.5 * (p_lo + p_hi)
        if energy_at(mid, search_opts) <= usable_kwh:
            p_lo = mid
        else:
            p_hi = mid
        if (p_hi - p_lo) < 0.25:
            break
    # fine envelope at the cap. The coarse envelope can misjudge the energy by a few per cent
    # (braking-dominated laps), so bracket the cap with fine solves and bisect again.
    def fine(p_kw):
        c = with_overrides(cfg, {"powertrain.ev.power_limit_kW": p_kw, "battery.max_power_kW": None})
        car_f, r_f = _solve(c, features, track, opts)
        return car_f, r_f, r_f.energy_J / KWH * laps

    p = p_lo
    car_l, r, e = fine(p)
    if e > usable_kwh + tol_kWh:
        hi, step = p, 2.0
        while p > 5.0:  # step down until the fine energy fits
            p = max(p - step, 5.0)
            car_l, r, e = fine(p)
            if e <= usable_kwh + tol_kWh:
                break
            hi, step = p, 2.0 * step
        lo = p
        if e <= usable_kwh + tol_kWh:
            best = (p, car_l, r)
            while hi - lo > 0.25:
                mid = 0.5 * (lo + hi)
                car_m, r_m, e_m = fine(mid)
                if e_m <= usable_kwh + tol_kWh:
                    lo, best = mid, (mid, car_m, r_m)
                else:
                    hi = mid
            p, car_l, r = best
    out.lap_time_s, out.required_kWh, out.power_cap_kW = r.lap_time, r.energy_J / KWH * laps, p
    out.feasible = out.required_kWh <= usable_kwh + tol_kWh
    out.breakdown, out.result = energy_breakdown(car_l, r), r
    return out


def energy_sweep(cfg: dict, track: Track, path: str, values, distance_km: float = 22.0,
                 features: ModelFeatures | None = None, opts: SolverOptions | None = None,
                 progress=None) -> list[tuple[float, EnduranceEnergy]]:
    """Energy-limited endurance for each value of one parameter (e.g. ``aero.cla_scale``).

    ``path`` may list several comma-separated paths that take the same value, e.g.
    ``"aero.cla_scale,aero.cda_scale"`` (a bigger / smaller wing package).
    """
    out = []
    paths = [p.strip() for p in path.split(",") if p.strip()]
    for k, x in enumerate(values):
        c = with_overrides(cfg, {p: float(x) for p in paths})
        en = endurance_energy(c, track, distance_km, features, opts)
        out.append((float(x), en))
        if progress:
            progress(k + 1, len(values), float(x), en)
    return out


def energy_sweep_table(path: str, rows: list[tuple[float, EnduranceEnergy]]) -> str:
    name = "+".join(p.split(".")[-1] for p in path.split(","))
    head = (f"{name[:18]:>18s} {'flat-out lap':>12s} {'flat-out kWh':>12s} {'drag kWh/lap':>12s} "
            f"{'usable kWh':>10s} {'cap kW':>7s} {'lap s':>8s} {'endurance s':>11s}  fits flat out")
    lines = [head, "-" * len(head)]
    for x, en in rows:
        usable = "-" if en.usable_kWh is None else f"{en.usable_kWh:10.2f}"
        cap = "-" if en.power_cap_kW is None else f"{en.power_cap_kW:7.1f}"
        fits = "-" if en.usable_kWh is None else ("yes" if en.unlimited_required_kWh <= en.usable_kWh else "NO")
        lines.append(f"{x:18.4g} {en.unlimited_lap_time_s:12.3f} {en.unlimited_required_kWh:12.2f} "
                     f"{en.breakdown.get('aero_drag_kWh', float('nan')):12.4f} {usable:>10s} {cap:>7s} "
                     f"{en.lap_time_s:8.3f} {en.total_time_s:11.1f}  {fits}")
    return "\n".join(lines)


def copy_cfg(cfg: dict) -> dict:
    return copy.deepcopy(cfg)
