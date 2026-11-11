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
    pack_loss = integral(ch["battery_loss_kW"] * 1000.0) if "battery_loss_kW" in ch else 0.0
    battery = res.energy_J / KWH
    gross = battery + regen - pack_loss
    return {
        "battery_kWh": battery,
        "wheel_work_kWh": wheel,
        "aero_drag_kWh": drag,
        "rolling_kWh": rolling,
        "braking_heat_kWh": braking - regen,
        "regen_returned_kWh": regen,
        "kinetic_change_kWh": wheel - drag - rolling - braking,
        "powertrain_losses_kWh": gross - wheel,
        "battery_loss_kWh": pack_loss,
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
    stages: list = field(default_factory=list)  # SocStage list (cell-based pack model)

    @property
    def soc_end(self) -> float | None:
        return self.stages[-1].soc_end if self.stages else None

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
                         "{powertrain_losses_kWh:.4f} + pack R I^2 {battery_loss_kWh:.4f} - regen {regen_returned_kWh:.4f}"
                         .format(**b))
            lines.append("                  wheel work = aero drag {aero_drag_kWh:.4f} + rolling {rolling_kWh:.4f} + braking "
                         "{braking_heat_kWh:.4f} + regen {regen_returned_kWh:.4f} + kinetic {kinetic_change_kWh:+.4f}"
                         .format(**b))
            lines.append(f"  aero drag = {b['aero_drag_share_of_battery'] * 100:.0f} % of the battery energy"
                         + (f", pack R I^2 = {b['battery_loss_kWh'] * 1000:.1f} Wh/lap" if b.get("battery_loss_kWh")
                            else ""))
        if self.stages:
            lines.append("  SOC stages (pack model):   laps   SOC start -> end   lap [s]   P_max [kW]   V_min [V]"
                         "   I_max [A]   R I^2 [kWh]")
            for st in self.stages:
                lines.append(f"                             {st.laps:5.1f}   {st.soc_start:5.3f} -> {st.soc_end:5.3f}"
                             f"   {st.lap_time_s:7.3f}   {st.power_kW:10.1f}   {st.v_terminal_min:9.0f}   {st.i_max:9.0f}"
                             f"   {st.loss_kWh:11.3f}")
        return "\n".join(lines)


def _solve(cfg, features, track, opts):
    car = VehicleModel(cfg, features)
    return car, QSSSolver(opts).solve(car, track, start="flying")


@dataclass
class SocStage:
    laps: float
    soc_start: float
    soc_end: float
    lap_time_s: float
    energy_kWh: float  # cell energy of the stage
    v_terminal_min: float  # V, lowest bus voltage on the lap
    i_max: float  # A, highest discharge current
    power_kW: float  # deliverable battery power at the stage SOC
    loss_kWh: float  # R I^2 of the stage


@dataclass
class _Run:
    car: VehicleModel  # the car at the start SOC
    res: LapResult  # representative lap (first stage)
    lap_time: float  # mean lap time
    energy_kWh: float  # whole distance
    stages: list = field(default_factory=list)
    dq_first: float | None = None  # charge per lap of the first stage, C (hint for the next run)


def _lap_charge(res: LapResult) -> float:
    i = res.channels["pack_current_A"]
    return float(np.sum(0.5 * (i[1:] + i[:-1]) * np.diff(res.t)))


def _run(cfg, features, track, laps, opts, dq_hint: float | None = None) -> _Run:
    """Whole endurance: one lap x laps, or SOC stages with the cell-based pack model.

    Each stage is solved once, at its mid SOC. The mid SOC needs the charge per lap, which is taken
    from the previous stage; for the first stage from ``dq_hint`` (e.g. the flat-out run) or, without
    a hint, from one extra lap at the start SOC.
    """
    car = VehicleModel(cfg, features)
    pack = car.powertrain.pack
    if pack is None:
        res = QSSSolver(opts).solve(car, track, start="flying")
        return _Run(car, res, res.lap_time, res.energy_J / KWH * laps)
    n = min(pack.soc_stages, laps)
    per = laps / n
    soc, total_t, total_e, stages = pack.soc, 0.0, 0.0, []
    dq_lap = dq_hint if dq_hint is not None else _lap_charge(QSSSolver(opts).solve(car, track, start="flying"))
    first, dq_first = None, None
    for k in range(n):
        mid = soc - 0.5 * per * dq_lap / pack.charge_C
        # solve the stage lap at its mid SOC, kept inside [soc_min, 1]: when the pack is short (the
        # flat-out run, or a power cap that is still too high) the SOC bookkeeping below soc_min goes on,
        # the lap itself uses the lowest allowed SOC (lowest voltage / power)
        soc_eval = float(min(max(mid, pack.soc_min + 1e-6), 1.0))
        c_k, r_k = _solve(with_overrides(cfg, {"battery.soc": soc_eval}, f"내구 SOC 단계 {k + 1}/{n}"), features,
                          track, opts)
        ch = r_k.channels
        dt = np.diff(r_k.t)
        dq_lap = _lap_charge(r_k)
        if k == 0:
            first, dq_first = r_k, dq_lap
        loss = float(np.sum(0.5 * (ch["battery_loss_kW"][1:] + ch["battery_loss_kW"][:-1]) * dt)) * 1000.0 / KWH
        soc_end = soc - per * dq_lap / pack.charge_C
        e_k = r_k.energy_J / KWH * per
        stages.append(SocStage(per, soc, soc_end, r_k.lap_time, e_k, float(np.min(ch["pack_voltage_V"])),
                               float(np.max(ch["pack_current_A"])), c_k.powertrain.power_limit / 1000.0, loss * per))
        total_t += r_k.lap_time * per
        total_e += e_k
        soc = soc_end
    return _Run(car, first, total_t / laps, total_e, stages, dq_first)


def endurance_energy(cfg: dict, track: Track, distance_km: float = 22.0, features: ModelFeatures | None = None,
                     opts: SolverOptions | None = None, tol_kWh: float = 0.01, derate: bool = True,
                     xtol_kW: float = 0.3) -> EnduranceEnergy:
    """Flat-out endurance energy and, if it does not fit the battery, the power cap that makes it fit.

    The cap is the root of ``energy(cap) = usable`` (Brent's method, ``xtol_kW``), evaluated with
    ``opts`` and interpolated between the two evaluations that bracket it. With the cell-based pack
    model the distance is driven in SOC stages (voltage and deliverable power fall as the pack
    discharges). ``derate=False`` only evaluates the flat-out run (``feasible`` says whether it fits).
    """
    from scipy.optimize import brentq

    opts = opts or SolverOptions()
    laps = max(1, round(distance_km * 1000.0 / track.length))
    run = _run(cfg, features, track, laps, opts)
    car = run.car
    usable = car.powertrain.usable_energy
    usable_kwh = None if usable is None else usable / KWH
    out = EnduranceEnergy(track.name, laps, laps * track.length / 1000.0, usable_kwh, run.lap_time, run.energy_kWh,
                          run.lap_time, run.energy_kWh, None, True, energy_breakdown(car, run.res), run.res,
                          run.stages)
    if usable_kwh is None or run.energy_kWh <= usable_kwh:
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
    memo: dict[float, _Run] = {}

    def capped(p_kw: float) -> _Run:
        key = round(float(p_kw), 6)
        if key not in memo:
            c = with_overrides(cfg, {"powertrain.ev.power_limit_kW": key, "battery.max_power_kW": None},
                               f"내구 출력 캡 탐색 ({key:.1f} kW)")
            memo[key] = _run(c, features, track, laps, opts, run.dq_first)
        return memo[key]

    def finish(p, r: _Run):
        out.lap_time_s, out.required_kWh, out.power_cap_kW = r.lap_time, r.energy_kWh, p
        out.feasible = out.required_kWh <= usable_kwh + tol_kWh
        out.breakdown, out.result, out.stages = energy_breakdown(r.car, r.res), r.res, r.stages
        return out

    if p_hi <= p_lo or capped(p_lo).energy_kWh > usable_kwh:
        return finish(p_lo, capped(p_lo))
    memo[round(p_hi, 6)] = run  # the flat-out run is the upper end of the bracket
    brentq(lambda p: capped(p).energy_kWh - usable_kwh, p_lo, p_hi, xtol=xtol_kW, rtol=1e-6, maxiter=40)
    fits = [k for k, r in memo.items() if r.energy_kWh <= usable_kwh + tol_kWh]
    best = max(fits)
    finish(best, memo[best])
    # The exact cap lies between the best fitting evaluation and the next one above it; interpolate
    # cap and lap time there so the result varies smoothly with the car (no root-finder jitter).
    above = [k for k in memo if k > best and memo[k].energy_kWh > usable_kwh]
    if above and memo[best].energy_kWh < usable_kwh:
        nxt = min(above)
        a, b = memo[best], memo[nxt]
        if nxt - best <= max(5.0 * xtol_kW, 1.0) and b.energy_kWh > a.energy_kWh:
            w = (usable_kwh - a.energy_kWh) / (b.energy_kWh - a.energy_kWh)
            out.power_cap_kW = best + w * (nxt - best)
            out.lap_time_s = a.lap_time + w * (b.lap_time - a.lap_time)
            out.required_kWh = usable_kwh
    return out


def energy_sweep(cfg: dict, track: Track, path: str, values, distance_km: float = 22.0,
                 features: ModelFeatures | None = None, opts: SolverOptions | None = None,
                 progress=None, relative: bool = False) -> list[tuple[float, EnduranceEnergy]]:
    """Energy-limited endurance for each value of one parameter (e.g. ``aero.cla_scale``).

    ``path`` may list several comma-separated paths, e.g. ``"aero.cla_scale,aero.cda_scale"`` (a
    bigger / smaller wing package). They all take the same value, or with ``relative=True`` each
    path's current value times the sweep value (keeps e.g. individually optimised device scales).
    """
    from ..config import get_path

    out = []
    paths = [p.strip() for p in path.split(",") if p.strip()]
    base = {p: float(get_path(cfg, p)) for p in paths} if relative else {}
    for k, x in enumerate(values):
        c = with_overrides(cfg, {p: float(x) * base[p] if relative else float(x) for p in paths},
                           f"에너지 스윕 {path} = {x:g}" + (" x 현재 값" if relative else ""))
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
