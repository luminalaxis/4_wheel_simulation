"""Energy-optimal endurance driving: lift-and-coast and throttle modulation by dynamic programming.

When the battery cannot hold a flat-out endurance, ``endurance_energy`` lowers the battery power cap
until it fits. A power cap is easy to apply but not the cheapest way to save energy: most of the lap
energy ends up as brake heat, and energy is saved best where it costs the least time - by lifting
before the braking points (coasting on drag instead of braking) and by not accelerating hard into
short straights.

This module finds the speed profile that minimises ``lap time + lambda * battery energy`` exactly
(within the speed grid) by dynamic programming over (track node, speed), and the ``lambda`` at which
the whole endurance uses the usable battery energy:

* state: speed at each node of the QSS track grid, ``v_k = v_flat_out(s) - k dv`` (the flat-out QSS
  profile is the upper limit of every feasible profile, so the grid is anchored on it and contains it);
* transitions ``v_a -> v_b`` over one interval are allowed when the QSS Heun steps allow them: ``v_b`` is
  not above the speed reachable with full acceleration from ``v_a``, and ``v_a`` not above the speed
  from which full braking still reaches ``v_b`` - the same g-g-V limits (tyres, power, aero) as the
  flat-out lap, which therefore is the ``lambda = 0`` solution;
* stage cost: time ``2 ds / (v_a + v_b)`` plus ``lambda`` x battery energy at the mid speed: tyre force
  ``m (a_x - a_coast)`` (``a_coast`` = drag + rolling resistance deceleration of the envelope),
  motor / inverter / drivetrain efficiency, rotor inertia, regeneration and pack ``R I^2`` exactly as the
  lap energy of the QSS solver;
* coasting (zero tyre force, no battery energy) is an extra, exact transition to an off-grid speed whose
  value is interpolated on the next node's grid - a speed grid alone cannot represent the slow drag
  deceleration of a coast and would replace it with alternating throttle and brake steps;
* closed laps: relative value iteration until the policy is periodic; the lap is then driven forward from
  the best start speed, choosing at every node (from the actual, possibly off-grid speed) the transition
  with the lowest cost plus interpolated cost-to-go.

The chosen profile is evaluated with the standard QSS post-processing (all channels, energy).
With the cell-based pack model the endurance is split into the SOC stages of the power-cap run (same
start and end SOC), one car per stage and one common ``lambda`` (Lagrange: the marginal time per joule
is the same over the whole run).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..config import with_overrides
from ..features import ModelFeatures
from ..solvers.base import LapResult
from ..solvers.ggv import SolverOptions
from ..solvers.qss import QSSSolver
from ..track import Track
from ..vehicle import VehicleModel
from .energy import KWH, endurance_energy, energy_breakdown


class LapDP:
    """Transition table of one car on one closed track; ``solve(lam)`` returns the optimal profile.

    ``lam`` is in seconds per kJ of battery energy.
    """

    def __init__(self, car: VehicleModel, track: Track, opts: SolverOptions | None = None, dv: float = 0.1,
                 depth: float = 0.55, band_mps: float | None = None, chunk: int = 64):
        if not track.closed:
            raise ValueError("the energy-optimal strategy needs a closed (endurance) track")
        self.car = car
        self.solver = QSSSolver(opts)
        self.flat = self.solver.solve(car, track, start="flying")
        trk = track.resampled(self.solver.opts.ds)
        self.track = track
        self.trk = trk
        v_fo = np.asarray(self.flat.v, dtype=float).copy()
        v_fo[-1] = v_fo[0] = min(v_fo[0], v_fo[-1])  # periodic: node N is node 0
        self.v_fo = v_fo
        self.modes = np.asarray(self.flat.mode, dtype=int)
        n = len(trk.s) - 1
        self.n = n
        self.dv = dv
        self.K = K = max(int(math.ceil(depth * float(v_fo.max()) / dv)), 8)
        ds_max = float(np.max(np.diff(trk.s)))
        band_mps = 3.5 * max(ds_max, 0.25) if band_mps is None else band_mps
        self.B = B = max(int(math.ceil(band_mps / dv)), 4)
        V = v_fo[:, None] - dv * np.arange(K)[None, :]
        self.ok = V >= 0.5  # speeds below 0.5 m/s are not states
        self.V = np.maximum(V, 0.5)
        ds = np.diff(trk.s)
        k_int = np.abs(trk.interval_kappa())
        pt = car.powertrain
        pack = pt.pack
        ggvs = {m: self.solver.ggv(car, int(m)) for m in np.unique(self.modes)}
        self.ggvs, self.ds, self.k_int = ggvs, ds, k_int
        # Heun steps of the QSS: reachable speed with full acceleration / highest speed that can still brake;
        # coasting (zero tyre force) from every grid speed
        vf = np.empty((n, K))
        vb = np.empty((n, K))
        vc = np.empty((n, K))
        for m, g in ggvs.items():
            sel = np.flatnonzero(self.modes[:-1] == m)
            if sel.size == 0:
                continue
            va = self.V[sel]
            kk = k_int[sel][:, None]
            d = ds[sel][:, None]
            vf[sel] = self._heun(g, "acc", va, kk, d)
            vc[sel] = self._heun(g, "coast", va, kk, d)
            vn = self.V[sel + 1]
            d1 = g.table_vec("brk", vn, vn * vn * kk)
            vq = np.sqrt(np.maximum(vn * vn - 2.0 * d1 * d, 0.0))
            d2 = g.table_vec("brk", vq, vq * vq * kk)
            vb[sel] = np.sqrt(np.maximum(vn * vn - (d1 + d2) * d, 0.0))
        self.vb = vb
        # coast target on the next grid: position x = (v_fo[i+1] - v_c) / dv, linear interpolation of J;
        # not allowed above the flat-out speed (a later constraint could not be met) or below the grid
        x = (v_fo[1:, None] - vc) / dv
        self.c_ok = self.ok[:-1] & (x >= -1e-9) & (x <= K - 1) & (vc >= 0.5)
        x = np.clip(x, 0.0, K - 1 - 1e-9)
        self.c_lo = np.floor(x).astype(np.int32)
        self.c_w = x - self.c_lo
        self.c_dt = np.where(self.c_ok, 2.0 * ds[:, None] / (self.V[:-1] + vc), np.inf)
        offs = np.arange(-B, B + 1)
        self.kb = np.arange(K)[:, None] + offs[None, :]  # target index per (k, offset)
        kb_ok = (self.kb >= 0) & (self.kb < K)
        kb_c = np.clip(self.kb, 0, K - 1)
        self.kb_c = kb_c
        dt = np.full((n, K, 2 * B + 1), np.inf, dtype=np.float32)
        de = np.zeros((n, K, 2 * B + 1), dtype=np.float32)
        m_car, m_rot = car.m, pt.m_rot
        for lo in range(0, n, chunk):
            idx = np.arange(lo, min(lo + chunk, n))
            va = self.V[idx][:, :, None]
            vbb = self.V[idx + 1][:, kb_c]
            feas = (kb_ok[None] & self.ok[idx][:, :, None] & self.ok[idx + 1][:, kb_c]
                    & (vbb <= vf[idx][:, :, None] * (1.0 + 1e-9) + 1e-9)
                    & (va <= vb[idx][:, kb_c] * (1.0 + 1e-9) + 1e-9))
            d = ds[idx][:, None, None]
            t = 2.0 * d / (va + vbb)
            ax = (vbb * vbb - va * va) / (2.0 * d)
            vm = 0.5 * (va + vbb)
            ay = vm * vm * k_int[idx][:, None, None]
            coast = np.empty(vm.shape)
            for m, g in ggvs.items():
                sel = self.modes[idx] == m
                if sel.any():
                    coast[sel] = g.table_vec("coast", vm[sel], ay[sel])
            force = m_car * (ax - coast)
            f_drive = np.maximum(force, 0.0)
            f_motor = f_drive + m_rot * np.maximum(ax, 0.0) * (f_drive > 0.0)
            p = pt.battery_power(vm, f_motor)
            if pt.regen_power > 0.0:
                p = p - pt.regen_battery_power(vm, np.maximum(-force, 0.0))
            if pack is not None:
                i_pack = pack.current(p)
                p = p + pack.resistance * i_pack * i_pack
            dt[idx] = np.where(feas, t, np.inf)
            de[idx] = np.where(feas, p * t, 0.0)
        self.dt, self.de = dt, de
        self._kb_ok = kb_ok
        self._j_end_hint = None

    @staticmethod
    def _heun(g, which, va, kk, d):
        a1 = g.table_vec(which, va, va * va * kk)
        vp = np.sqrt(np.maximum(va * va + 2.0 * a1 * d, 1e-6))
        a2 = g.table_vec(which, vp, vp * vp * kk)
        return np.sqrt(np.maximum(va * va + (a1 + a2) * d, 1e-6))

    def _interp_j(self, j_next, lo, w):
        hi = np.minimum(lo + 1, self.K - 1)
        a, b = j_next[lo], j_next[hi]
        out = (1.0 - w) * a + w * b
        # an infeasible neighbour only counts when the target is (almost) on it
        out = np.where(np.isfinite(a) & np.isfinite(b), out, np.where(w < 1e-6, a, np.where(w > 1.0 - 1e-6, b, np.inf)))
        return out

    # ------------------------------------------------------------------ dynamic programming
    def _backward(self, lam_j: float, j_end: np.ndarray):
        n, K = self.n, self.K
        jall = np.empty((n + 1, K))
        jall[n] = j_end
        j = j_end
        rows = np.arange(K)
        for i in range(n - 1, -1, -1):
            jn = np.where(self._kb_ok, j[self.kb_c], np.inf)
            tot = self.dt[i] + lam_j * self.de[i] + jn
            best = tot[rows, np.argmin(tot, axis=1)]
            coast = self.c_dt[i] + self._interp_j(j, self.c_lo[i], self.c_w[i])
            j = np.minimum(best, coast)
            jall[i] = j
        return jall

    def _step_costs(self, i: int, va: float, lam_j: float, j_next: np.ndarray):
        """From the actual speed ``va`` at node i: (next speed, dt, energy) of the best transition."""
        car, pt = self.car, self.car.powertrain
        g = self.ggvs[int(self.modes[i])]
        d = float(self.ds[i])
        kk = float(self.k_int[i])
        vb = self.V[i + 1]
        vfa = float(self._heun(g, "acc", np.array([va]), kk, d)[0])
        feas = self.ok[i + 1] & (vb <= vfa * (1.0 + 1e-9) + 1e-9) & (va <= self.vb[i] * (1.0 + 1e-9) + 1e-9)
        t = 2.0 * d / (va + vb)
        ax = (vb * vb - va * va) / (2.0 * d)
        vm = 0.5 * (va + vb)
        force = car.m * (ax - g.table_vec("coast", vm, vm * vm * kk))
        f_drive = np.maximum(force, 0.0)
        p = pt.battery_power(vm, f_drive + pt.m_rot * np.maximum(ax, 0.0) * (f_drive > 0.0))
        if pt.regen_power > 0.0:
            p = p - pt.regen_battery_power(vm, np.maximum(-force, 0.0))
        if pt.pack is not None:
            i_pack = pt.pack.current(p)
            p = p + pt.pack.resistance * i_pack * i_pack
        e = p * t
        tot = np.where(feas, t + lam_j * e + j_next, np.inf)
        k = int(np.argmin(tot))
        best = (float(vb[k]), float(t[k]), float(e[k]), float(tot[k]))
        vc = float(self._heun(g, "coast", np.array([va]), kk, d)[0])
        x = (self.v_fo[i + 1] - vc) / self.dv
        if vc >= 0.5 and -1e-9 <= x <= self.K - 1:
            x = min(max(x, 0.0), self.K - 1 - 1e-9)
            lo = int(x)
            jc = float(self._interp_j(j_next, np.array([lo]), np.array([x - lo]))[0])
            tc = 2.0 * d / (va + vc)
            if tc + jc < best[3]:
                best = (vc, tc, 0.0, tc + jc)
        return best

    def solve(self, lam: float, max_pass: int = 8):
        """Optimal periodic profile for ``lam`` [s/kJ]: (speeds at the nodes, time, battery energy [J])."""
        lam_j = float(lam) / 1000.0
        j_end = self._j_end_hint if self._j_end_hint is not None else np.zeros(self.K)
        prev = None
        for _ in range(max_pass):
            jall = self._backward(lam_j, j_end)
            j0 = jall[0]
            if not np.isfinite(j0).any():
                raise RuntimeError("no feasible speed profile (grid too coarse?)")
            k0 = int(np.argmin(j0))
            if prev is not None and np.allclose(j0 - j0[k0], prev, atol=1e-6, equal_nan=True):
                break
            prev = j0 - j0[k0]
            j_end = j0 - j0[k0]
        self._j_end_hint = np.where(np.isfinite(j_end), j_end, np.inf)  # warm start of the next lambda
        v = np.empty(self.n + 1)
        v[0] = self.V[0, k0]
        t_sum = e_sum = 0.0
        for i in range(self.n):
            v[i + 1], dt, de, _ = self._step_costs(i, float(v[i]), lam_j, jall[i + 1])
            t_sum += dt
            e_sum += de
        return v, t_sum, e_sum

    def lap(self, lam: float) -> LapResult:
        """Optimal profile for ``lam`` evaluated like any QSS lap (channels, energy)."""
        v, _, _ = self.solve(lam)
        return self.solver.evaluate_profile(self.car, self.trk, v, self.modes)


@dataclass
class LiftPoint:
    s_lift: float  # m, where the driver lifts off the throttle
    s_brake: float  # m, where braking starts (or the corner, if no braking follows)
    v_lift_kmh: float
    v_end_kmh: float

    @property
    def coast_m(self) -> float:
        return self.s_brake - self.s_lift


def lift_points(res: LapResult, min_coast_m: float = 2.0, f_tol: float = 30.0) -> list[LiftPoint]:
    """Coasting stretches (no drive, no brake force) that end in braking or a slower corner."""
    ch = res.channels
    drive = ch["drive_force_N"] > f_tol
    brake = ch["brake_force_N"] > f_tol
    coast = ~drive & ~brake
    s, v = res.s, res.v
    out = []
    i, n = 0, len(s)
    while i < n:
        if coast[i] and (i == 0 or drive[i - 1]):
            j = i
            while j < n - 1 and coast[j + 1]:
                j += 1
            if s[j] - s[i] >= min_coast_m and (j + 1 >= n or not drive[j + 1] or v[j] < v[i] - 0.3):
                out.append(LiftPoint(float(s[i]), float(s[min(j + 1, n - 1)]), float(v[i] * 3.6), float(v[j] * 3.6)))
            i = j + 1
        else:
            i += 1
    return out


@dataclass
class StrategyStage:
    laps: float
    soc: float | None
    dp: LapDP = field(repr=False)


@dataclass
class EnduranceStrategy:
    track_name: str
    laps: int
    usable_kWh: float | None
    flat_lap_s: float
    flat_kWh: float  # whole distance
    cap: object  # EnduranceEnergy of the power-cap strategy
    lam: float  # s/kJ
    lap_s: float  # mean lap time of the optimal strategy
    required_kWh: float
    result: LapResult  # first-stage optimal lap
    lifts: list[LiftPoint]
    pareto_opt: list[tuple[float, float]] = field(default_factory=list)  # (kWh / lap, lap s), first stage
    pareto_cap: list[tuple[float, float, float]] = field(default_factory=list)  # (kWh / lap, lap s, cap kW)
    stages: list = field(default_factory=list)
    stage_laps: list = field(default_factory=list)  # (laps, lap s, kWh) per stage
    feasible: bool = True
    mix: float | None = None  # share of laps driven with the next-lower lambda to use the budget exactly

    @property
    def total_s(self) -> float:
        return self.lap_s * self.laps

    @property
    def gain_s(self) -> float:
        return self.cap.total_time_s - self.total_s

    def report(self) -> str:
        c = self.cap
        lines = [f"energy-optimal endurance - {self.track_name}: {self.laps} laps"]
        if self.usable_kWh is None:
            lines.append("  battery.capacity_kWh not set -> nothing to save (flat out)")
        lines.append(f"  flat out:            lap {self.flat_lap_s:8.3f} s   {self.flat_kWh:6.2f} kWh")
        if self.usable_kWh is not None:
            lines.append(f"  usable battery:      {self.usable_kWh:6.2f} kWh")
        cap = "no cap" if c.power_cap_kW is None else f"cap {c.power_cap_kW:5.1f} kW"
        lines.append(f"  power cap:           lap {c.lap_time_s:8.3f} s   {c.required_kWh:6.2f} kWh   total "
                     f"{c.total_time_s:8.1f} s   ({cap})")
        lines.append(f"  energy-optimal:      lap {self.lap_s:8.3f} s   {self.required_kWh:6.2f} kWh   total "
                     f"{self.total_s:8.1f} s   (lambda {self.lam:.4g} s/kJ)")
        if self.mix:
            lines.append(f"     ({self.mix * 100:.0f} % of the laps with the next-lower lambda, so the budget is used "
                         "exactly)")
        lines.append(f"  -> {self.gain_s:+.1f} s over the endurance ({self.gain_s / max(c.total_time_s, 1e-9) * 100:+.2f} %)"
                     + ("" if self.feasible else "   WARNING: does not fit the battery"))
        if self.stage_laps and len(self.stage_laps) > 1:
            lines.append("  SOC stages:  laps   SOC    lap [s]   kWh")
            for (laps, t, e), st in zip(self.stage_laps, self.stages):
                lines.append(f"              {laps:4.1f}   {st.soc:4.2f}   {t:7.3f}   {e:6.3f}")
        b = energy_breakdown(self.stages[0].dp.car, self.result) if self.stages else {}
        cb = c.breakdown
        if b and cb:
            lines.append("  per lap [kWh]            power cap   optimal")
            for key, label in (("battery_kWh", "battery"), ("braking_heat_kWh", "brake heat"),
                               ("aero_drag_kWh", "aero drag"), ("powertrain_losses_kWh", "powertrain losses"),
                               ("battery_loss_kWh", "pack R I^2")):
                lines.append(f"    {label:20s} {cb.get(key, float('nan')):10.4f} {b.get(key, float('nan')):9.4f}")
        if self.lifts:
            lines.append(f"  lift-and-coast points of the optimal lap ({len(self.lifts)}, coasting "
                         f"{sum(p.coast_m for p in self.lifts):.0f} m per lap):")
            lines.append("      lift at [m]   coast [m]   speed at lift -> end [km/h]")
            for p in self.lifts:
                lines.append(f"      {p.s_lift:10.1f}   {p.coast_m:9.1f}   {p.v_lift_kmh:13.1f} -> {p.v_end_kmh:5.1f}")
        if self.pareto_opt:
            lines.append("  time-energy trade-off per lap (first stage):  optimal [kWh, s] | power cap [kWh, s, kW]")
            for i in range(max(len(self.pareto_opt), len(self.pareto_cap))):
                a = self.pareto_opt[i] if i < len(self.pareto_opt) else None
                b2 = self.pareto_cap[i] if i < len(self.pareto_cap) else None
                lines.append("      " + (f"{a[0]:7.4f} {a[1]:8.3f}" if a else " " * 16) + "   |   "
                             + (f"{b2[0]:7.4f} {b2[1]:8.3f} {b2[2]:6.1f}" if b2 else ""))
        return "\n".join(lines)


def _stage_cfgs(cfg: dict, cap) -> list[tuple[float, float | None, dict]]:
    """(laps, soc, cfg) per stage: the SOC stages of the power-cap run (pack model) or one stage."""
    if cap.stages:
        out = []
        for k, st in enumerate(cap.stages):
            soc = 0.5 * (st.soc_start + st.soc_end)
            pack_soc_min = cfg.get("battery", {}).get("soc_min", 0.05)
            soc = float(min(max(soc, float(pack_soc_min) + 1e-6), 1.0))
            out.append((st.laps, soc, with_overrides(cfg, {"battery.soc": soc}, f"에너지 최적 주행 SOC 단계 {k + 1}")))
        return out
    return [(float(cap.laps), None, cfg)]


def endurance_strategy(cfg: dict, track: Track, distance_km: float = 22.0, features: ModelFeatures | None = None,
                       opts: SolverOptions | None = None, dv: float = 0.1, target_kWh: float | None = None,
                       pareto: bool = True, progress=None) -> EnduranceStrategy:
    """Energy-optimal endurance vs the power-cap derate, for the usable battery energy (or ``target_kWh``)."""
    opts = opts or SolverOptions()
    log = progress or (lambda msg: None)
    log("power-cap reference ...")
    cap = endurance_energy(cfg, track, distance_km, features, opts)
    usable = cap.usable_kWh if target_kWh is None else float(target_kWh)
    stages = []
    for laps, soc, c in _stage_cfgs(cfg, cap):
        log(f"transition table{'' if soc is None else f' (SOC {soc:.2f})'} ...")
        stages.append(StrategyStage(laps, soc, LapDP(VehicleModel(c, features), track, opts, dv=dv)))

    def run(lam):
        laps, total_t, total_e = [], 0.0, 0.0
        for st in stages:
            res = st.dp.lap(lam)
            e = res.energy_J / KWH
            laps.append((st.laps, res.lap_time, e, res))
            total_t += res.lap_time * st.laps
            total_e += e * st.laps
        return total_t, total_e, laps

    flat_t, flat_e, flat_laps = run(0.0)
    lam, best = 0.0, (flat_t, flat_e, flat_laps)
    above = None  # (time, energy) of the strategy just above the budget
    feasible = True
    if usable is not None and flat_e > usable:
        lo, hi = 1e-5, 1e-2
        above = (flat_t, flat_e)
        while True:  # bracket: energy(hi) <= usable
            t, e, lp = run(hi)
            if e <= usable or hi > 1e3:
                break
            lo, hi, above = hi, hi * 10.0, (t, e)
        best, lam = (t, e, lp), hi
        feasible = e <= usable
        if feasible:
            for _ in range(16):  # bisection in log(lambda): smallest lambda that fits
                mid = math.sqrt(lo * hi)
                t, e, lp = run(mid)
                log(f"  lambda {mid:.4g} s/kJ: {t:.1f} s, {e:.3f} kWh")
                if e <= usable:
                    hi, best, lam = mid, (t, e, lp), mid
                else:
                    lo, above = mid, (t, e)
                if hi / lo < 1.03:
                    break
    total_t, total_e, lp = best
    mixed = None
    if feasible and usable is not None and lam > 0.0 and above is not None and above[1] > total_e:
        # driving part of the laps with the next-lower lambda uses the battery exactly: the mix of two
        # strategies is itself a strategy, linear in (energy, time)
        w = (usable - total_e) / (above[1] - total_e)
        if 0.0 < w < 1.0:
            mixed = w
            total_t = total_t + w * (above[0] - total_t)
            total_e = usable
    n_laps = cap.laps
    first = lp[0][3]
    out = EnduranceStrategy(track.name, n_laps, usable, flat_t / n_laps, flat_e, cap, lam, total_t / n_laps, total_e,
                            first, lift_points(first), stages=stages,
                            stage_laps=[(a, b, c) for a, b, c, _ in lp], feasible=feasible, mix=mixed)
    if pareto:
        log("time-energy trade-off ...")
        dp0 = stages[0].dp
        lams = sorted({0.0, *(lam * f for f in (0.25, 0.5, 1.0, 2.0, 4.0) if lam > 0)} or {0.0})
        if lam == 0.0:
            lams = [0.0, 0.002, 0.005, 0.01, 0.02, 0.05]
        for x in lams:
            r = dp0.lap(x)
            out.pareto_opt.append((r.energy_J / KWH, r.lap_time))
        c0 = _stage_cfgs(cfg, cap)[0][2]
        car0 = dp0.car
        p_hi = car0.powertrain.power_limit / 1000.0 if car0.powertrain.power_limit else 80.0
        for p_kw in np.linspace(0.45 * p_hi, p_hi, 6):
            cc = with_overrides(c0, {"powertrain.ev.power_limit_kW": float(p_kw), "battery.max_power_kW": None},
                                f"출력 캡 {p_kw:.1f} kW")
            r = QSSSolver(opts).solve(VehicleModel(cc, features), track, start="flying")
            out.pareto_cap.append((r.energy_J / KWH, r.lap_time, float(p_kw)))
    return out
