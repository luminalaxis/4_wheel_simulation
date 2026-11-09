"""Transient (time-domain) lap solver.

What it adds to the quasi-steady-state solver:

* **ride dynamics (7 DOF)** - sprung-mass heave, pitch and roll plus a vertical degree of
  freedom for every wheel (unsprung mass between the suspension and the tyre spring). Corner
  springs, dampers, the third (heave) springs and the anti-roll bars act on each corner's
  suspension travel. Ride heights, pitch and roll lag behind the loads and can overshoot, the
  wheels can hop, and the aero map sees the *dynamic* floor height above the road;
* **road input** - optional rough road (ISO 8608 class or measured profile, left and right
  wheel tracks) under every wheel: tyre loads fluctuate (less average grip because of the
  tyre load sensitivity) and the floor height varies (aero, bottoming);
* **tyre relaxation** - the grip follows tyre load changes with a lag of ``relaxation_length``;
* **dynamic load transfer** - tyre loads come from the tyre deflection; the elastic part of the
  load transfer arrives through springs / dampers, the geometric part (anti, roll centre)
  through the links into the wheels;
* **sideslip lag** - the aero yaw angle relaxes to its steady value with ``yaw_lag``;
* **aero actuator dynamics** - the DRS / active-aero flap moves at a finite rate
  (``open_delay_s`` / ``close_delay_s``) and the coefficients are blended in between;
* **driver** - follows the QSS speed profile (feed-forward acceleration + speed-error gain)
  with a jerk limit and ramp anticipation, limited at every step by what the tyres and the
  powertrain can deliver at the current tyre loads. Where the tyres were over-used the speed
  target is lowered and the lap is driven again (``iterations`` learning laps).

The vehicle follows the track centre line (ay = v^2 kappa). Ride states live in the vehicle
frame (left / right), so roll that is still present at a change of direction is carried over
correctly. The force building blocks are the ones the QSS solver uses
(``VehicleModel.load_components``, ``corner_camber``, ``grip_usage``); on a smooth road a car
in steady state gives the QSS answer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..road import RoadProfile
from ..vehicle import G, LIMIT_NAMES, VehicleModel
from .base import LapResult, powertrain_channels
from .ggv import SolverOptions
from .qss import QSSSolver, _ShiftModel

FL, FR, RL, RR = 0, 1, 2, 3  # vehicle frame corner order of the ride model


@dataclass
class TransientOptions:
    dt: float = 0.005  # s, main step (driver, tyres, aero)
    ride_dt: float = 0.0005  # s, sub-step of the ride (body + wheel) dynamics
    kp: float = 8.0  # 1/s, driver speed-error gain
    max_jerk: float = 150.0  # m/s^3, how fast the driver can apply brakes / throttle
    max_jerk_release: float = 400.0  # m/s^3, how fast brakes / throttle can be released
    excess_filter: float = 0.1  # s, tyre over-use is low-pass filtered before learning
    turn_in_length: float = 3.0  # m, curvature is smoothed over this length (steering transition)
    yaw_lag: float = 0.05  # s, sideslip relaxation time
    iterations: int = 4  # learning laps after the first one
    excess_tol: float = 0.02  # allowed tyre over-utilisation before re-planning
    learn_gain: float = 0.3  # speed-target reduction per unit excess and learning lap
    n_search: int = 10  # candidate accelerations per refinement round
    lateral_dof: bool = True  # lateral / yaw dynamics with a steering driver (False: follow the line exactly)
    max_path_error: float = 0.75  # m, allowed lateral deviation of the CG from the line
    spin_sideslip_deg: float = 15.0  # sideslip beyond which the car is counted as spun
    slide_sideslip_deg: float = 8.0  # sideslip from which the driver learns to slow down
    steer_preview: float = 0.05  # s, the steering feed-forward looks this far ahead
    steer_lag: float = 0.03  # s, steering actuation lag
    lift_gain: float = 3.0  # (m/s) per m, speed-target cut when the car runs wide of the line
    yaw_rate_gain: float = 0.08  # s, steer correction per rad/s of yaw rate above the line's
    v_kinematic: float = 3.0  # m/s, below this the lateral motion is kinematic
    road: str | None = None  # None/"flat", ISO 8608 class "A".."E", or a profile CSV path
    road_seed: int = 1
    road_scale: float = 1.0  # multiplies the road amplitude
    relaxation_length: float = 0.25  # m, tyre force follows load changes with this lag
    min_unsprung_kg: float = 3.0  # per corner, used when the vehicle has less (or none)
    tyre_damping_ratio: float = 0.05
    qss: SolverOptions = field(default_factory=SolverOptions)


class RideModel:
    """7-DOF vertical model in the vehicle frame (corners FL, FR, RL, RR).

    Body: heave ``h`` (down +), pitch ``th`` (nose down +), roll ``ph`` (right side down +).
    Wheels: vertical displacement ``w`` (down +). Road height ``r`` (up +). All are deviations
    from the static equilibrium. Corner body displacement ``d = h + x th + y ph``, suspension
    compression ``delta = d - w``, tyre compression ``w + r``.

    Generalised loads per main step: ``el_f, el_r`` (axle loads the springs carry in steady
    state), ``m_roll`` (roll moment, vehicle frame) and ``geo`` (per-corner load transfer through
    the links, applied to the wheels).
    """

    def __init__(self, car: VehicleModel, min_unsprung_kg: float = 3.0, tyre_damping_ratio: float = 0.05):
        self.car = car
        fr, rr = car.front, car.rear
        self.x = [car.a_s, car.a_s, -car.b_s, -car.b_s]
        self.y = [-0.5 * fr.track, 0.5 * fr.track, -0.5 * rr.track, 0.5 * rr.track]
        self.kw = [fr.wheel_rate, fr.wheel_rate, rr.wheel_rate, rr.wheel_rate]
        self.cw = [0.5 * car.C_heave_f] * 2 + [0.5 * car.C_heave_r] * 2
        self.k3 = [fr.heave_rate, rr.heave_rate]
        self.karb = [fr.arb_rate / fr.track ** 2, rr.arb_rate / rr.track ** 2]
        mu_f = max(0.5 * car.m_uf, min_unsprung_kg)
        mu_r = max(0.5 * car.m_ur, min_unsprung_kg)
        self.mu = [mu_f, mu_f, mu_r, mu_r]
        self.kt = car.tire.vertical_rate
        self.ct = [2.0 * tyre_damping_ratio * math.sqrt(self.kt * m) for m in self.mu]
        self.fz0 = [0.5 * car.W_f, 0.5 * car.W_f, 0.5 * car.W_r, 0.5 * car.W_r]
        self.k_grav = car.m_s * G * car.d_roll
        self.h = self.hd = self.th = self.thd = self.ph = self.phd = 0.0
        self.w = [0.0] * 4
        self.wd = [0.0] * 4

    # ---------------------------------------------------------------- forces
    def _forces(self, h, hd, th, thd, ph, phd, w, wd, r, rd):
        x, y = self.x, self.y
        d = [h + x[i] * th + y[i] * ph for i in range(4)]
        dd = [hd + x[i] * thd + y[i] * phd for i in range(4)]
        dl = [d[i] - w[i] for i in range(4)]
        ddl = [dd[i] - wd[i] for i in range(4)]
        f = [self.kw[i] * dl[i] + self.cw[i] * ddl[i] for i in range(4)]
        for a, (i, j) in enumerate(((FL, FR), (RL, RR))):
            third = 0.25 * self.k3[a] * (dl[i] + dl[j])
            arb = self.karb[a] * (dl[j] - dl[i])
            f[i] += third - arb
            f[j] += third + arb
        t = [max(self.kt * (w[i] + r[i]) + self.ct[i] * (wd[i] + rd[i]), -self.fz0[i]) for i in range(4)]
        return f, t, dl, ddl

    def _accel(self, state, loads, r, rd):
        h, hd, th, thd, ph, phd, w, wd = state
        el_f, el_r, m_roll, geo = loads
        c = self.car
        f, t, _, _ = self._forces(h, hd, th, thd, ph, phd, w, wd, r, rd)
        sf = sum(f)
        hdd = (el_f + el_r - sf) / c.m_s
        thdd = (c.a_s * el_f - c.b_s * el_r - sum(f[i] * self.x[i] for i in range(4))) / c.I_pitch
        phdd = (m_roll + self.k_grav * ph - sum(f[i] * self.y[i] for i in range(4))) / c.I_roll
        wdd = [(f[i] + geo[i] - t[i]) / self.mu[i] for i in range(4)]
        return hdd, thdd, phdd, wdd

    # ---------------------------------------------------------------- integration
    def step(self, loads, road_fn, t0: float, dt: float, n_sub: int) -> None:
        """Advance ``dt`` with ``n_sub`` semi-implicit Euler sub-steps; ``road_fn(t)`` -> (r, rd)."""
        h_ = dt / n_sub
        for k in range(n_sub):
            r, rd = road_fn(t0 + k * h_)
            hdd, thdd, phdd, wdd = self._accel(
                (self.h, self.hd, self.th, self.thd, self.ph, self.phd, self.w, self.wd), loads, r, rd)
            self.hd += hdd * h_
            self.thd += thdd * h_
            self.phd += phdd * h_
            self.h += self.hd * h_
            self.th += self.thd * h_
            self.ph += self.phd * h_
            for i in range(4):
                self.wd[i] += wdd[i] * h_
                self.w[i] += self.wd[i] * h_

    def set_equilibrium(self, loads, r) -> None:
        """Static equilibrium for constant loads and road (the model is linear without wheel lift)."""
        zero4 = [0.0] * 4

        def resid(xv):
            st = (xv[0], 0.0, xv[1], 0.0, xv[2], 0.0, list(xv[3:7]), zero4)
            hdd, thdd, phdd, wdd = self._accel(st, loads, r, zero4)
            return np.array([hdd, thdd, phdd] + wdd)

        x0 = np.zeros(7)
        r0 = resid(x0)
        jac = np.empty((7, 7))
        eps = 1e-4
        for j in range(7):
            xp = x0.copy()
            xp[j] = eps
            jac[:, j] = (resid(xp) - r0) / eps
        xs = np.linalg.solve(jac, -r0)
        self.h, self.th, self.ph = float(xs[0]), float(xs[1]), float(xs[2])
        self.w = [float(v) for v in xs[3:7]]
        self.hd = self.thd = self.phd = 0.0
        self.wd = [0.0] * 4

    # ---------------------------------------------------------------- outputs
    def tyre_loads(self, r, rd) -> tuple[list[float], bool]:
        _, t, _, _ = self._forces(self.h, self.hd, self.th, self.thd, self.ph, self.phd, self.w, self.wd, r, rd)
        fz = [self.fz0[i] + t[i] for i in range(4)]
        return fz, any(v <= 1e-6 for v in fz)

    def travel(self, r=None, rd=None) -> tuple[list[float], list[float]]:
        """Suspension compression and its rate per corner."""
        z4 = [0.0] * 4
        _, _, dl, ddl = self._forces(self.h, self.hd, self.th, self.thd, self.ph, self.phd, self.w, self.wd,
                                     r or z4, rd or z4)
        return dl, ddl

    def ride_heights(self, r) -> tuple[float, float]:
        """Floor height above the road at the front / rear axle (m)."""
        c = self.car
        d = [self.h + self.x[i] * self.th + self.y[i] * self.ph for i in range(4)]
        rh_f = c.front.static_ride_height - 0.5 * (d[FL] + d[FR] + r[FL] + r[FR])
        rh_r = c.rear.static_ride_height - 0.5 * (d[RL] + d[RR] + r[RL] + r[RR])
        return rh_f, rh_r


@dataclass
class _Run:
    t: np.ndarray
    s: np.ndarray
    rec: dict[str, np.ndarray]
    lap_time: float
    energy_J: float
    node_excess: np.ndarray
    node_v: np.ndarray

    @property
    def max_excess(self) -> float:
        return float(self.node_excess.max()) if self.node_excess.size else 0.0


class TransientSolver:
    name = "transient"

    def __init__(self, options: TransientOptions | None = None):
        self.opts = options or TransientOptions()
        self.qss = QSSSolver(self.opts.qss)
        self.qss_reference: LapResult | None = None
        self.road: RoadProfile | None = None

    # ------------------------------------------------------------------ main
    def solve(self, vehicle: VehicleModel, track, start: str = "flying",
              reference: LapResult | None = None) -> LapResult:
        o = self.opts
        trk = track.resampled(o.qss.ds)
        qss_raw = reference or self.qss.solve(vehicle, trk, start=start)
        if o.turn_in_length > 0.0:
            trk = trk.smoothed(o.turn_in_length, o.qss.ds)
            ref = self.qss.solve(vehicle, trk, start=start)
        else:
            ref = qss_raw
        self.qss_reference = ref  # QSS lap on the same (smoothed) line and a smooth road
        self.road = RoadProfile.from_spec(o.road, trk.length, periodic=trk.closed, seed=o.road_seed,
                                          scale=o.road_scale)
        qss_time = ref.lap_time
        v_cap = np.full(len(trk.s), np.inf)
        history = []
        run = None
        for it in range(o.iterations + 1):
            run = self._simulate(vehicle, trk, ref, start)
            history.append({"iteration": it, "reference_lap_s": ref.lap_time, "lap_s": run.lap_time,
                            "max_lateral_excess": run.max_excess})
            if run.max_excess <= o.excess_tol or it == o.iterations:
                break
            v_cap = np.minimum(v_cap, self._learned_cap(trk, run, ref))
            ref = self.qss.solve(vehicle, trk, start=start, v_cap=v_cap)
        res = self._to_lap_result(vehicle, trk, run, ref)
        fz = np.stack([run.rec[k] for k in ("fl", "fr", "rl", "rr")])
        res.info = {"qss_lap_time_s": qss_raw.lap_time, "qss_smoothed_lap_time_s": qss_time,
                    "iterations": history, "max_lateral_excess": run.max_excess, "dt_s": o.dt,
                    "max_instant_excess": float(np.max(run.rec["excess"])) if len(run.rec["excess"]) else 0.0,
                    "road": self.road.name if self.road else "flat",
                    "road_rms_mm": self.road.rms() * 1000.0 if self.road else 0.0,
                    "tyre_load_std_N": _detrended_std(run.rec["t"], fz)}
        return res

    def _learned_cap(self, trk, run: "_Run", ref: LapResult) -> np.ndarray:
        """New speed limit where the car got into trouble (tyre over-use, running wide, spin).

        Inside a corner the car is at the lateral limit and cannot brake any more, so the speed
        has to come off before it: the reference is lowered (``learn_gain`` x excess per learning
        lap) from just before the corner entry up to the trouble spot, and the QSS re-plan moves
        the braking point accordingly.
        """
        cap = np.full(len(trk.s), np.inf)
        bad = np.flatnonzero(run.node_excess > self.opts.excess_tol)
        if bad.size == 0:
            return cap
        k_abs = np.abs(trk.kappa)
        ds = trk.ds
        after = max(1, int(round(1.0 / ds)))
        lead = max(1, int(round(3.0 / ds)))
        search = max(1, int(round(30.0 / ds)))
        for i in bad:
            ex = min(run.node_excess[i], 1.0)
            j = i
            if k_abs[j] <= 1e-3:  # trouble on a straight: blame the corner just before it
                while j > 0 and i - j < search and k_abs[j] <= 1e-3:
                    j -= 1
                if k_abs[j] <= 1e-3:
                    j = i
            entry = j
            if k_abs[j] > 1e-3:  # walk back to where that corner starts
                while entry > 0 and j - entry < search and k_abs[entry - 1] >= 0.5 * k_abs[j]:
                    entry -= 1
            lo, hi = max(0, entry - lead), min(len(cap), max(i, j) + after + 1)
            target = ref.v[lo:hi] * max(1.0 - self.opts.learn_gain * ex, 0.5)
            cap[lo:hi] = np.minimum(cap[lo:hi], target)
        return cap

    # ------------------------------------------------------------------ one lap
    def _simulate(self, car: VehicleModel, trk, ref: LapResult, start: str) -> "_Run":
        o = self.opts
        dt = o.dt
        n_sub = max(1, int(math.ceil(dt / o.ride_dt - 1e-9)))
        strat = car.active_strategy()
        active = strat.type != "fixed" and strat.open_mode != strat.closed_mode
        rate_open = 1.0 / strat.open_delay if strat.open_delay > 0 else math.inf
        rate_close = 1.0 / strat.close_delay if strat.close_delay > 0 else math.inf
        s_nodes = trk.s
        s_list = s_nodes.tolist()
        k_list = trk.interval_kappa().tolist()
        n_int = len(k_list)
        length = trk.length
        # reference: constant acceleration per interval (exactly what the QSS integration did)
        v_ref = ref.v
        a_int = ((v_ref[1:] ** 2 - v_ref[:-1] ** 2) / (2.0 * np.diff(s_nodes))).tolist()
        v2_list = (v_ref ** 2).tolist()
        open_ref = _flap_command(ref, strat) if active else np.zeros(len(ref.s))
        crr = car.tire.rolling_resistance
        shift = _ShiftModel(car)
        roll_yaw = car.features.roll_yaw_aero
        road = self.road
        a_ax, b_ax = car.a, car.b  # CG -> front / rear axle along the path

        def road_at(s_cg, v_cg):
            if road is None:
                return [0.0] * 4, [0.0] * 4
            lf, rf_, slf, srf = road.at(s_cg + a_ax)
            lr, rr_, slr, srr = road.at(s_cg - b_ax)
            return [lf, rf_, lr, rr_], [slf * v_cg, srf * v_cg, slr * v_cg, srr * v_cg]

        # initial state: quasi-static equilibrium of the reference at s = 0
        v = 0.0 if start == "standing" else float(ref.v[0])
        mode0 = int(ref.mode[0])
        sgn = 1.0 if trk.interval_kappa()[0] >= 0.0 else -1.0
        st0 = car.solve_state(v, float(ref.ax[0]), abs(float(ref.ay[0])), mode0)
        comp0 = car.load_components(st0.ax, st0.ay, st0.df_f, st0.df_r, st0.drag, st0.roll)
        ride = RideModel(car, o.min_unsprung_kg, o.tyre_damping_ratio)
        r0, _ = road_at(0.0, v)
        ride.set_equilibrium(_ride_loads({k: float(val) for k, val in comp0.items()},
                                         float(st0.df_f), float(st0.df_r), sgn), r0)
        beta = float(st0.yaw) * sgn  # vehicle frame
        lam = float(open_ref[0])
        gear = shift.gear(v)
        shift_left = 0.0
        ax_prev = float(ref.ax[0]) if start != "standing" else 0.0
        jerk_dt = o.max_jerk * dt
        rel_dt = o.max_jerk_release * dt
        half_inv_jerk = 0.5 / o.max_jerk
        ex_filt = 0.0
        fz_eff = None  # tyre loads seen by the grip (relaxation-length lag)
        k_filt = min(1.0, dt / o.excess_filter) if o.excess_filter > 0 else 1.0
        latched = -1
        horizon = 60.0 * half_inv_jerk  # look far enough for a 60 m/s^2 swing
        # lateral / yaw state (vehicle frame, left +) and path errors
        lat_on = o.lateral_dof
        alpha_pk = car.tire.peak_slip
        spin_rad = math.radians(o.spin_sideslip_deg)
        slide_warn = math.radians(o.slide_sideslip_deg)
        k0 = k_list[0]
        vy = v * math.tan(beta) if lat_on else 0.0
        yaw_rate = v * k0
        delta = float(st0.steer) * sgn
        e_y = e_psi = 0.0
        ay_prev = v * v * k0
        fyf = fyr = 0.0

        rec: dict[str, list] = {k: [] for k in (
            "t", "s", "v", "ax", "kappa", "rh_f", "rh_r", "roll", "yaw", "pitch", "clearance", "cla_f",
            "cla_r", "cda", "fl", "fr", "rl", "rr", "cfl", "cfr", "crl", "crr", "u_front", "u_rear",
            "u_power", "excess", "f_drive", "f_brake", "lam", "v_ref", "heave_f", "heave_r",
            "damper_f", "damper_r", "road_f", "road_r", "limit", "lift", "oor", "alpha_f", "alpha_r",
            "steer", "usi", "excess_f", "ay", "e_y", "e_psi", "yaw_rate", "learn", "spin")}
        t = s = 0.0
        idx = 0
        max_steps = int(20.0 * max(ref.lap_time, 1.0) / dt) + 1000
        for _ in range(max_steps):
            if s >= length:
                break
            while idx < n_int - 1 and s >= s_list[idx + 1]:
                idx += 1
            kap = k_list[idx]
            dyn = lat_on and v >= o.v_kinematic
            if dyn:  # outside of the turn from the actual lateral acceleration
                sgn = 1.0 if ay_prev >= 0.0 else -1.0
            else:  # left turn: outside = right
                sgn = 1.0 if kap >= 0.0 else -1.0
            vr = math.sqrt(max(v2_list[idx] + 2.0 * a_int[idx] * (s - s_list[idx]), 0.0))
            # anticipation: start a jerk-limited ramp half a transition before the reference
            # switches, so the ramp is centred on the switch point
            if latched >= 0 and s < s_list[latched]:
                ar = a_int[latched]  # keep ramping towards the target once started
            else:
                latched = -1
                ar = a_int[idx]
                j = idx + 1
                v_eff = max(v, 1.0)
                while j < n_int and s_list[j] - s < v_eff * horizon:
                    if abs(a_int[j] - ax_prev) * half_inv_jerk >= (s_list[j] - s) / v_eff:
                        ar = a_int[j]
                        latched = j
                        break
                    j += 1
            lam_cmd = 1.0 if active and float(np.interp(s, s_nodes, open_ref)) >= 0.5 else 0.0

            # tyre loads, travel and floor height from the ride model (vehicle frame)
            r_now, rd_now = road_at(s, v)
            fz_v, lift = ride.tyre_loads(r_now, rd_now)
            if fz_eff is None or o.relaxation_length <= 0.0:
                fz_eff = list(fz_v)
            else:
                k_rel = min(1.0, max(v, 0.5) * dt / o.relaxation_length)
                fz_eff = [fz_eff[i] + k_rel * (fz_v[i] - fz_eff[i]) for i in range(4)]
            dl, ddl = ride.travel(r_now, rd_now)
            rh_f, rh_r = ride.ride_heights(r_now)
            phi = ride.ph
            roll_out = phi * sgn  # roll towards the outside of the turn
            oi = (FR, FL, RR, RL) if sgn > 0 else (FL, FR, RL, RR)  # vehicle -> (FO, FI, RO, RI)
            fz_oi = np.array([fz_eff[i] for i in oi])[:, None]
            cam_oi, inc_oi = car.corner_camber(np.array([dl[i] for i in oi])[:, None], roll_out)

            # aero at the current (dynamic) attitude and flap position
            cla_f, cla_r, cda, oor = self._aero(car, strat, lam, rh_f, rh_r, phi, beta)
            q = 0.5 * car.rho * v * v
            df_f, df_r, drag = q * cla_f, q * cla_r, q * cda
            ax_coast = -(drag + crr * sum(fz_v)) / car.m

            # lateral tyre forces from the slip angles, or the kinematic demand of the line.
            # Lateral priority: the slip curve uses the full capacity and the longitudinal limiter
            # below only allows the braking / drive that fits into the rest of the friction ellipse
            # (the driver keeps the car pointing where it should go).
            fy_dem: tuple = ()
            cap_f = cap_r = 0.0
            if dyn:
                if car.features.camber:
                    _, fyc = car.tire.capacity(fz_oi[:, 0], inc_oi[:, 0], cam_oi[:, 0])
                else:
                    _, fyc = car.tire.capacity(fz_oi[:, 0])
                cap_f, cap_r = float(fyc[0] + fyc[1]), float(fyc[2] + fyc[3])
                vx = max(v, 0.5)
                alpha_f = delta - (vy + car.a * yaw_rate) / vx
                alpha_r = -(vy - car.b * yaw_rate) / vx
                fyf = cap_f * _lat_shape(alpha_f / alpha_pk)
                fyr = cap_r * _lat_shape(alpha_r / alpha_pk)
                ay_act = (fyf + fyr) / car.m  # small steer angles, as in the QSS yaw balance
                fy_dem = (abs(fyf), abs(fyr))
            else:
                ay_act = v * v * kap
            ay = abs(ay_act)

            # driver command: jerk-limited, then limited by tyres / powertrain at the current loads
            # releasing (towards zero) is faster than applying
            if dyn and kap != 0.0:
                # running wide (outside of the line): the driver lifts / brakes a little
                e_out = -e_y * (1.0 if kap > 0.0 else -1.0)
                vr = max(vr - o.lift_gain * max(e_out - 0.05, 0.0), 0.0)
            lo_j = ax_prev - (rel_dt if ax_prev > 0.0 else jerk_dt)
            hi_j = ax_prev + (rel_dt if ax_prev < 0.0 else jerk_dt)
            ax_cmd = min(max(ar + o.kp * (vr - v), lo_j), hi_j)
            if shift_left > 0.0:
                ax_cmd = min(ax_cmd, max(ax_coast, lo_j))
            if dyn and abs(alpha_r) > alpha_pk:  # rear sliding: no throttle
                ax_cmd = min(ax_cmd, max(ax_coast, lo_j))
            grip = (fz_oi, cam_oi, inc_oi, drag) + fy_dem
            ax, g = self._limit_ax(car, v, ay, ax_cmd, ax_coast, grip)
            if ax < lo_j or ax > hi_j:  # the tyres want a faster change than the driver can make
                ax = min(max(ax, lo_j), hi_j)
                e = self._evaluate(car, v, ay, np.array([ax]), grip)
                g = _pick(e, 0, bool(e["feasible"][0]))
            excess = 0.0 if g["feasible"] else min(max(max(g["u_front"], g["u_rear"]) - 1.0, 0.0), 1.0)
            # sustained over-use (what the driver can learn); short spikes from a rough road are
            # absorbed by tyre slip and only cost speed through the scrub below
            ex_filt += k_filt * (excess - ex_filt)
            ax_prev = ax
            if dyn:  # the slip curve caps the lateral forces; running wide is the consequence
                ax_scrub = 0.0
            else:
                # lateral demand beyond grip: the car slides and scrubs speed (never a gain)
                lat_excess = min(max(g["uy_f"], g["uy_r"]) - 1.0, 1.0)
                ax_scrub = -lat_excess * ay if lat_excess > 0.0 else 0.0

            # record (vehicle frame; roll / yaw positive to the right like the QSS channels)
            rec["t"].append(t)
            rec["s"].append(s)
            rec["v"].append(v)
            rec["ax"].append(ax)
            rec["kappa"].append(kap)
            rec["rh_f"].append(rh_f)
            rec["rh_r"].append(rh_r)
            rec["roll"].append(phi)
            rec["yaw"].append(beta)
            rec["pitch"].append(((rh_r - car.rear.static_ride_height)
                                 - (rh_f - car.front.static_ride_height)) / car.L)
            rec["clearance"].append(min(rh_f, rh_r) - car.floor_half_width * math.sin(abs(phi)))
            rec["cla_f"].append(cla_f)
            rec["cla_r"].append(cla_r)
            rec["cda"].append(cda)
            for k, i in (("fl", FL), ("fr", FR), ("rl", RL), ("rr", RR)):
                rec[k].append(fz_v[i])
            cam_v = [0.0] * 4
            for pos, i in enumerate(oi):
                cam_v[i] = float(cam_oi[pos, 0])
            for k, i in (("cfl", FL), ("cfr", FR), ("crl", RL), ("crr", RR)):
                rec[k].append(cam_v[i])
            for k in ("u_front", "u_rear", "u_power"):
                rec[k].append(g[k])
            if dyn:  # actual slip angles relative to the turn, actual road-wheel steer
                rec["alpha_f"].append(abs(alpha_f))
                rec["alpha_r"].append(abs(alpha_r))
                rec["steer"].append(delta)
            else:
                rec["alpha_f"].append(g["alpha_f"])
                rec["alpha_r"].append(g["alpha_r"])
                rec["steer"].append(g["steer"] * sgn)
            rec["ay"].append(ay_act)
            rec["e_y"].append(e_y)
            rec["e_psi"].append(e_psi)
            rec["yaw_rate"].append(yaw_rate)
            rec["usi"].append(g["understeer_index"])
            rec["excess"].append(excess)
            rec["excess_f"].append(ex_filt)
            rec["f_drive"].append(g["f_drive"])
            rec["f_brake"].append(g["f_brake"])
            rec["lam"].append(lam)
            rec["v_ref"].append(vr)
            rec["heave_f"].append(0.5 * (dl[FL] + dl[FR]))
            rec["heave_r"].append(0.5 * (dl[RL] + dl[RR]))
            rec["damper_f"].append(0.5 * (ddl[FL] + ddl[FR]))
            rec["damper_r"].append(0.5 * (ddl[RL] + ddl[RR]))
            rec["road_f"].append(0.5 * (r_now[FL] + r_now[FR]))
            rec["road_r"].append(0.5 * (r_now[RL] + r_now[RR]))
            rec["limit"].append(int(g["limit"]))
            rec["lift"].append(lift)
            rec["oor"].append(bool(oor))

            # integrate: loads for the chosen acceleration drive the ride model
            comp = car.load_components(ax, ay, df_f, df_r, drag, abs(phi))
            loads = _ride_loads({k: float(val) for k, val in comp.items()}, df_f, df_r, sgn)
            s0, v0 = s, v
            ride.step(loads, lambda tau: road_at(s0 + v0 * tau, v0), 0.0, dt, n_sub)
            spin = 0.0
            if dyn:
                # lateral / yaw dynamics and the motion relative to the line (Frenet frame)
                vy += (ay_act - vx * yaw_rate) * dt
                yaw_rate += (car.a * fyf - car.b * fyr) / car.I_yaw * dt
                s_dot = (vx * math.cos(e_psi) - vy * math.sin(e_psi)) / max(1.0 - kap * e_y, 0.2)
                e_y += (vx * math.sin(e_psi) + vy * math.cos(e_psi)) * dt
                e_psi += (yaw_rate - kap * s_dot) * dt
                v_new = max(v + (ax + vy * yaw_rate) * dt, 0.0)
                slip = math.atan2(vy, vx)
                if abs(slip) > spin_rad:  # spun: lose speed, gather it up again on the line
                    spin = 1.0
                    v_new *= 0.9
                    vy, yaw_rate, e_psi = 0.0, v_new * kap, 0.0
                    slip = 0.0
                e_lim = min(3.0 * o.max_path_error, 0.5 / max(abs(kap), 1e-6))
                e_y = min(max(e_y, -e_lim), e_lim)
                beta = slip if roll_yaw else 0.0
                # steering: feed-forward for the curvature just ahead + preview feedback on the error
                j = idx
                s_ff = s + v * o.steer_preview
                while j < n_int - 1 and s_ff >= s_list[j + 1]:
                    j += 1
                k_ff = k_list[j]
                ay_ss = v * v * abs(k_ff)
                uf_ss = min(car.m * ay_ss * car.b / car.L / max(cap_f, 1.0), 0.999)
                ur_ss = min(car.m * ay_ss * car.a / car.L / max(cap_r, 1.0), 0.999)
                d_ss = float(car.tire.slip_angle(uf_ss)) - float(car.tire.slip_angle(ur_ss))
                l_p = min(max(0.25 * v + 2.0, 2.5), 8.0)
                e_p = e_y + l_p * math.sin(e_psi + slip)
                d_cmd = (car.L * k_ff + math.copysign(d_ss, k_ff) - 2.0 * car.L * e_p / l_p ** 2
                         - o.yaw_rate_gain * (yaw_rate - v * k_ff))  # counter-steer a yaw-rate excess
                # no more lock than the front tyres can use: front slip at most just past the peak
                d_mid = (vy + car.a * yaw_rate) / vx
                d_cmd = min(max(d_cmd, d_mid - 1.05 * alpha_pk), d_mid + 1.05 * alpha_pk)
                d_cmd = min(max(d_cmd, -0.6), 0.6)
                delta += (d_cmd - delta) * min(1.0, dt / o.steer_lag) if o.steer_lag > 0 else d_cmd - delta
                s += s_dot * dt
            else:
                v_new = max(v + (ax + ax_scrub) * dt, 0.0)
                if roll_yaw and o.yaw_lag > 0:
                    beta += dt * (g["beta"] * sgn - beta) / o.yaw_lag
                elif roll_yaw:
                    beta = g["beta"] * sgn
                vy = v_new * math.tan(beta) if lat_on else 0.0
                yaw_rate = v_new * kap
                delta = g["steer"] * sgn
                s += 0.5 * (v + v_new) * dt
            ay_prev = ay_act
            # learning signals (0..1): sustained tyre over-use, running off the line, sliding / spin
            path_ex = 0.5 * min(max(abs(e_y) - o.max_path_error, 0.0) / o.max_path_error, 1.0)
            slide_ex = 0.5 * min(max(abs(beta) - slide_warn, 0.0) / (spin_rad - slide_warn), 1.0)
            rec["spin"].append(spin)
            rec["learn"].append(max(ex_filt, path_ex, slide_ex, 0.5 * spin))
            if lam_cmd > lam:
                lam = min(lam_cmd, lam + dt * rate_open)
            elif lam_cmd < lam:
                lam = max(lam_cmd, lam - dt * rate_close)
            t += dt
            if shift.enabled:
                if shift_left > 0.0:
                    shift_left -= dt
                g_new = shift.gear(v_new)
                if g_new > gear and ax > ax_coast:
                    shift_left = shift.time
                gear = g_new
            v = v_new
        else:
            raise RuntimeError("transient lap did not finish (driver stalled?)")

        rec_a = {k: np.asarray(val) for k, val in rec.items()}
        # close the lap: one final sample at s >= length
        t_hist = np.append(rec_a["t"], t)
        s_hist = np.append(rec_a["s"], s)
        lap_time = float(np.interp(length, s_hist, t_hist))
        node_idx = np.clip(np.searchsorted(s_nodes, rec_a["s"], side="right") - 1, 0, len(s_nodes) - 1)
        node_excess = np.zeros(len(s_nodes))
        np.maximum.at(node_excess, node_idx, rec_a["learn"])
        node_v = np.interp(s_nodes, rec_a["s"], rec_a["v"])
        _, energy = powertrain_channels(car, rec_a["v"], rec_a["ax"], rec_a["f_drive"], np.diff(rec_a["t"]),
                                        rec_a["f_brake"])
        return _Run(t_hist, s_hist, rec_a, lap_time, energy, node_excess, node_v)

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _aero(car: VehicleModel, strat, lam, rh_f, rh_r, roll, yaw):
        if lam <= 0.0 or strat.open_mode == strat.closed_mode:
            c = car.aero_coefficients(strat.closed_mode, rh_f, rh_r, roll, yaw)
            return float(c[0]), float(c[1]), float(c[2]), bool(c[3])
        o = car.aero_coefficients(strat.open_mode, rh_f, rh_r, roll, yaw)
        if lam >= 1.0:
            return float(o[0]), float(o[1]), float(o[2]), bool(o[3])
        c = car.aero_coefficients(strat.closed_mode, rh_f, rh_r, roll, yaw)
        mix = [(1.0 - lam) * float(a) + lam * float(b) for a, b in zip(c[:3], o[:3])]
        return mix[0], mix[1], mix[2], bool(c[3]) or bool(o[3])

    @staticmethod
    def _evaluate(car: VehicleModel, v, ay, ax, grip):
        """Utilisation for candidate accelerations ``ax`` (1-D array) at the current tyre loads."""
        fz, camber, incl, drag, *fy = grip
        g = car.grip_usage(v, ax, ay, fz, camber, incl, drag, *(fy or (None, None)))
        return {"ax": ax, "g": g, "feasible": g["utilization"] <= 1.0}

    def _limit_ax(self, car, v, ay, ax_cmd, ax_coast, grip):
        """Acceleration closest to the driver command that the tyres / powertrain allow.

        One vectorised evaluation of [command, coast, coarse line coast -> command], then one
        refinement of the bracket. Returns ``(ax, picked_values)``.
        """
        n1, n2 = 2 * self.opts.n_search + 4, self.opts.n_search + 6
        line = np.linspace(ax_coast, ax_cmd, n1)
        e = self._evaluate(car, v, ay, np.concatenate([[ax_cmd, ax_coast], line]), grip)
        feas = e["feasible"]
        if feas[0]:
            return ax_cmd, _pick(e, 0, True)
        if not feas[1]:  # lateral grip exceeded: coast and report the excess
            return ax_coast, _pick(e, 1, False)
        k = _feasible_prefix(feas[2:])
        ok, bad = float(line[k]), float(line[min(k + 1, n1 - 1)])
        cand = np.linspace(ok, bad, n2 + 2)[1:-1]
        e2 = self._evaluate(car, v, ay, cand, grip)
        f2 = e2["feasible"]
        if f2[0]:
            j = _feasible_prefix(f2)
            return float(cand[j]), _pick(e2, j, True)
        return ok, _pick(e, 2 + k, True)

    # ------------------------------------------------------------------ result
    def _to_lap_result(self, car: VehicleModel, trk, run: "_Run", ref: LapResult) -> LapResult:
        r = run.rec
        s_n = trk.s
        s_h = r["s"]

        def at(key):
            return np.interp(s_n, s_h, r[key])

        def nearest(key):
            j = np.clip(np.searchsorted(s_h, s_n), 0, len(s_h) - 1)
            return r[key][j]

        v = at("v")
        kappa = trk.kappa
        ch: dict[str, np.ndarray] = {
            "rh_front_mm": at("rh_f") * 1000.0, "rh_rear_mm": at("rh_r") * 1000.0,
            "clearance_mm": at("clearance") * 1000.0, "roll_deg": np.degrees(at("roll")),
            "pitch_deg": np.degrees(at("pitch")), "yaw_deg": np.degrees(at("yaw")),
            "steer_deg": np.degrees(at("steer")),
            "understeer_deg": np.degrees(at("alpha_f") - at("alpha_r")),
            "cla_front": at("cla_f"), "cla_rear": at("cla_r"), "cla": at("cla_f") + at("cla_r"),
            "cda": at("cda"),
        }
        cla = ch["cla"]
        ch["cop_front"] = np.where(np.abs(cla) > 1e-9, ch["cla_front"] / np.where(cla == 0, 1.0, cla), np.nan)
        q = 0.5 * car.rho * v * v
        ch["downforce_N"] = q * cla
        ch["drag_N"] = q * ch["cda"]
        for k in ("fl", "fr", "rl", "rr"):
            ch[f"fz_{k}_N"] = at(k)
        for k, src in (("fl", "cfl"), ("fr", "cfr"), ("rl", "crl"), ("rr", "crr")):
            ch[f"camber_{k}_deg"] = np.degrees(at(src))
        for k in ("u_front", "u_rear", "u_power"):
            ch[k] = at(k)
        ch["understeer_index"] = at("usi")
        ch["drive_force_N"] = at("f_drive")
        ch["brake_force_N"] = at("f_brake")
        pch, _ = powertrain_channels(car, v, at("ax"), ch["drive_force_N"], np.diff(np.interp(s_n, run.s, run.t)),
                                     ch["brake_force_N"])
        ch.update(pch)
        ch["v_ref_kmh"] = at("v_ref") * 3.6
        ch["flap_position"] = at("lam")
        ch["heave_front_mm"] = at("heave_f") * 1000.0
        ch["heave_rear_mm"] = at("heave_r") * 1000.0
        ch["damper_vel_front_mm_s"] = at("damper_f") * 1000.0
        ch["damper_vel_rear_mm_s"] = at("damper_r") * 1000.0
        ch["road_front_mm"] = at("road_f") * 1000.0
        ch["road_rear_mm"] = at("road_r") * 1000.0
        ch["lateral_excess"] = run.node_excess
        ch["path_error_m"] = at("e_y")
        ch["heading_error_deg"] = np.degrees(at("e_psi"))
        ch["yaw_rate_dps"] = np.degrees(at("yaw_rate"))
        node_spin = np.zeros(len(s_n), dtype=bool)
        np.logical_or.at(node_spin, np.clip(np.searchsorted(s_n, s_h, side="right") - 1, 0, len(s_n) - 1),
                         r["spin"] > 0.5)
        ch["spin"] = node_spin
        braking = ch["brake_force_N"] > 1.0
        lim = nearest("limit")
        ch["limit"] = np.array([("braking " if b else "") + LIMIT_NAMES[int(i)] for i, b in zip(lim, braking)],
                               dtype=object)
        ch["shifting"] = np.zeros(len(s_n), dtype=bool)
        # floor contact anywhere between two nodes counts (rough road: short dips)
        node_idx = np.clip(np.searchsorted(s_n, s_h, side="right") - 1, 0, len(s_n) - 1)
        node_min = np.full(len(s_n), np.inf)
        np.minimum.at(node_min, node_idx, r["clearance"])
        ch["bottoming"] = node_min < car.bottoming_clearance
        ch["clearance_mm"] = np.minimum(ch["clearance_mm"], np.where(np.isfinite(node_min), node_min * 1000.0,
                                                                     ch["clearance_mm"]))
        node_lift = np.zeros(len(s_n), dtype=bool)
        np.logical_or.at(node_lift, node_idx, r["lift"])
        ch["wheel_lift"] = node_lift
        ch["aero_out_of_range"] = nearest("oor").astype(bool)
        ch["not_converged"] = np.zeros(len(s_n), dtype=bool)
        strat = car.active_strategy()
        mode = np.where(at("lam") >= 0.5, strat.open_mode, strat.closed_mode).astype(int)
        t = np.interp(s_n, run.s, run.t)
        x, y = trk.xy()
        return LapResult(
            track_name=trk.name, vehicle_name=car.name, level=car.features.label, solver=self.name,
            s=s_n.copy(), v=v, ax=at("ax"), ay=at("ay"), kappa=kappa, t=t, mode=mode,
            mode_names=list(car.mode_names), channels=ch, x=x, y=y, energy_J=run.energy_J,
            default_mode=car.default_mode,
        )


def _ride_loads(comp: dict, df_f: float, df_r: float, sgn: float):
    """Ride-model loads (vehicle frame) from the QSS load components of one state.

    The geometric part of the longitudinal transfer and the roll-centre / unsprung part of the
    lateral transfer go through the links straight into the wheels.
    """
    geo_long_f = (df_f - comp["d_long"]) - comp["el_f"]
    geo_long_r = (df_r + comp["d_long"]) - comp["el_r"]
    gf, gr = comp["lat_geo_f"] * sgn, comp["lat_geo_r"] * sgn  # positive -> right side loaded
    geo = [0.5 * geo_long_f - gf, 0.5 * geo_long_f + gf, 0.5 * geo_long_r - gr, 0.5 * geo_long_r + gr]
    return comp["el_f"], comp["el_r"], comp["m_roll"] * sgn, geo


def _detrended_std(t: np.ndarray, fz: np.ndarray, window_s: float = 0.25) -> float:
    """Standard deviation of the tyre loads around their 0.25 s moving average (road-induced part)."""
    if len(t) < 4:
        return 0.0
    dt = float(np.median(np.diff(t)))
    n = max(1, int(round(window_s / dt)))
    kernel = np.ones(n) / n
    out = []
    for row in fz:
        smooth = np.convolve(row, kernel, mode="same")
        out.append(np.std((row - smooth)[n:-n]) if len(row) > 2 * n else 0.0)
    return float(np.mean(out))


def _flap_command(ref: LapResult, strat) -> np.ndarray:
    """Flap *command* (1 = open) per node from a QSS mode array.

    The QSS ``rule`` strategy already contains the actuator delays (the flap counts as open for
    ``close_delay`` after the command to close, and opens ``open_delay`` after the command to
    open). The transient solver models the actuator itself, so the transitions are moved earlier
    by those delays to recover the command. ``ideal`` mode arrays carry no delays.
    """
    is_open = (ref.mode == strat.open_mode).astype(float)
    if strat.type != "rule":
        return is_open
    cmd = is_open.copy()
    t = ref.t
    changes = np.flatnonzero(np.diff(is_open) != 0) + 1
    for k in changes:
        delay = strat.close_delay if is_open[k] < is_open[k - 1] else strat.open_delay
        if delay <= 0:
            continue
        lo = int(np.searchsorted(t, t[k] - delay))
        cmd[lo:k] = is_open[k]
    return cmd


_G_KEYS = ("uy_f", "uy_r", "u_front", "u_rear", "u_power", "alpha_f", "alpha_r", "steer",
           "understeer_index", "f_drive", "f_brake", "limit", "beta", "ux_f", "ux_r")


def _lat_shape(x: float) -> float:
    """Normalised lateral force vs slip ratio x = alpha / alpha_peak (odd function).

    ``2x / (1 + x^2)`` up to the peak, then a gentle fall to 85 % of the peak when sliding.
    """
    ax = abs(x)
    if ax <= 1.0:
        return 2.0 * x / (1.0 + x * x)
    return math.copysign(1.0 - 0.15 * (1.0 - 1.0 / ax), x)


def _feasible_prefix(feas: np.ndarray) -> int:
    """Index of the last element of the leading run of True values (0 if the first is False)."""
    bad = np.flatnonzero(~feas)
    if bad.size == 0:
        return len(feas) - 1
    return max(int(bad[0]) - 1, 0)


def _pick(ev: dict, k: int, feasible: bool) -> dict:
    """Scalar values of candidate ``k`` of a vectorised evaluation."""
    n = ev["ax"].shape
    out = {"feasible": feasible}
    for key in _G_KEYS:
        out[key] = float(np.broadcast_to(ev["g"][key], n)[k])
    return out
