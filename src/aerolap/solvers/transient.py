"""Transient (time-domain) lap solver.

What it adds to the quasi-steady-state solver:

* **body dynamics** - sprung-mass heave, pitch and roll with axle springs, dampers and
  inertia. Ride heights, pitch and roll lag behind the loads and can overshoot, and the aero
  map sees the *dynamic* attitude (e.g. front ride height dipping below the steady value at
  the start of a braking zone);
* **dynamic load transfer split** - tyre loads come from the forces the springs and dampers
  actually transmit; only the geometric (anti / roll-centre / unsprung) part is instantaneous;
* **sideslip lag** - the aero yaw angle relaxes to its steady value with ``yaw_lag``;
* **aero actuator dynamics** - the DRS / active-aero flap moves at a finite rate
  (``open_delay_s`` / ``close_delay_s``) and the coefficients are blended in between;
* **driver** - follows the QSS speed profile (feed-forward acceleration + speed-error gain),
  limited at every step by what the tyres and the powertrain can deliver at the current
  dynamic loads. Where the car cannot hold the line (lateral demand above grip) the speed
  target is lowered there and the lap is driven again (``iterations`` learning laps).

The vehicle follows the track centre line (ay = v^2 kappa); the physics are the same
building blocks the QSS solver uses (``VehicleModel.load_components``, ``tyre_loads``,
``attitude``, ``grip_usage``), so a car that reaches steady state gives the QSS answer.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field

import numpy as np

from ..vehicle import FI, FO, G, LIMIT_NAMES, RI, RO, VehicleModel
from .base import LapResult
from .ggv import SolverOptions
from .qss import QSSSolver, _ShiftModel


@dataclass
class TransientOptions:
    dt: float = 0.005  # s
    kp: float = 8.0  # 1/s, driver speed-error gain
    max_jerk: float = 150.0  # m/s^3, how fast the driver / tyres can change ax
    turn_in_length: float = 3.0  # m, curvature is smoothed over this length (steering transition)
    yaw_lag: float = 0.05  # s, sideslip relaxation time
    iterations: int = 4  # learning laps after the first one
    excess_tol: float = 0.02  # allowed tyre over-utilisation before re-planning
    learn_gain: float = 0.3  # speed-target reduction per unit excess and learning lap
    n_search: int = 10  # candidate accelerations per refinement round
    qss: SolverOptions = field(default_factory=SolverOptions)


class BodyDynamics:
    """Sprung-mass heave h (down +), pitch theta (nose down +) and roll phi (to the outside +).

    Generalised forces are the axle loads the springs would carry in steady state
    (``el_f``, ``el_r`` from :meth:`VehicleModel.load_components`) and the roll moment
    ``m_roll``; the springs and dampers resist them. Spring compression at the axles:
    ``q_f = h + a_s theta``, ``q_r = h - b_s theta``.
    """

    def __init__(self, car: VehicleModel):
        self.car = car
        self.kf, self.kr = car.front.heave_stiffness, car.rear.heave_stiffness
        self.cf, self.cr = car.C_heave_f, car.C_heave_r
        self.a, self.b = car.a_s, car.b_s
        self.h = self.hd = self.th = self.thd = self.phi = self.phid = 0.0

    @property
    def q_f(self) -> float:
        return self.h + self.a * self.th

    @property
    def q_r(self) -> float:
        return self.h - self.b * self.th

    @property
    def qd_f(self) -> float:
        return self.hd + self.a * self.thd

    @property
    def qd_r(self) -> float:
        return self.hd - self.b * self.thd

    def set_equilibrium(self, el_f: float, el_r: float, m_roll: float) -> None:
        q_f, q_r = el_f / self.kf, el_r / self.kr
        self.th = (q_f - q_r) / (self.a + self.b)
        self.h = q_f - self.a * self.th
        self.phi = m_roll / self.car._k_roll_eff
        self.hd = self.thd = self.phid = 0.0

    def spring_forces(self) -> tuple[float, float]:
        return (self.kf * self.q_f + self.cf * self.qd_f, self.kr * self.q_r + self.cr * self.qd_r)

    def roll_moments(self) -> tuple[float, float]:
        c = self.car
        return (c.K_f * self.phi + c.C_roll_f * self.phid, c.K_r * self.phi + c.C_roll_r * self.phid)

    def step(self, el_f: float, el_r: float, m_roll: float, dt: float) -> None:
        """Semi-implicit Euler step."""
        c = self.car
        f_f, f_r = self.spring_forces()
        hdd = (el_f + el_r - f_f - f_r) / c.m_s
        thdd = (self.a * (el_f - f_f) - self.b * (el_r - f_r)) / c.I_pitch
        k_roll = c.K_f + c.K_r - c.m_s * G * c.d_roll
        phidd = (m_roll - k_roll * self.phi - (c.C_roll_f + c.C_roll_r) * self.phid) / c.I_roll
        self.hd += hdd * dt
        self.thd += thdd * dt
        self.phid += phidd * dt
        self.h += self.hd * dt
        self.th += self.thd * dt
        self.phi += self.phid * dt


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

    # ------------------------------------------------------------------ main
    def solve(self, vehicle: VehicleModel, track, start: str = "flying",
              reference: LapResult | None = None) -> LapResult:
        trk = track.resampled(self.opts.qss.ds)
        qss_raw = reference or self.qss.solve(vehicle, trk, start=start)
        if self.opts.turn_in_length > 0.0:
            trk = trk.smoothed(self.opts.turn_in_length, self.opts.qss.ds)
            ref = self.qss.solve(vehicle, trk, start=start)
        else:
            ref = qss_raw
        self.qss_reference = ref  # QSS lap on the same (smoothed) line, for comparison
        qss_time = ref.lap_time
        v_cap = np.full(len(trk.s), np.inf)
        history = []
        run = None
        for it in range(self.opts.iterations + 1):
            run = self._simulate(vehicle, trk, ref, start)
            history.append({"iteration": it, "reference_lap_s": ref.lap_time, "lap_s": run.lap_time,
                            "max_lateral_excess": run.max_excess})
            if run.max_excess <= self.opts.excess_tol or it == self.opts.iterations:
                break
            v_cap = np.minimum(v_cap, self._learned_cap(trk, run, ref))
            ref = self.qss.solve(vehicle, trk, start=start, v_cap=v_cap)
        res = self._to_lap_result(vehicle, trk, run, ref)
        res.info = {"qss_lap_time_s": qss_raw.lap_time, "qss_smoothed_lap_time_s": qss_time,
                    "iterations": history,
                    "max_lateral_excess": run.max_excess, "dt_s": self.opts.dt}
        return res

    def _learned_cap(self, trk, run: _Run, ref: LapResult) -> np.ndarray:
        """New speed limit around the nodes where the tyres were over-used.

        The excess is a mix of over-speed, combined braking + cornering during the jerk-limited
        brake release and the transient load distribution, so the target speed is lowered
        gently (``learn_gain`` x excess per learning lap) and the QSS re-plan moves the braking
        point accordingly.
        """
        cap = np.full(len(trk.s), np.inf)
        bad = np.flatnonzero(run.node_excess > self.opts.excess_tol)
        if bad.size == 0:
            return cap
        width = max(1, int(round(1.0 / trk.ds)))
        for i in bad:
            target = min(ref.v[i], run.node_v[i]) * (1.0 - self.opts.learn_gain * run.node_excess[i])
            lo, hi = max(0, i - width), min(len(cap), i + width + 1)
            cap[lo:hi] = np.minimum(cap[lo:hi], target)
        return cap

    # ------------------------------------------------------------------ one lap
    def _simulate(self, car: VehicleModel, trk, ref: LapResult, start: str) -> _Run:
        o = self.opts
        dt = o.dt
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

        # initial state: quasi-static equilibrium of the reference at s = 0
        v = 0.0 if start == "standing" else float(ref.v[0])
        mode0 = int(ref.mode[0])
        st0 = car.solve_state(v, float(ref.ax[0]), abs(float(ref.ay[0])), mode0)
        comp0 = car.load_components(st0.ax, st0.ay, st0.df_f, st0.df_r, st0.drag, st0.roll)
        body = BodyDynamics(car)
        body.set_equilibrium(float(comp0["el_f"]), float(comp0["el_r"]), float(comp0["m_roll"]))
        rh_f, rh_r = float(st0.rh_f), float(st0.rh_r)
        beta = float(st0.yaw)
        lam = float(open_ref[0])
        gear = shift.gear(v)
        shift_left = 0.0
        ax_prev = float(ref.ax[0]) if start != "standing" else 0.0
        jerk_dt = o.max_jerk * dt
        half_inv_jerk = 0.5 / o.max_jerk
        latched = -1
        horizon = 60.0 * half_inv_jerk  # look far enough for a 60 m/s^2 swing

        rec: dict[str, list] = {k: [] for k in (
            "t", "s", "v", "ax", "kappa", "rh_f", "rh_r", "roll", "yaw", "pitch", "clearance", "cla_f",
            "cla_r", "cda", "fo", "fi", "ro", "ri", "cfo", "cfi", "cro", "cri", "u_front", "u_rear",
            "u_power", "excess", "f_drive", "f_brake", "lam", "v_ref", "heave_f", "heave_r",
            "damper_f", "damper_r", "limit", "lift", "oor", "alpha_f", "alpha_r", "steer", "usi")}
        t = s = 0.0
        idx = 0
        energy = 0.0
        max_steps = int(20.0 * max(ref.lap_time, 1.0) / dt) + 1000
        for _ in range(max_steps):
            if s >= length:
                break
            while idx < n_int - 1 and s >= s_list[idx + 1]:
                idx += 1
            kap = k_list[idx]
            ay = v * v * abs(kap)
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

            # aero at the current (dynamic) attitude and flap position
            cla_f, cla_r, cda, oor = self._aero(car, strat, lam, rh_f, rh_r, body.phi, beta)
            q = 0.5 * car.rho * v * v
            df_f, df_r, drag = q * cla_f, q * cla_r, q * cda
            ax_coast = -(drag + crr * (car.m * G + df_f + df_r)) / car.m

            # driver command: jerk-limited, then limited by tyres / powertrain at the current
            # suspension forces
            lo_j, hi_j = ax_prev - jerk_dt, ax_prev + jerk_dt
            ax_cmd = min(max(ar + o.kp * (vr - v), lo_j), hi_j)
            if shift_left > 0.0:
                ax_cmd = min(ax_cmd, max(ax_coast, lo_j))
            ax, g = self._limit_ax(car, body, v, ay, ax_cmd, ax_coast, df_f, df_r, drag)
            if ax < lo_j or ax > hi_j:  # the tyres want a faster change than the driver can make
                ax = min(max(ax, lo_j), hi_j)
                e = self._evaluate(car, body, v, ay, np.array([ax]), df_f, df_r, drag)
                g = _pick(e, 0, bool(e["feasible"][0]))
            excess = 0.0 if g["feasible"] else max(max(g["u_front"], g["u_rear"]) - 1.0, 0.0)
            ax_prev = ax
            # lateral demand beyond grip: the car slides and scrubs speed (never a gain)
            lat_excess = max(g["uy_f"], g["uy_r"]) - 1.0
            ax_scrub = -lat_excess * ay if lat_excess > 0.0 else 0.0

            # record
            fz, cam = g["fz"], g["camber"]
            sgn = 1.0 if kap >= 0.0 else -1.0
            rec["t"].append(t)
            rec["s"].append(s)
            rec["v"].append(v)
            rec["ax"].append(ax)
            rec["kappa"].append(kap)
            rec["rh_f"].append(rh_f)
            rec["rh_r"].append(rh_r)
            rec["roll"].append(body.phi * sgn)
            rec["yaw"].append(beta * sgn)
            rec["pitch"].append(((rh_r - car.rear.static_ride_height)
                                 - (rh_f - car.front.static_ride_height)) / car.L)
            rec["clearance"].append(min(rh_f, rh_r) - car.floor_half_width * math.sin(abs(body.phi)))
            rec["cla_f"].append(cla_f)
            rec["cla_r"].append(cla_r)
            rec["cda"].append(cda)
            for k, j in (("fo", FO), ("fi", FI), ("ro", RO), ("ri", RI)):
                rec[k].append(fz[j])
            for k, j in (("cfo", FO), ("cfi", FI), ("cro", RO), ("cri", RI)):
                rec[k].append(cam[j])
            for k in ("u_front", "u_rear", "u_power", "alpha_f", "alpha_r", "steer"):
                rec[k].append(g[k])
            rec["usi"].append(g["understeer_index"])
            rec["excess"].append(excess)
            rec["f_drive"].append(g["f_drive"])
            rec["f_brake"].append(g["f_brake"])
            rec["lam"].append(lam)
            rec["v_ref"].append(vr)
            rec["heave_f"].append(body.q_f)
            rec["heave_r"].append(body.q_r)
            rec["damper_f"].append(body.qd_f)
            rec["damper_r"].append(body.qd_r)
            rec["limit"].append(int(g["limit"]))
            rec["lift"].append(g["lift"])
            rec["oor"].append(bool(oor))
            energy += g["f_drive"] * v * dt

            # integrate
            body.step(g["el_f"], g["el_r"], g["m_roll"], dt)
            if roll_yaw and o.yaw_lag > 0:
                beta += dt * (g["beta"] - beta) / o.yaw_lag
            elif roll_yaw:
                beta = g["beta"]
            if lam_cmd > lam:
                lam = min(lam_cmd, lam + dt * rate_open)
            elif lam_cmd < lam:
                lam = max(lam_cmd, lam - dt * rate_close)
            rh_f, rh_r = g["rh_f"], g["rh_r"]
            # the attitude above used the spring compression before the step; refresh it
            rh_f -= body.q_f - rec["heave_f"][-1]
            rh_r -= body.q_r - rec["heave_r"][-1]
            v_new = max(v + (ax + ax_scrub) * dt, 0.0)
            s += 0.5 * (v + v_new) * dt
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
        t_hist, s_hist = rec_a["t"], rec_a["s"]
        s_hist = np.append(s_hist, s)
        t_hist = np.append(t_hist, t)
        lap_time = float(np.interp(length, s_hist, t_hist))
        node_idx = np.clip(np.searchsorted(s_nodes, rec_a["s"], side="right") - 1, 0, len(s_nodes) - 1)
        node_excess = np.zeros(len(s_nodes))
        np.maximum.at(node_excess, node_idx, rec_a["excess"])
        node_v = np.interp(s_nodes, rec_a["s"], rec_a["v"])
        rec_a["t_end"] = t_hist
        rec_a["s_end"] = s_hist
        return _Run(t_hist, s_hist, rec_a, lap_time, energy / car.powertrain.efficiency, node_excess, node_v)

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
    def _evaluate(car: VehicleModel, body: BodyDynamics, v, ay, ax, df_f, df_r, drag):
        """Utilisation for candidate accelerations ``ax`` (1-D array) with the body state frozen."""
        comp = car.load_components(ax, ay, df_f, df_r, drag, body.phi)
        f_f, f_r = body.spring_forces()
        m_f, m_r = body.roll_moments()
        fz, lift, axle_f, axle_r = car.tyre_loads(df_f, df_r, comp, f_f, f_r, m_f, m_r)
        rh_f, rh_r, camber, incl = car.attitude(body.q_f, body.q_r, axle_f, axle_r, body.phi)
        g = car.grip_usage(v, ax, ay, fz, camber, incl, drag)
        return {"ax": ax, "comp": comp, "fz": fz, "lift": lift, "rh_f": rh_f, "rh_r": rh_r,
                "camber": camber, "g": g, "feasible": g["utilization"] <= 1.0}

    def _limit_ax(self, car, body, v, ay, ax_cmd, ax_coast, df_f, df_r, drag):
        """Acceleration closest to the driver command that the tyres / powertrain allow.

        One vectorised evaluation of [command, coast, coarse line coast -> command], then one
        refinement of the bracket. Returns ``(ax, picked_values)``.
        """
        n1, n2 = 2 * self.opts.n_search + 4, self.opts.n_search + 6
        line = np.linspace(ax_coast, ax_cmd, n1)
        e = self._evaluate(car, body, v, ay, np.concatenate([[ax_cmd, ax_coast], line]), df_f, df_r, drag)
        feas = e["feasible"]
        if feas[0]:
            return ax_cmd, _pick(e, 0, True)
        if not feas[1]:  # lateral grip exceeded: coast and report the excess
            return ax_coast, _pick(e, 1, False)
        k = _feasible_prefix(feas[2:])
        ok, bad = float(line[k]), float(line[min(k + 1, n1 - 1)])
        cand = np.linspace(ok, bad, n2 + 2)[1:-1]
        e2 = self._evaluate(car, body, v, ay, cand, df_f, df_r, drag)
        f2 = e2["feasible"]
        if f2[0]:
            j = _feasible_prefix(f2)
            return float(cand[j]), _pick(e2, j, True)
        return ok, _pick(e, 2 + k, True)

    # ------------------------------------------------------------------ result
    def _to_lap_result(self, car: VehicleModel, trk, run: _Run, ref: LapResult) -> LapResult:
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
        left = kappa >= 0.0
        ch: dict[str, np.ndarray] = {
            "rh_front_mm": at("rh_f") * 1000.0, "rh_rear_mm": at("rh_r") * 1000.0,
            "clearance_mm": at("clearance") * 1000.0, "roll_deg": np.degrees(at("roll")),
            "pitch_deg": np.degrees(at("pitch")), "yaw_deg": np.degrees(at("yaw")),
            "steer_deg": np.degrees(at("steer")) * np.where(left, 1.0, -1.0),
            "understeer_deg": np.degrees(at("alpha_f") - at("alpha_r")),
            "cla_front": at("cla_f"), "cla_rear": at("cla_r"), "cla": at("cla_f") + at("cla_r"),
            "cda": at("cda"),
        }
        cla = ch["cla"]
        ch["cop_front"] = np.where(np.abs(cla) > 1e-9, ch["cla_front"] / np.where(cla == 0, 1.0, cla), np.nan)
        q = 0.5 * car.rho * v * v
        ch["downforce_N"] = q * cla
        ch["drag_N"] = q * ch["cda"]
        fo, fi, ro, ri = at("fo"), at("fi"), at("ro"), at("ri")
        ch["fz_fl_N"] = np.where(left, fi, fo)
        ch["fz_fr_N"] = np.where(left, fo, fi)
        ch["fz_rl_N"] = np.where(left, ri, ro)
        ch["fz_rr_N"] = np.where(left, ro, ri)
        cfo, cfi, cro, cri = (np.degrees(at(k)) for k in ("cfo", "cfi", "cro", "cri"))
        ch["camber_fl_deg"] = np.where(left, cfi, cfo)
        ch["camber_fr_deg"] = np.where(left, cfo, cfi)
        ch["camber_rl_deg"] = np.where(left, cri, cro)
        ch["camber_rr_deg"] = np.where(left, cro, cri)
        for k in ("u_front", "u_rear", "u_power"):
            ch[k] = at(k)
        ch["understeer_index"] = at("usi")
        ch["drive_force_N"] = at("f_drive")
        ch["brake_force_N"] = at("f_brake")
        ch["drive_power_kW"] = ch["drive_force_N"] * v / 1000.0
        ch["v_ref_kmh"] = at("v_ref") * 3.6
        ch["flap_position"] = at("lam")
        ch["heave_front_mm"] = at("heave_f") * 1000.0
        ch["heave_rear_mm"] = at("heave_r") * 1000.0
        ch["damper_vel_front_mm_s"] = at("damper_f") * 1000.0
        ch["damper_vel_rear_mm_s"] = at("damper_r") * 1000.0
        ch["lateral_excess"] = run.node_excess
        braking = ch["brake_force_N"] > 1.0
        lim = nearest("limit")
        ch["limit"] = np.array([("braking " if b else "") + LIMIT_NAMES[int(i)] for i, b in zip(lim, braking)],
                               dtype=object)
        ch["shifting"] = np.zeros(len(s_n), dtype=bool)
        ch["bottoming"] = ch["clearance_mm"] < car.bottoming_clearance * 1000.0
        ch["aero_out_of_range"] = nearest("oor").astype(bool)
        ch["wheel_lift"] = nearest("lift").astype(bool)
        ch["not_converged"] = np.zeros(len(s_n), dtype=bool)
        strat = car.active_strategy()
        mode = np.where(at("lam") >= 0.5, strat.open_mode, strat.closed_mode).astype(int)
        t = np.interp(s_n, run.s, run.t)
        x, y = trk.xy()
        return LapResult(
            track_name=trk.name, vehicle_name=car.name, level=car.features.label, solver=self.name,
            s=s_n.copy(), v=v, ax=at("ax"), ay=v * v * kappa, kappa=kappa, t=t, mode=mode,
            mode_names=list(car.mode_names), channels=ch, x=x, y=y, energy_J=run.energy_J,
            default_mode=car.default_mode,
        )


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
           "understeer_index", "f_drive", "f_brake", "limit", "beta")


def _feasible_prefix(feas: np.ndarray) -> int:
    """Index of the last element of the leading run of True values (0 if the first is False)."""
    bad = np.flatnonzero(~feas)
    if bad.size == 0:
        return len(feas) - 1
    return max(int(bad[0]) - 1, 0)


def _pick(ev: dict, k: int, feasible: bool) -> dict:
    """Scalar values of candidate ``k`` of a vectorised evaluation."""
    n = ev["ax"].shape

    def one(val):
        return float(np.broadcast_to(val, n)[k])

    out = {"feasible": feasible, "rh_f": one(ev["rh_f"]), "rh_r": one(ev["rh_r"]),
           "lift": bool(np.broadcast_to(ev["lift"], n)[k])}
    out["el_f"], out["el_r"], out["m_roll"] = (one(ev["comp"][key]) for key in ("el_f", "el_r", "m_roll"))
    fz = np.broadcast_to(ev["fz"], (4,) + n)[:, k]
    cam = np.broadcast_to(ev["camber"], (4,) + n)[:, k]
    out["fz"] = [float(x) for x in fz]
    out["camber"] = [float(x) for x in cam]
    for key in _G_KEYS:
        out[key] = one(ev["g"][key])
    return out
