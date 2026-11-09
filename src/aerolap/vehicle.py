"""Four-wheel vehicle model with suspension-coupled aerodynamics.

All methods are vectorised: ``v``, ``ax``, ``ay`` may be scalars or arrays of any
(broadcast-compatible) shape. ``ay`` is the magnitude of the lateral acceleration;
tyres are labelled by their position relative to the turn (outer / inner).

Physics are written as explicit *state -> forces/loads* functions
(:meth:`VehicleModel.evaluate`). The quasi-steady-state equilibrium
(:meth:`VehicleModel.solve_state`) is one consumer of these functions; a transient
solver can call :meth:`VehicleModel.evaluate` with dynamic ride heights / roll / yaw.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .aero.maps import AeroModel
from .aero.strategy import StrategyConfig
from .environment import density_from_config
from .features import ModelFeatures
from .powertrain import Powertrain
from .suspension import Axle
from .tire import Tire

G = 9.80665
FO, FI, RO, RI = 0, 1, 2, 3  # front-outer, front-inner, rear-outer, rear-inner
LIMIT_NAMES = ("front axle", "rear axle", "power")


@dataclass
class VehicleState:
    """Quasi-static vehicle state (all arrays share one shape; per-tyre arrays have a leading 4)."""

    v: np.ndarray
    ax: np.ndarray
    ay: np.ndarray
    mode: int
    rh_f: np.ndarray  # m, axle ride heights
    rh_r: np.ndarray
    roll: np.ndarray  # rad, body roll towards the outside of the turn
    yaw: np.ndarray  # rad, body sideslip angle (aero yaw)
    pitch: np.ndarray  # rad, nose-down positive, relative to static
    cla_f: np.ndarray
    cla_r: np.ndarray
    cda: np.ndarray
    df_f: np.ndarray  # N
    df_r: np.ndarray
    drag: np.ndarray
    fz: np.ndarray  # (4, ...) N
    camber: np.ndarray  # (4, ...) rad, wheel frame (negative = top towards chassis)
    fx_cap: np.ndarray  # (4, ...)
    fy_cap: np.ndarray  # (4, ...)
    clearance: np.ndarray  # m, minimum floor clearance (front/rear axle, roll included)
    alpha_f: np.ndarray  # rad
    alpha_r: np.ndarray
    steer: np.ndarray  # rad, road-wheel steer angle
    uy_f: np.ndarray  # lateral utilisation of each axle
    uy_r: np.ndarray
    u_front: np.ndarray  # combined utilisation of each axle (<= 1 feasible)
    u_rear: np.ndarray
    u_power: np.ndarray
    utilization: np.ndarray
    limit: np.ndarray  # index into LIMIT_NAMES
    understeer_index: np.ndarray  # uy_f / uy_r (> 1: front saturates first)
    fx_drive: np.ndarray  # N, tractive force demanded (>= 0)
    fx_brake: np.ndarray  # N, braking force demanded (>= 0)
    aero_out_of_range: np.ndarray
    bottoming: np.ndarray
    wheel_lift: np.ndarray
    converged: np.ndarray = field(default=None)  # type: ignore[assignment]

    @property
    def cla(self):
        return self.cla_f + self.cla_r

    @property
    def cop_front(self):
        total = self.cla
        return np.where(np.abs(total) > 1e-9, self.cla_f / np.where(total == 0, 1.0, total), np.nan)

    @property
    def feasible(self):
        return self.utilization <= 1.0


class VehicleModel:
    def __init__(self, cfg: dict, features: ModelFeatures | None = None):
        self.cfg = cfg
        self.features = features or ModelFeatures()
        self.name = str(cfg.get("name", "vehicle"))
        ms = cfg["mass"]
        geo = cfg["geometry"]
        self.m = float(ms["total_kg"])
        self.wd_front = float(ms["weight_dist_front"])
        self.L = float(geo["wheelbase_mm"]) / 1000.0
        self.a = self.L * (1.0 - self.wd_front)  # CG -> front axle
        self.b = self.L * self.wd_front  # CG -> rear axle
        self.h = float(ms["cg_height_mm"]) / 1000.0

        self.tire = Tire.from_config(cfg["tires"])
        r = self.tire.radius
        sus = cfg["suspension"]
        m_uf = float(ms["unsprung_front_kg"])
        m_ur = float(ms["unsprung_rear_kg"])
        self.front = Axle.from_config(sus["front"], float(geo["track_front_mm"]),
                                      self.tire.vertical_rate, m_uf, is_front=True)
        self.rear = Axle.from_config(sus["rear"], float(geo["track_rear_mm"]),
                                     self.tire.vertical_rate, m_ur, is_front=False)
        self.m_uf, self.m_ur = m_uf, m_ur
        self.m_s = self.m - m_uf - m_ur
        if self.m_s <= 0:
            raise ValueError("unsprung mass must be smaller than the total mass")
        self.h_s = (self.m * self.h - (m_uf + m_ur) * r) / self.m_s
        self.a_s = (self.m * self.a - m_ur * self.L) / self.m_s  # sprung CG -> front axle
        self.b_s = self.L - self.a_s
        h_ra = self.front.roll_center + (self.rear.roll_center - self.front.roll_center) * self.a_s / self.L
        self.d_roll = self.h_s - h_ra  # sprung CG height above the roll axis
        self.K_f = self.front.roll_stiffness
        self.K_r = self.rear.roll_stiffness
        k_eff = self.K_f + self.K_r - self.m_s * G * self.d_roll
        if k_eff <= 0:
            raise ValueError("roll stiffness too low: the car is unstable in roll")
        self._k_roll_eff = k_eff

        self.W_f = self.m * G * self.b / self.L
        self.W_r = self.m * G * self.a / self.L

        aero = cfg["aero"]
        self.aero = AeroModel(aero, cfg.get("_base_dir", "."))
        self.h_drag = float(aero["drag_height_mm"]) / 1000.0
        self.floor_half_width = 0.5 * float(aero["floor_width_mm"]) / 1000.0
        self.bottoming_clearance = float(aero["bottoming_clearance_mm"]) / 1000.0
        self.cop_lateral = float(aero["cop_lateral_shift_mm_per_deg"]) / 1000.0 * 180.0 / math.pi  # m/rad
        self.strategy = StrategyConfig.from_config(aero["strategy"], self.aero)

        self.rho = density_from_config(cfg["environment"])
        self.powertrain = Powertrain(cfg["powertrain"], r)
        bias = cfg["brakes"].get("bias_front")
        self.brake_bias = None if bias is None else float(bias)

    # ------------------------------------------------------------------ info
    @property
    def mode_names(self) -> list[str]:
        return self.aero.mode_names

    @property
    def default_mode(self) -> int:
        return self.aero.default_mode

    def active_strategy(self) -> StrategyConfig:
        if self.features.active_aero:
            return self.strategy
        return StrategyConfig.fixed(self.default_mode)

    def ride_frequencies(self) -> tuple[float, float]:
        mf = 0.5 * (self.m_s * self.b_s / self.L)
        mr = 0.5 * (self.m_s * self.a_s / self.L)
        return self.front.ride_frequency(mf), self.rear.ride_frequency(mr)

    def static_aero(self, mode: int | None = None):
        mode = self.default_mode if mode is None else mode
        return self.aero.coefficients(mode, self.front.static_ride_height * 1000.0,
                                      self.rear.static_ride_height * 1000.0, 0.0, 0.0)

    # ------------------------------------------------------------------ physics
    def aero_coefficients(self, mode: int, rh_f, rh_r, roll, yaw):
        """ClA_f, ClA_r, CdA (m^2) for a body attitude, honouring the feature switches."""
        f = self.features
        if not f.heave_pitch_aero:
            rh_f, rh_r = self.front.static_ride_height, self.rear.static_ride_height
        if not f.roll_yaw_aero:
            roll, yaw = 0.0, 0.0
        clamp = self.bottoming_clearance
        cla_f, cla_r, cda, oor = self.aero.coefficients(
            mode, np.maximum(rh_f, clamp) * 1000.0, np.maximum(rh_r, clamp) * 1000.0,
            np.degrees(roll), np.degrees(yaw))
        if not f.aero_balance:
            total = cla_f + cla_r
            cla_f, cla_r = total * self.wd_front, total * (1.0 - self.wd_front)
        return cla_f, cla_r, cda, oor

    def suspension_response(self, ax, ay, df_f, df_r, drag, roll):
        """Tyre loads and the quasi-static body attitude for given forces.

        ``roll`` is only used for the aero lateral CoP shift. Returns a dict with the
        four tyre loads, the new ride heights, roll angle and corner cambers.
        """
        f = self.features
        fr, rr = self.front, self.rear
        L, r = self.L, self.tire.radius
        zero = np.zeros(np.broadcast(ax, ay, df_f, drag).shape)
        if f.load_transfer:
            h_d = self.h_drag if f.drag_height else 0.0
            d_long = (self.m * ax * self.h + drag * h_d) / L  # load moved to the rear axle
            sprung_long = self.m_s * ax * self.h_s / L
            anti_f = np.where(ax < 0, fr.anti_brake, fr.anti_drive)
            anti_r = np.where(ax < 0, rr.anti_brake, rr.anti_drive)
            el_long_f = -sprung_long * (1.0 - anti_f) - drag * h_d / L
            el_long_r = sprung_long * (1.0 - anti_r) + drag * h_d / L
            y_cp = self.cop_lateral * np.abs(roll)
            m_roll = self.m_s * ay * self.d_roll + (df_f + df_r) * y_cp
            roll_new = m_roll / self._k_roll_eff
            dz_f = (self.K_f * roll_new + self.m_s * ay * (self.b_s / L) * fr.roll_center
                    + self.m_uf * ay * r) / fr.track
            dz_r = (self.K_r * roll_new + self.m_s * ay * (self.a_s / L) * rr.roll_center
                    + self.m_ur * ay * r) / rr.track
        else:
            d_long = el_long_f = el_long_r = roll_new = dz_f = dz_r = zero
        axle_f = self.W_f + df_f - d_long
        axle_r = self.W_r + df_r + d_long
        fz = np.stack(np.broadcast_arrays(0.5 * axle_f + dz_f, 0.5 * axle_f - dz_f,
                                          0.5 * axle_r + dz_r, 0.5 * axle_r - dz_r))
        wheel_lift = (fz < 0.0).any(axis=0)
        fz = np.maximum(fz, 0.0)

        # Ride heights: elastic part through the springs, total load change through the tyres.
        heave_f = (df_f + el_long_f) / fr.heave_stiffness
        heave_r = (df_r + el_long_r) / rr.heave_stiffness
        rh_f = fr.static_ride_height - heave_f - (axle_f - self.W_f) / (2.0 * fr.tire_rate)
        rh_r = rr.static_ride_height - heave_r - (axle_r - self.W_r) / (2.0 * rr.tire_rate)

        # Corner bump travel and camber (wheel frame).
        phi_f = self.K_f * roll_new / fr.roll_stiffness_springs
        phi_r = self.K_r * roll_new / rr.roll_stiffness_springs
        z = np.stack(np.broadcast_arrays(heave_f + 0.5 * fr.track * phi_f, heave_f - 0.5 * fr.track * phi_f,
                                         heave_r + 0.5 * rr.track * phi_r, heave_r - 0.5 * rr.track * phi_r))
        cam0 = np.array([fr.static_camber, fr.static_camber, rr.static_camber, rr.static_camber])
        gain = np.array([fr.camber_gain, fr.camber_gain, rr.camber_gain, rr.camber_gain])
        side = np.array([1.0, -1.0, 1.0, -1.0])  # body roll tilts outer wheels positive
        shp = (4,) + (1,) * (z.ndim - 1)
        camber = cam0.reshape(shp) + gain.reshape(shp) * z + side.reshape(shp) * roll_new
        inclination = -side.reshape(shp) * camber  # lean towards the turn centre
        return {
            "fz": fz, "rh_f": rh_f, "rh_r": rh_r, "roll": roll_new, "camber": camber,
            "inclination": inclination, "wheel_lift": wheel_lift,
        }

    def evaluate(self, v, ax, ay, mode: int, rh_f, rh_r, roll, yaw):
        """One explicit evaluation of every subsystem for a given attitude.

        Returns ``(state, new_attitude)`` where ``new_attitude`` is the attitude the
        quasi-static spring balance would settle at for these forces.
        """
        f = self.features
        q = 0.5 * self.rho * v * v
        cla_f, cla_r, cda, oor = self.aero_coefficients(mode, rh_f, rh_r, roll, yaw)
        df_f, df_r, drag = q * cla_f, q * cla_r, q * cda
        sr = self.suspension_response(ax, ay, df_f, df_r, drag, roll)
        fz = sr["fz"]
        if f.camber:
            fx_cap, fy_cap = self.tire.capacity(fz, sr["inclination"], sr["camber"])
        else:
            fx_cap, fy_cap = self.tire.capacity(fz)

        # Lateral demand from steady-state yaw-moment balance.
        fy_f = self.m * ay * self.b / self.L
        fy_r = self.m * ay * self.a / self.L
        cap_yf = fy_cap[FO] + fy_cap[FI]
        cap_yr = fy_cap[RO] + fy_cap[RI]
        uy_f = _safe_ratio(fy_f, cap_yf)
        uy_r = _safe_ratio(fy_r, cap_yr)

        # Sideslip / steer from the tyre slip curve.
        kappa = ay / np.maximum(v, 1.0) ** 2
        alpha_f = self.tire.slip_angle(uy_f)
        alpha_r = self.tire.slip_angle(uy_r)
        beta = self.b * kappa - alpha_r
        steer = self.L * kappa + alpha_f - alpha_r

        # Longitudinal demand and its distribution.
        rr_force = self.tire.rolling_resistance * fz.sum(axis=0)
        fx_req = self.m * ax + drag + rr_force
        f_drive = np.maximum(fx_req, 0.0)
        f_brake = np.maximum(-fx_req, 0.0)
        lat_f = np.sqrt(np.clip(1.0 - uy_f * uy_f, 0.0, 1.0))
        lat_r = np.sqrt(np.clip(1.0 - uy_r * uy_r, 0.0, 1.0))
        min_f = np.minimum(fx_cap[FO], fx_cap[FI])
        min_r = np.minimum(fx_cap[RO], fx_cap[RI])
        sum_f = fx_cap[FO] + fx_cap[FI]
        sum_r = fx_cap[RO] + fx_cap[RI]
        pt = self.powertrain
        den_drive_f = 2.0 * min_f if pt.differential == "open" else sum_f
        den_drive_r = 2.0 * min_r if pt.differential == "open" else sum_r
        if pt.drive == "rwd":
            fd_f, fd_r = 0.0 * f_drive, f_drive
        elif pt.awd_front_fraction is not None:
            fd_f, fd_r = pt.awd_front_fraction * f_drive, (1.0 - pt.awd_front_fraction) * f_drive
        else:
            cf, cr = den_drive_f * lat_f, den_drive_r * lat_r
            share = _safe_ratio(cf, cf + cr, default=0.5)
            fd_f, fd_r = share * f_drive, (1.0 - share) * f_drive
        # Brakes: equal pressure left/right -> equal force per wheel on an axle.
        if self.brake_bias is None:
            cf, cr = 2.0 * min_f * lat_f, 2.0 * min_r * lat_r
            share = _safe_ratio(cf, cf + cr, default=self.wd_front)
        else:
            share = self.brake_bias
        fb_f, fb_r = share * f_brake, (1.0 - share) * f_brake
        ux_f = _safe_ratio(fd_f, den_drive_f) + _safe_ratio(fb_f, 2.0 * min_f)
        ux_r = _safe_ratio(fd_r, den_drive_r) + _safe_ratio(fb_r, 2.0 * min_r)
        u_front = np.sqrt(ux_f * ux_f + uy_f * uy_f)
        u_rear = np.sqrt(ux_r * ux_r + uy_r * uy_r)
        f_power, _ = pt.max_force(v)
        u_power = _safe_ratio(f_drive, f_power)
        stack = np.stack(np.broadcast_arrays(u_front, u_rear, u_power))
        limit = np.argmax(stack, axis=0)
        utilization = stack.max(axis=0)
        with np.errstate(divide="ignore", invalid="ignore"):
            usi = np.where(uy_r > 1e-9, uy_f / np.where(uy_r > 1e-9, uy_r, 1.0), np.nan)

        rh_f, rh_r = np.broadcast_arrays(rh_f, rh_r)
        clearance = np.minimum(rh_f, rh_r) - self.floor_half_width * np.sin(np.abs(roll))
        pitch = ((rh_r - self.rear.static_ride_height) - (rh_f - self.front.static_ride_height)) / self.L
        bshape = utilization.shape
        state = VehicleState(
            v=_b(v, bshape), ax=_b(ax, bshape), ay=_b(ay, bshape), mode=mode,
            rh_f=_b(rh_f, bshape), rh_r=_b(rh_r, bshape), roll=_b(roll, bshape), yaw=_b(yaw, bshape),
            pitch=_b(pitch, bshape), cla_f=_b(cla_f, bshape), cla_r=_b(cla_r, bshape), cda=_b(cda, bshape),
            df_f=_b(df_f, bshape), df_r=_b(df_r, bshape), drag=_b(drag, bshape), fz=fz,
            camber=np.broadcast_to(sr["camber"], fz.shape), fx_cap=fx_cap, fy_cap=fy_cap,
            clearance=_b(clearance, bshape), alpha_f=_b(alpha_f, bshape), alpha_r=_b(alpha_r, bshape),
            steer=_b(steer, bshape), uy_f=_b(uy_f, bshape), uy_r=_b(uy_r, bshape),
            u_front=_b(u_front, bshape), u_rear=_b(u_rear, bshape), u_power=_b(u_power, bshape),
            utilization=utilization, limit=limit, understeer_index=_b(usi, bshape),
            fx_drive=_b(f_drive, bshape), fx_brake=_b(f_brake, bshape), aero_out_of_range=_b(oor, bshape),
            bottoming=_b(clearance < self.bottoming_clearance, bshape), wheel_lift=_b(sr["wheel_lift"], bshape),
        )
        new_attitude = (sr["rh_f"], sr["rh_r"], sr["roll"], beta if f.roll_yaw_aero else 0.0 * beta)
        return state, new_attitude

    def solve_state(self, v, ax, ay, mode: int | None = None, init: VehicleState | None = None,
                    max_iter: int = 40, tol: float = 1e-5, relax: float = 1.0) -> VehicleState:
        """Quasi-static equilibrium: aero <-> loads <-> ride heights / roll / yaw fixed point."""
        mode = self.default_mode if mode is None else mode
        v, ax, ay = (np.array(x, dtype=float) for x in np.broadcast_arrays(
            np.asarray(v, dtype=float), np.asarray(ax, dtype=float), np.abs(np.asarray(ay, dtype=float))))
        shape = v.shape
        if init is not None and init.rh_f.shape == shape:
            rh_f, rh_r, roll, yaw = init.rh_f.copy(), init.rh_r.copy(), init.roll.copy(), init.yaw.copy()
        else:
            rh_f = np.full(shape, self.front.static_ride_height)
            rh_r = np.full(shape, self.rear.static_ride_height)
            roll = np.zeros(shape)
            yaw = np.zeros(shape)
        converged = np.zeros(shape, dtype=bool)
        state = None
        for _ in range(max_iter):
            state, (nf, nr, nroll, nyaw) = self.evaluate(v, ax, ay, mode, rh_f, rh_r, roll, yaw)
            dmax = np.maximum.reduce([np.abs(nf - rh_f), np.abs(nr - rh_r),
                                      np.abs(nroll - roll), np.abs(nyaw - yaw)])
            converged = dmax < tol
            rh_f = rh_f + relax * (nf - rh_f)
            rh_r = rh_r + relax * (nr - rh_r)
            roll = roll + relax * (nroll - roll)
            yaw = yaw + relax * (nyaw - yaw)
            if converged.all():
                break
        state, _ = self.evaluate(v, ax, ay, mode, rh_f, rh_r, roll, yaw)
        state.converged = converged
        return state

    def coast_ax(self, state: VehicleState) -> np.ndarray:
        """Acceleration with zero tyre longitudinal force (drag + rolling resistance only)."""
        rr_force = self.tire.rolling_resistance * state.fz.sum(axis=0)
        return -(state.drag + rr_force) / self.m


def _safe_ratio(num, den, default: float = 0.0):
    num = np.asarray(num, dtype=float)
    den = np.asarray(den, dtype=float)
    with np.errstate(divide="ignore", invalid="ignore"):
        out = np.where(den > 1e-9, num / np.where(den > 1e-9, den, 1.0),
                       np.where(num > 1e-9, np.inf, default))
    return out


def _b(x, shape):
    return np.broadcast_to(np.asarray(x), shape)
