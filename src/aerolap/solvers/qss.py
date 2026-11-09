"""Quasi-steady-state lap solver (apex limits + forward/backward integration)."""

from __future__ import annotations

import math

import numpy as np

from ..aero.strategy import StrategyConfig
from ..vehicle import LIMIT_NAMES, VehicleModel
from .base import LapResult, powertrain_channels
from .ggv import GGV, SolverOptions, compute_ggv


class QSSSolver:
    name = "qss"

    def __init__(self, options: SolverOptions | None = None):
        self.opts = options or SolverOptions()
        self._cache: dict[tuple[int, int], GGV] = {}

    # ------------------------------------------------------------------ envelope
    def ggv(self, vehicle: VehicleModel, mode: int) -> GGV:
        key = (id(vehicle), mode)
        if key not in self._cache:
            self._cache[key] = compute_ggv(vehicle, mode, self.opts)
        return self._cache[key]

    # ------------------------------------------------------------------ main
    def solve(self, vehicle: VehicleModel, track, start: str = "flying",
              v_cap: np.ndarray | None = None) -> LapResult:
        """Quasi-steady-state lap.

        ``v_cap`` (optional, one value per node of ``track.resampled(options.ds)``) is an extra
        speed limit, e.g. from the transient driver's learning laps.
        """
        if start not in ("flying", "standing"):
            raise ValueError("start must be 'flying' or 'standing'")
        trk = track.resampled(self.opts.ds)
        n = trk.n_intervals
        strat = vehicle.active_strategy()
        modes_needed = sorted({strat.closed_mode, strat.open_mode})
        ggvs = {m: self.ggv(vehicle, m) for m in modes_needed}

        ds = np.diff(trk.s)
        k_int = trk.interval_kappa()
        periodic = trk.closed and start == "flying"
        cap = np.full(len(trk.s), np.inf) if v_cap is None else np.asarray(v_cap, dtype=float)
        if cap.shape != trk.s.shape:
            raise ValueError("v_cap must have one value per track node")
        if periodic:  # 3 laps, keep the middle one
            cap = np.concatenate([cap[:-1], cap[:-1], cap])
            k_node = np.concatenate([trk.kappa[:-1], trk.kappa[:-1], trk.kappa])
            k_int = np.concatenate([k_int, k_int, k_int])
            ds = np.concatenate([ds, ds, ds])
            lo, hi = n, 2 * n
        else:
            k_node = trk.kappa.copy()
            lo, hi = 0, n
        s_all = np.concatenate([[0.0], np.cumsum(ds)])
        v0 = 0.0 if start == "standing" else None
        geo = _Geometry(s_all, ds, k_node, k_int, cap)

        shift = _ShiftModel(vehicle)
        if strat.type == "fixed":
            modes = np.full(k_node.shape, strat.closed_mode, dtype=int)
            prof = self._profile(ggvs, geo, modes, v0, shift)
        elif strat.type == "ideal":
            prof = self._profile(ggvs, geo, None, v0, shift)
        else:
            prof = self._rule_profile(ggvs, geo, v0, shift, strat)

        sl = slice(lo, hi + 1)
        ax = _node_ax(prof["v"], geo)
        return self._postprocess(vehicle, trk, prof["v"][sl], ax[sl], prof["modes"][sl], k_node[sl],
                                 prof["shifting"][sl])

    # ------------------------------------------------------------------ profiles
    def _profile(self, ggvs: dict[int, GGV], geo: "_Geometry", modes, v0, shift):
        """Speed profile for a given mode array (``None`` = ideal mode choice).

        Interval ``i`` (node i -> i+1) uses its own curvature ``k_int[i]`` and the aero
        mode of node ``i``; node speeds are capped by the apex limit of ``k_node``.
        """
        k_abs = np.abs(geo.k_node)
        n_nodes = len(k_abs)
        mode_keys = list(ggvs)
        apex_all = np.stack([ggvs[m].apex_speed(k_abs) for m in mode_keys])
        if modes is None:
            best = np.argmax(apex_all, axis=0)
            v_lim = apex_all[best, np.arange(n_nodes)]
            apex_mode = np.asarray(mode_keys)[best]
            cand = [ggvs[m] for m in mode_keys]
            node_ggv = None
        else:
            pos = {m: i for i, m in enumerate(mode_keys)}
            v_lim = apex_all[[pos[m] for m in modes], np.arange(n_nodes)]
            apex_mode = modes
            node_ggv = [ggvs[m] for m in modes]
            cand = None
        v_lim = np.minimum(v_lim, geo.v_cap)
        k_list = np.abs(geo.k_int).tolist()
        ds_list = geo.ds.tolist()
        lim = v_lim.tolist()

        def acc(i, v):
            ay = v * v * k_list[i]
            if node_ggv is not None:
                g = node_ggv[i]
                return g.acc_at(v, ay), g.mode
            best_a, best_m = -1e9, cand[0].mode
            for g in cand:
                a = g.acc_at(v, ay)
                if a > best_a:
                    best_a, best_m = a, g.mode
            return best_a, best_m

        def coast(i, v):
            g = node_ggv[i] if node_ggv is not None else cand[0]
            return g.coast_at(v, v * v * k_list[i])

        def brk(i, v):
            ay = v * v * k_list[i]
            if node_ggv is not None:
                g = node_ggv[i]
                return g.brk_at(v, ay), g.mode
            best_a, best_m = 1e9, cand[0].mode
            for g in cand:
                a = g.brk_at(v, ay)
                if a < best_a:
                    best_a, best_m = a, g.mode
            return best_a, best_m

        # ---- forward (acceleration) pass, Heun integration in v^2
        v_f = [0.0] * n_nodes
        acc_mode = [int(apex_mode[0])] * n_nodes
        shifting = [False] * n_nodes
        v_f[0] = lim[0] if v0 is None else min(lim[0], v0)
        gear = shift.gear(v_f[0])
        t_left = 0.0
        for i in range(n_nodes - 1):
            vi = v_f[i]
            d = ds_list[i]
            if t_left > 0.0:
                a1 = coast(i, vi)
                shifting[i] = True
                m1 = acc_mode[i - 1] if i > 0 else acc_mode[0]
            else:
                a1, m1 = acc(i, vi)
            acc_mode[i] = m1
            vp = math.sqrt(max(vi * vi + 2.0 * a1 * d, 1e-6))
            a2 = coast(i, vp) if t_left > 0.0 else acc(i, vp)[0]
            vn = math.sqrt(max(vi * vi + (a1 + a2) * d, 1e-6))
            if vn > lim[i + 1]:
                vn = lim[i + 1]
            if t_left > 0.0:
                t_left -= 2.0 * d / (vi + vn)
            v_f[i + 1] = vn
            if shift.enabled:
                g_new = shift.gear(vn)
                if g_new > gear and vn < lim[i + 1] - 1e-9:
                    t_left = shift.time
                gear = g_new
        acc_mode[-1] = acc_mode[-2] if n_nodes > 1 else acc_mode[0]

        # ---- backward (braking) pass
        v_b = list(v_f)
        brk_mode = list(acc_mode)
        for i in range(n_nodes - 2, -1, -1):
            vn = v_b[i + 1]
            d = ds_list[i]
            d1, m1 = brk(i, vn)
            vp = math.sqrt(max(vn * vn - 2.0 * d1 * d, 0.0))
            d2, _ = brk(i, vp)
            vb = math.sqrt(max(vn * vn - (d1 + d2) * d, 0.0))
            brk_mode[i] = m1
            if vb < v_b[i]:
                v_b[i] = vb

        v = np.asarray(v_b)
        v_fwd = np.asarray(v_f)
        braking = v < v_fwd - 1e-6
        apex = (~braking) & (np.abs(v - v_lim) < 1e-6) & (k_abs > 1e-9)
        if modes is None:
            mode_arr = np.where(braking, brk_mode, np.where(apex, apex_mode, acc_mode)).astype(int)
        else:
            mode_arr = np.asarray(modes, dtype=int)
        return {"v": v, "v_fwd": v_fwd, "v_lim": v_lim, "modes": mode_arr, "braking": braking,
                "apex": apex, "shifting": np.asarray(shifting)}

    def _rule_profile(self, ggvs, geo: "_Geometry", v0, shift, strat: StrategyConfig):
        cand = strat.candidate_mask(geo.k_node, geo.s)
        modes = np.where(cand, strat.open_mode, strat.closed_mode).astype(int)
        prof = self._profile(ggvs, geo, modes, v0, shift)
        seen = [modes]
        for _ in range(strat.max_iterations):
            new = strat.rule_modes(cand, prof["v"], prof["braking"], prof["apex"], geo.s)
            if any(np.array_equal(new, m) for m in seen):
                if not np.array_equal(new, modes):
                    prof = self._profile(ggvs, geo, new, v0, shift)
                break
            seen.append(new)
            modes = new
            prof = self._profile(ggvs, geo, modes, v0, shift)
        return prof

    # ------------------------------------------------------------------ post
    def _postprocess(self, vehicle: VehicleModel, trk, v, ax, modes, kappa, shifting) -> LapResult:
        ds = np.diff(trk.s)
        v = np.asarray(v, dtype=float)
        v2 = v * v
        ay_signed = v2 * kappa
        dt = 2.0 * ds / np.maximum(v[1:] + v[:-1], 1e-6)
        t = np.concatenate([[0.0], np.cumsum(dt)])

        n = len(v)
        ch: dict[str, np.ndarray] = {}
        names = ["rh_front_mm", "rh_rear_mm", "clearance_mm", "roll_deg", "pitch_deg", "yaw_deg",
                 "steer_deg", "understeer_deg", "cla_front", "cla_rear", "cla", "cda", "cop_front",
                 "downforce_N", "drag_N", "fz_fl_N", "fz_fr_N", "fz_rl_N", "fz_rr_N",
                 "camber_fl_deg", "camber_fr_deg", "camber_rl_deg", "camber_rr_deg",
                 "u_front", "u_rear", "u_power", "understeer_index", "drive_force_N", "brake_force_N",
                 "drive_power_kW"]
        for k in names:
            ch[k] = np.full(n, np.nan)
        limit = np.zeros(n, dtype=int)
        flags = {k: np.zeros(n, dtype=bool) for k in ("bottoming", "aero_out_of_range", "wheel_lift",
                                                      "not_converged")}
        left = kappa >= 0.0  # left turn -> right-hand wheels are on the outside
        for m in np.unique(modes):
            sel = modes == m
            st = vehicle.solve_state(v[sel], ax[sel], np.abs(ay_signed[sel]), int(m))
            lf = left[sel]
            ch["rh_front_mm"][sel] = st.rh_f * 1000.0
            ch["rh_rear_mm"][sel] = st.rh_r * 1000.0
            ch["clearance_mm"][sel] = st.clearance * 1000.0
            ch["roll_deg"][sel] = np.degrees(st.roll) * np.where(lf, 1.0, -1.0)
            ch["pitch_deg"][sel] = np.degrees(st.pitch)
            ch["yaw_deg"][sel] = np.degrees(st.yaw) * np.where(lf, 1.0, -1.0)
            ch["steer_deg"][sel] = np.degrees(st.steer) * np.where(lf, 1.0, -1.0)
            ch["understeer_deg"][sel] = np.degrees(st.alpha_f - st.alpha_r)
            ch["cla_front"][sel] = st.cla_f
            ch["cla_rear"][sel] = st.cla_r
            ch["cla"][sel] = st.cla
            ch["cda"][sel] = st.cda
            ch["cop_front"][sel] = st.cop_front
            ch["downforce_N"][sel] = st.df_f + st.df_r
            ch["drag_N"][sel] = st.drag
            fo, fi, ro, ri = st.fz
            ch["fz_fl_N"][sel] = np.where(lf, fi, fo)
            ch["fz_fr_N"][sel] = np.where(lf, fo, fi)
            ch["fz_rl_N"][sel] = np.where(lf, ri, ro)
            ch["fz_rr_N"][sel] = np.where(lf, ro, ri)
            cfo, cfi, cro, cri = np.degrees(st.camber)
            ch["camber_fl_deg"][sel] = np.where(lf, cfi, cfo)
            ch["camber_fr_deg"][sel] = np.where(lf, cfo, cfi)
            ch["camber_rl_deg"][sel] = np.where(lf, cri, cro)
            ch["camber_rr_deg"][sel] = np.where(lf, cro, cri)
            ch["u_front"][sel] = st.u_front
            ch["u_rear"][sel] = st.u_rear
            ch["u_power"][sel] = st.u_power
            ch["understeer_index"][sel] = st.understeer_index
            ch["drive_force_N"][sel] = st.fx_drive
            ch["brake_force_N"][sel] = st.fx_brake
            ch["drive_power_kW"][sel] = st.fx_drive * v[sel] / 1000.0
            limit[sel] = st.limit
            flags["bottoming"][sel] = st.bottoming
            flags["aero_out_of_range"][sel] = st.aero_out_of_range
            flags["wheel_lift"][sel] = st.wheel_lift
            flags["not_converged"][sel] = ~st.converged
        braking = ch["brake_force_N"] > 1.0
        ch["limit"] = np.array([("braking " if b else "") + LIMIT_NAMES[i] for i, b in zip(limit, braking)],
                               dtype=object)
        if vehicle.powertrain.type == "ice":
            _, gear = vehicle.powertrain.max_force(v)
            ch["gear"] = gear + 1
        ch["shifting"] = np.asarray(shifting, dtype=bool)
        for k, val in flags.items():
            ch[k] = val
        pch, energy = powertrain_channels(vehicle, v, ax, ch["drive_force_N"], dt)
        ch.update(pch)
        x, y = trk.xy()
        return LapResult(
            track_name=trk.name, vehicle_name=vehicle.name, level=vehicle.features.label,
            solver=self.name, s=trk.s.copy(), v=v, ax=ax, ay=ay_signed, kappa=kappa, t=t,
            mode=np.asarray(modes, dtype=int), mode_names=list(vehicle.mode_names), channels=ch,
            x=x, y=y, energy_J=energy, default_mode=vehicle.default_mode,
        )


def _node_ax(v: np.ndarray, geo: "_Geometry") -> np.ndarray:
    """Longitudinal acceleration at the nodes.

    Inside a constant-curvature section it is the mean of the two adjacent intervals; at a
    segment boundary it is taken from the interval whose curvature defines the node (the
    tighter one), so a corner-exit node is not given the straight's acceleration on top of
    the full cornering load.
    """
    v2 = v * v
    ax_int = (v2[1:] - v2[:-1]) / (2.0 * geo.ds)
    before = np.concatenate([ax_int[:1], ax_int])
    after = np.concatenate([ax_int, ax_int[-1:]])
    kb = np.abs(np.concatenate([geo.k_int[:1], geo.k_int]))
    ka = np.abs(np.concatenate([geo.k_int, geo.k_int[-1:]]))
    tol = 1e-9
    return np.where(kb > ka + tol, before, np.where(ka > kb + tol, after, 0.5 * (before + after)))


class _Geometry:
    """Node positions / curvature and interval lengths / curvature of the (tiled) track."""

    def __init__(self, s, ds, k_node, k_int, v_cap):
        self.s = s
        self.ds = ds
        self.k_node = k_node
        self.k_int = k_int
        self.v_cap = v_cap


class _ShiftModel:
    """Torque interruption during up-shifts (ICE only)."""

    def __init__(self, vehicle: VehicleModel):
        pt = vehicle.powertrain
        self.enabled = pt.type == "ice" and pt.shift_time > 0.0 and len(pt.ratios) > 1
        self.time = pt.shift_time
        if self.enabled:
            self._step, self._gears = pt.gear_lookup(pt.v_max + 5.0)

    def gear(self, v: float) -> int:
        if not self.enabled:
            return 0
        i = int(v / self._step)
        return self._gears[min(max(i, 0), len(self._gears) - 1)]
