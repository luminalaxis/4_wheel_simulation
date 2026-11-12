"""Matplotlib figures (saved as PNG; English labels so no CJK font is needed)."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

G = 9.80665


def _save(fig, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return path


def _shade_modes(ax, res):
    if len(res.mode_names) < 2:
        return
    other = res.mode != res.default_mode
    if not other.any():
        return
    ax.fill_between(res.s, 0, 1, where=other, transform=ax.get_xaxis_transform(), color="tab:green",
                    alpha=0.12, lw=0, label=f"{'/'.join(n for i, n in enumerate(res.mode_names) if i != res.default_mode)} open")


def plot_lap(res, path):
    ch = res.channels
    fig, axs = plt.subplots(6, 1, figsize=(11, 15), sharex=True)
    s = res.s
    ax = axs[0]
    ax.plot(s, res.v * 3.6, color="k")
    _shade_modes(ax, res)
    ax.set_ylabel("speed [km/h]")
    ax.set_title(f"{res.vehicle_name} - {res.track_name} - {res.level}  lap {res.lap_time:.3f} s")
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc="lower right")
    ax = axs[1]
    ax.plot(s, res.ax / G, label="ax")
    ax.plot(s, res.ay / G, label="ay")
    ax.set_ylabel("accel [g]")
    ax.legend(loc="lower right")
    ax = axs[2]
    ax.plot(s, ch["rh_front_mm"], label="ride height front")
    ax.plot(s, ch["rh_rear_mm"], label="ride height rear")
    ax.plot(s, ch["clearance_mm"], "--", color="gray", label="floor clearance (roll incl.)")
    ax.set_ylabel("[mm]")
    ax.legend(loc="lower right", fontsize=8)
    ax = axs[3]
    ax.plot(s, ch["roll_deg"], label="roll")
    ax.plot(s, ch["yaw_deg"], label="yaw (sideslip)")
    ax.plot(s, ch["pitch_deg"] * 10.0, label="pitch x10")
    ax.set_ylabel("[deg]")
    ax.legend(loc="lower right", fontsize=8)
    ax = axs[4]
    ax.plot(s, ch["cla"], label="ClA")
    ax.plot(s, ch["cda"], label="CdA")
    ax.set_ylabel("[m^2]")
    ax2 = ax.twinx()
    ax2.plot(s, ch["cop_front"] * 100.0, color="tab:red", lw=0.8, label="CoP front")
    ax2.set_ylabel("CoP front [%]", color="tab:red")
    ax.legend(loc="lower right", fontsize=8)
    ax = axs[5]
    for k in ("fz_fl_N", "fz_fr_N", "fz_rl_N", "fz_rr_N"):
        ax.plot(s, ch[k], lw=0.9, label=k[3:5].upper())
    ax.set_ylabel("tyre load [N]")
    ax.set_xlabel("distance [m]")
    ax.legend(loc="lower right", fontsize=8, ncol=4)
    for a in axs:
        a.grid(alpha=0.3)
    return _save(fig, path)


def plot_track_map(res, path):
    x, y = res.x, res.y
    fig, ax = plt.subplots(figsize=(7, 7))
    pts = np.column_stack([x, y]).reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="viridis", linewidths=4)
    lc.set_array(0.5 * (res.v[1:] + res.v[:-1]) * 3.6)
    ax.add_collection(lc)
    fig.colorbar(lc, ax=ax, label="speed [km/h]", shrink=0.8)
    other = res.mode != res.default_mode
    if other.any():
        ax.plot(np.where(other, x, np.nan), np.where(other, y, np.nan), color="tab:red", lw=1.2,
                label="low-drag mode")
        ax.legend(loc="best")
    ax.plot(x[0], y[0], "ko", ms=6)
    ax.set_aspect("equal")
    ax.autoscale()
    ax.set_title(f"{res.track_name}  {res.lap_time:.3f} s")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    return _save(fig, path)


def time_delta(ref, other) -> np.ndarray:
    """Time difference of ``other`` vs ``ref`` along the reference distance."""
    return np.interp(ref.s, other.s, other.t) - ref.t


def plot_compare(results, labels, path, title="comparison"):
    fig, axs = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    ref = results[0]
    for r, lab in zip(results, labels):
        axs[0].plot(r.s, r.v * 3.6, lw=1.0, label=f"{lab}  {r.lap_time:.3f} s")
        axs[1].plot(ref.s, time_delta(ref, r), lw=1.0, label=lab)
        axs[2].plot(r.s, r.channels["cla"], lw=1.0, label=lab)
    axs[0].set_ylabel("speed [km/h]")
    axs[0].legend(fontsize=8)
    axs[0].set_title(title)
    axs[1].set_ylabel(f"time delta vs {labels[0]} [s]")
    axs[2].set_ylabel("ClA [m^2]")
    axs[2].set_xlabel("distance [m]")
    for a in axs:
        a.grid(alpha=0.3)
    return _save(fig, path)


def plot_ladder(steps, path):
    fig, ax = plt.subplots(figsize=(9, 4.5))
    base = steps[0].result.lap_time
    prev = base
    for i, st in enumerate(steps):
        t = st.result.lap_time
        if i == 0:
            ax.bar(i, t, color="tab:gray")
        else:
            ax.bar(i, t - prev, bottom=prev, color="tab:red" if t > prev else "tab:green")
        ax.text(i, max(t, prev) + 0.02 * abs(base) * 0.05, f"{t:.3f}\n({t - prev:+.3f})", ha="center",
                va="bottom", fontsize=8)
        prev = t
    times = [s.result.lap_time for s in steps]
    span = max(times) - min(times)
    ax.set_ylim(min(times) - max(span, 0.05) * 1.5, max(times) + max(span, 0.05) * 1.5)
    ax.set_xticks(range(len(steps)), [s.level for s in steps])
    ax.set_ylabel("lap time [s]")
    ax.set_title(f"effect ladder - {steps[0].result.track_name}")
    ax.grid(axis="y", alpha=0.3)
    return _save(fig, path)


def plot_cornering(data, path, title="steady-state cornering limit"):
    v = data["v_kmh"]
    fig, axs = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    a = axs[0, 0]
    a.plot(v, data["ay_max_g"], "k.-")
    a.set_ylabel("ay max [g]")
    for vi, ai, ri in list(zip(v, data["ay_max_g"], data["radius_m"]))[::2]:
        a.annotate(f"R {ri:.0f} m", (vi, ai), textcoords="offset points", xytext=(0, 6), fontsize=7,
                   ha="center", color="tab:gray")
    a = axs[0, 1]
    a.plot(v, data["understeer_index"], ".-")
    a.axhline(1.0, color="k", lw=0.8)
    a.set_ylabel("understeer index u_f/u_r\n(>1 front limited)")
    a = axs[1, 0]
    a.plot(v, data["rh_front_mm"], label="front")
    a.plot(v, data["rh_rear_mm"], label="rear")
    a.plot(v, data["clearance_mm"], "--", color="gray", label="floor clearance")
    a.set_ylabel("ride height [mm]")
    a.legend(fontsize=8)
    a = axs[1, 1]
    a.plot(v, data["roll_deg"], label="roll")
    a.plot(v, data["yaw_deg"], label="yaw")
    a.plot(v, data["steer_deg"], label="steer")
    a.set_ylabel("[deg]")
    a.legend(fontsize=8)
    a = axs[2, 0]
    a.plot(v, data["cla"], label="ClA")
    a.plot(v, data["cda"], label="CdA")
    a.set_ylabel("[m^2]")
    a.legend(fontsize=8)
    a.set_xlabel("speed [km/h]")
    a = axs[2, 1]
    a.plot(v, data["cop_front"] * 100.0, label="aero CoP front")
    a.set_ylabel("CoP front [%]")
    a.set_xlabel("speed [km/h]")
    for ax in axs.flat:
        ax.grid(alpha=0.3)
    fig.suptitle(title)
    return _save(fig, path)


def plot_ggv(ggvs, mode_names, path, speeds_kmh=(20, 40, 60, 80, 100)):
    fig, axs = plt.subplots(1, 2, figsize=(13, 5.5))
    styles = ["-", "--", ":"]
    for k, g in enumerate(ggvs):
        ls = styles[k % len(styles)]
        axs[0].plot(g.v * 3.6, g.ay_max / G, ls, label=f"ay max ({mode_names[g.mode]})")
        axs[0].plot(g.v * 3.6, g.ax_acc[:, 0] / G, ls, label=f"ax accel ({mode_names[g.mode]})")
        axs[0].plot(g.v * 3.6, g.ax_brk[:, 0] / G, ls, label=f"ax brake ({mode_names[g.mode]})")
        for j, vk in enumerate(speeds_kmh):
            v = vk / 3.6
            if v > g.v_max:
                continue
            aym = np.interp(v, g.v, g.ay_max)
            ay = g.fracs * aym
            acc = np.array([np.interp(v, g.v, g.ax_acc[:, i]) for i in range(g.nf)])
            brk = np.array([np.interp(v, g.v, g.ax_brk[:, i]) for i in range(g.nf)])
            yy = np.concatenate([acc, brk[::-1]]) / G
            xx = np.concatenate([ay, ay[::-1]]) / G
            axs[1].plot(np.concatenate([-xx[::-1], xx]), np.concatenate([yy[::-1], yy]), ls,
                        color=f"C{j}", label=f"{vk} km/h" if k == 0 else None)
    axs[0].set_xlabel("speed [km/h]")
    axs[0].set_ylabel("[g]")
    axs[0].legend(fontsize=7)
    axs[1].set_xlabel("ay [g]")
    axs[1].set_ylabel("ax [g]")
    axs[1].set_aspect("equal")
    axs[1].legend(fontsize=8)
    axs[1].set_title("g-g diagram (" + ", ".join(mode_names[g.mode] for g in ggvs) + ")")
    for a in axs:
        a.grid(alpha=0.3)
    return _save(fig, path)


def plot_sweep(sw, path, unit=""):
    obj = sw.objective()
    feas = sw.feasible()
    if len(sw.paths) == 1:
        x = sw.values[0]
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(x, obj, "k-")
        ax.plot(x[feas], obj[feas], "o", color="tab:green", label="constraints ok")
        ax.plot(x[~feas], obj[~feas], "x", color="tab:red", label="constraint violated")
        ax.set_xlabel(sw.paths[0])
        ax.set_ylabel(f"objective [{unit}]")
        ax.legend()
        ax.grid(alpha=0.3)
    else:
        x, y = sw.values[0], sw.values[1]
        fig, ax = plt.subplots(figsize=(8, 6))
        cs = ax.contourf(x, y, obj.T, levels=20, cmap="viridis_r")
        fig.colorbar(cs, ax=ax, label=f"objective [{unit}]")
        X, Y = np.meshgrid(x, y, indexing="ij")
        ax.plot(X[~feas], Y[~feas], "x", color="tab:red", ms=5, label="constraint violated")
        i, j = np.unravel_index(np.nanargmin(np.where(feas, obj, np.nan)), obj.shape) if feas.any() else (0, 0)
        if feas.any():
            ax.plot(x[i], y[j], "*", color="w", ms=14, mec="k", label="best feasible")
        ax.set_xlabel(sw.paths[0])
        ax.set_ylabel(sw.paths[1])
        ax.legend(fontsize=8)
    return _save(fig, path)


def plot_optimization(result, path):
    hist = result.history
    n = [h.n for h in hist]
    val = np.array([h.value for h in hist])
    feas = np.array([h.feasible for h in hist])
    fig, axs = plt.subplots(1 + len(result.spec.variables), 1,
                            figsize=(9, 2.2 * (1 + len(result.spec.variables))), sharex=True)
    axs = np.atleast_1d(axs)
    best = np.minimum.accumulate(np.where(feas, val, np.inf))
    axs[0].plot(n, val, ".", color="gray", ms=3, label="evaluated")
    axs[0].plot(np.array(n)[~feas], val[~feas], "x", color="tab:red", ms=4, label="infeasible")
    axs[0].plot(n, best, "k-", label="best feasible")
    axs[0].set_ylabel(f"{result.label}\n[{result.unit}]", fontsize=8)
    finite = val[np.isfinite(val)]
    if finite.size:
        lo, hi = np.percentile(finite, [0, 90])
        axs[0].set_ylim(lo - 0.05 * (hi - lo + 1e-9), hi + 0.05 * (hi - lo + 1e-9))
    axs[0].legend(fontsize=7)
    for ax, var in zip(axs[1:], result.spec.variables):
        ax.plot(n, [h.x[var.path] for h in hist], ".", ms=3)
        ax.axhline(result.best[var.path], color="k", lw=0.8)
        ax.set_ylabel(var.path.split(".")[-1], fontsize=7)
        ax.set_ylim(var.lo, var.hi)
    axs[-1].set_xlabel("evaluation")
    for a in axs:
        a.grid(alpha=0.3)
    return _save(fig, path)


def plot_dynamics(qss, tr, path):
    """QSS vs transient: speed, ride heights, roll, aero, tyre loads and tyre over-use."""
    lateral = "path_error_m" in tr.channels and np.nanmax(np.abs(tr.channels["path_error_m"])) > 0
    fig, axs = plt.subplots(7 if lateral else 6, 1, figsize=(11, 19 if lateral else 16.5), sharex=True)
    ch_q, ch_t = qss.channels, tr.channels
    axs[0].plot(qss.s, qss.v * 3.6, color="tab:gray", lw=1.0, label=f"QSS  {qss.lap_time:.3f} s")
    axs[0].plot(tr.s, tr.v * 3.6, color="tab:blue", lw=1.0, label=f"transient  {tr.lap_time:.3f} s")
    if "v_ref_kmh" in ch_t:
        axs[0].plot(tr.s, ch_t["v_ref_kmh"], ":", color="tab:orange", lw=0.8, label="driver target (learned)")
    axs[0].set_ylabel("speed [km/h]")
    axs[0].legend(fontsize=8)
    road = tr.info.get("road", "flat") if tr.info else "flat"
    axs[0].set_title(f"{tr.vehicle_name} - {tr.track_name}\nquasi-steady-state vs transient (road: {road})")
    for key, lab, c in (("rh_front_mm", "front", "tab:blue"), ("rh_rear_mm", "rear", "tab:red")):
        axs[1].plot(qss.s, ch_q[key], "--", color=c, lw=0.8, label=f"{lab} QSS")
        axs[1].plot(tr.s, ch_t[key], color=c, lw=0.8, label=f"{lab} transient")
    axs[1].plot(tr.s, ch_t["clearance_mm"], color="k", lw=0.6, label="min floor clearance (transient)")
    axs[1].set_ylabel("ride height [mm]")
    axs[1].legend(fontsize=7, ncol=3)
    axs[2].plot(qss.s, ch_q["roll_deg"], "--", color="tab:gray", lw=0.8, label="roll QSS")
    axs[2].plot(tr.s, ch_t["roll_deg"], color="tab:blue", lw=1.0, label="roll transient")
    axs[2].plot(tr.s, ch_t["pitch_deg"] * 10.0, color="tab:green", lw=0.8, label="pitch x10 transient")
    axs[2].set_ylabel("[deg]")
    axs[2].legend(fontsize=8)
    axs[3].plot(qss.s, ch_q["cla"], "--", color="tab:gray", lw=0.8, label="ClA QSS")
    axs[3].plot(tr.s, ch_t["cla"], color="tab:blue", lw=1.0, label="ClA transient")
    if "flap_position" in ch_t and np.nanmax(ch_t["flap_position"]) > 0:
        ax2 = axs[3].twinx()
        ax2.plot(tr.s, ch_t["flap_position"], color="tab:green", lw=0.8)
        ax2.set_ylabel("flap open [-]", color="tab:green")
    axs[3].set_ylabel("ClA [m^2]")
    axs[3].legend(fontsize=8)
    for key, c in (("fz_fl_N", "tab:blue"), ("fz_rr_N", "tab:red")):
        axs[4].plot(qss.s, ch_q[key], "--", color=c, lw=0.8, label=f"{key[3:5].upper()} QSS")
        axs[4].plot(tr.s, ch_t[key], color=c, lw=0.7, label=f"{key[3:5].upper()} transient")
    if "road_front_mm" in ch_t and np.nanmax(np.abs(ch_t["road_front_mm"])) > 0:
        ax2 = axs[4].twinx()
        ax2.plot(tr.s, ch_t["road_front_mm"], color="tab:brown", lw=0.5, alpha=0.6)
        ax2.set_ylabel("road under front axle [mm]", color="tab:brown")
    axs[4].set_ylabel("tyre load [N]")
    axs[4].legend(fontsize=7, ncol=4)
    if lateral:
        ax = axs[5]
        ax.plot(tr.s, ch_t["path_error_m"], color="tab:purple", lw=1.0, label="path error (left +)")
        ax.set_ylabel("path error [m]")
        ax2 = ax.twinx()
        ax2.plot(qss.s, ch_q["yaw_deg"], "--", color="tab:gray", lw=0.8)
        ax2.plot(tr.s, ch_t["yaw_deg"], color="tab:orange", lw=0.8)
        ax2.set_ylabel("sideslip [deg] (QSS dashed)", color="tab:orange")
        ax.legend(fontsize=8, loc="upper left")
    if "lateral_excess" in ch_t:
        axs[-1].plot(tr.s, ch_t["lateral_excess"] * 100.0, color="tab:red", lw=1.0)
    axs[-1].set_ylabel("learning signal [%]\n(over-use / slide / path)")
    axs[-1].set_xlabel("distance [m]")
    for a in axs:
        a.grid(alpha=0.3)
    return _save(fig, path)


def plot_energy_sweep(path_name, rows, path):
    """Endurance energy and time vs one parameter (battery-limited when the energy does not fit)."""
    x = np.array([r[0] for r in rows])
    e_flat = np.array([r[1].unlimited_required_kWh for r in rows])
    e_drag = np.array([r[1].breakdown.get("aero_drag_kWh", np.nan) * r[1].laps for r in rows])
    t_flat = np.array([r[1].unlimited_lap_time_s * r[1].laps for r in rows])
    t_lim = np.array([r[1].total_time_s for r in rows])
    usable = rows[0][1].usable_kWh
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    a1.plot(x, e_flat, "o-", label="flat-out endurance energy")
    a1.plot(x, e_drag, "s--", label="of which aero drag (at the final cap)")
    if usable is not None:
        a1.axhline(usable, color="k", lw=1.2, label=f"usable battery {usable:.2f} kWh")
    a1.set_ylabel("energy [kWh]")
    a1.legend()
    a1.grid(alpha=0.3)
    a2.plot(x, t_flat, "o--", label="flat out (ignoring the battery)")
    a2.plot(x, t_lim, "o-", label="energy-limited (power cap)")
    a2.set_ylabel("endurance time [s]")
    a2.set_xlabel(path_name)
    a2.legend()
    a2.grid(alpha=0.3)
    fig.suptitle(f"{rows[0][1].track_name}: {rows[0][1].laps} laps")
    _save(fig, path)


def plot_aero_split(res, path):
    """Objective vs each device's design variable (others fixed); joint optimum marked."""
    n = len(res.names)
    cols = min(n, 4)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 3.8 * rows), sharey=True, squeeze=False)
    for ax in axes.flat[n:]:
        ax.set_visible(False)
    for ax, name in zip(axes.flat, res.names):
        x = np.asarray(res.values[name])
        y = np.array([e.value for e in res.sweeps[name]])
        ok = np.array([e.feasible for e in res.sweeps[name]])
        ax.plot(x, y, "-", color="0.4")
        ax.plot(x[ok], y[ok], "o", color="tab:green", label="constraints ok")
        ax.plot(x[~ok], y[~ok], "x", color="tab:red", label="violated")
        if res.joint is not None:
            ax.axvline(res.joint.best[res.paths[name]], color="tab:blue", ls=":", label="joint optimum")
        if res.base is not None:
            ax.axhline(res.base.value, color="k", lw=0.8, label="current")
        unit = res.units.get(name, "x")
        what = {"deg": ("flap angle to main chord [deg]" if "." in name else "main aoa [deg]"),
                "m2": "exit area [m^2]"}.get(unit, "scale [-]")
        ax.set_xlabel(f"{name}: {what}")
        ax.grid(alpha=0.3)
    for r in range(rows):
        axes[r][0].set_ylabel(f"{res.label} [{res.unit}]")
    axes[0][0].legend(fontsize=8)
    fig.suptitle("aero devices: one at a time (search accuracy), dotted = joint optimum")
    _save(fig, path)


def plot_aero_envelope(res, env, vehicle, path):
    """Operating points on the ClA map, aero balance vs speed, roll / yaw, ride heights along the lap."""
    from .analysis.aero_envelope import operating_grid

    ch = res.channels
    rh_f, rh_r = ch["rh_front_mm"], ch["rh_rear_mm"]
    ok = np.isfinite(rh_f) & np.isfinite(rh_r)
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    a = ax[0, 0]
    pad = 5.0
    gf = np.linspace(np.nanmin(rh_f[ok]) - pad, np.nanmax(rh_f[ok]) + pad, 40)
    gr = np.linspace(np.nanmin(rh_r[ok]) - pad, np.nanmax(rh_r[ok]) + pad, 40)
    F, R, cla, share, _ = operating_grid(vehicle, vehicle.default_mode, gf, gr)
    cs = a.contourf(F, R, cla, levels=14, cmap="Blues")
    fig.colorbar(cs, ax=a, label="ClA [m^2] (roll = yaw = 0)")
    cl = a.contour(F, R, share, levels=8, colors="0.3", linewidths=0.7)
    a.clabel(cl, fmt="%.2f", fontsize=7)
    sc = a.scatter(rh_f[ok], rh_r[ok], c=res.v[ok] * 3.6, s=4, cmap="autumn")
    fig.colorbar(sc, ax=a, label="speed [km/h]")
    a.set_xlabel("front ride height [mm]")
    a.set_ylabel("rear ride height [mm]")
    a.set_title("operating points on the aero model (grey lines: front share)")
    b = ax[0, 1]
    v = res.v * 3.6
    col = np.where(res.ax < -2, "tab:blue", np.where(res.ax > 2, "tab:red", "0.7"))
    b.scatter(v, ch["cop_front"], s=3, c=col)
    for k, val in env.balance.items():
        if np.isfinite(val):
            b.axhline(val, lw=0.8, ls=":", color="0.4")
            b.text(v.min(), val, f" {k}", fontsize=7, va="bottom", color="0.3")
    b.set_xlabel("speed [km/h]")
    b.set_ylabel("aero balance (front share)")
    b.set_title("balance: blue = braking, red = accelerating, grey = other")
    b.grid(alpha=0.3)
    c = ax[1, 0]
    c.hist(np.abs(ch["roll_deg"][ok]), bins=30, alpha=0.6, label="|roll| [deg]", weights=np.gradient(res.s)[ok])
    c.hist(np.abs(ch["yaw_deg"][ok]), bins=30, alpha=0.6, label="|yaw| [deg]", weights=np.gradient(res.s)[ok])
    c.set_xlabel("deg")
    c.set_ylabel("distance [m]")
    c.legend()
    c.set_title("roll / yaw the aero sees")
    d = ax[1, 1]
    d.plot(res.s, rh_f, lw=0.8, label="front ride height")
    d.plot(res.s, rh_r, lw=0.8, label="rear ride height")
    d.plot(res.s, ch["clearance_mm"], lw=0.8, ls="--", label="floor clearance (roll incl.)")
    for k, segs in env.where.items():
        for s0, s1 in segs:
            d.axvspan(s0, s1, color="red", alpha=0.15)
    d.set_xlabel("distance [m]")
    d.set_ylabel("mm")
    d.legend(fontsize=8)
    d.grid(alpha=0.3)
    d.set_title("ride heights along the lap (red: flagged zones)")
    fig.suptitle(f"aero operating envelope - {res.track_name}")
    fig.subplots_adjust(hspace=0.32, wspace=0.3)
    _save(fig, path)


def plot_aero_exchange(ex, path):
    fig, ax = plt.subplots(figsize=(8, 0.45 * len(ex.items) + 1.5))
    labels = [i.per for i in ex.items][::-1]
    vals = [i.delta for i in ex.items][::-1]
    ax.barh(labels, vals, color=["tab:green" if x < 0 else "tab:red" for x in vals])
    ax.axvline(0, color="k", lw=0.8)
    ax.set_xlabel(f"objective change [{ex.unit}] (negative = faster)")
    ax.set_title(f"aero exchange rates - {ex.label}")
    ax.grid(alpha=0.3, axis="x")
    _save(fig, path)


def plot_section_rank(rk, path):
    """Objective of each section on the car vs the section-only figures of merit."""
    sc = rk.scores
    y = np.array([s.value for s in sc])
    ok = np.array([s.feasible for s in sc])
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.8), sharey=True)
    for a, x, lab in ((ax[0], [s.max_cl_2d for s in sc], "section max Cl (2-D)"),
                      (ax[1], [s.max_ld_2d for s in sc], "section max L/D (2-D)")):
        x = np.array(x)
        a.scatter(x[ok], y[ok], c="tab:green", label="constraints ok")
        a.scatter(x[~ok], y[~ok], c="tab:red", marker="x", label="violated")
        best = rk.ordered()[0]
        a.scatter([best.max_cl_2d if "Cl" in lab else best.max_ld_2d], [best.value], s=160, facecolors="none",
                  edgecolors="k", label="best for the car")
        a.axhline(rk.current_search.value, color="k", lw=0.8, ls=":", label="current car")
        a.set_xlabel(lab)
        a.grid(alpha=0.3)
    ax[0].set_ylabel(f"{rk.label} [{rk.unit}]")
    ax[0].legend(fontsize=8)
    fig.suptitle(f"{rk.component} wing sections: car objective vs section ranking"
                 + ("  [SYNTHETIC DATA]" if rk.synthetic else ""))
    _save(fig, path)


def plot_auto_aero(res, path):
    """Search history (penalised objective per evaluation, by stage) and what each free variable is worth."""
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5), gridspec_kw={"width_ratios": [1.2, 1]})
    colors = {"start": "k", "options": "tab:purple", "screen": "tab:orange", "search": "tab:blue"}
    h = res.history
    for stage, c in colors.items():
        pts = [(n, v, ok) for n, s, v, _t, ok in h if s == stage and np.isfinite(v)]
        if pts:
            n, v, ok = (np.array(x) for x in zip(*pts))
            a1.plot(n[ok], v[ok], "o", ms=3, color=c, label=stage)
            a1.plot(n[~ok], v[~ok], "x", ms=4, color=c)
    best, run = [], np.inf
    for n, _s, v, t, ok in h:
        if ok and np.isfinite(v):
            run = min(run, v)
        best.append(run)
    a1.plot([x[0] for x in h], best, "-", color="tab:green", label="best feasible")
    a1.axhline(res.initial_search.value, color="0.3", lw=0.8, ls="--", label="start")
    a1.set_xlabel("evaluation")
    a1.set_ylabel(f"{res.label} [{res.unit}] (search accuracy)")
    a1.set_title("search history (x = constraint violated)")
    a1.grid(alpha=0.3)
    a1.legend(fontsize=8)
    scr = sorted(res.screen, key=lambda s: abs(s.delta))[-20:]
    if scr:
        y = np.arange(len(scr))
        a2.barh(y, [s.delta for s in scr], color=["tab:blue" if s.used else "0.7" for s in scr])
        a2.set_yticks(y, [f"{s.var.name} ({s.step:+.3g} {s.var.unit})" for s in scr], fontsize=7)
        a2.axvline(0, color="k", lw=0.8)
        a2.set_xlabel(f"objective change for one step [{res.unit}] (< 0 = better)")
        a2.set_title("what each free variable is worth (blue = searched)")
        a2.grid(alpha=0.3, axis="x")
    _save(fig, path)


def plot_tire(tire, path, loads=None):
    """Magic Formula tyre: Fy(alpha), Fx(kappa) per load, mu vs load, lean angle, combined envelope."""
    fz_ref = tire.fz_ref
    loads = loads or [0.5 * fz_ref, fz_ref, 1.5 * fz_ref, 2.0 * fz_ref]
    fig, axs = plt.subplots(2, 3, figsize=(15, 8.5))
    a = np.radians(np.linspace(0.0, 15.0, 151))
    k = np.linspace(0.0, 0.35, 151)
    for fz in loads:
        line, = axs[0, 0].plot(np.degrees(a), tire.fy_pure(a, fz), label=f"{fz:.0f} N")
        pk = float(tire.peak_slip_at(fz))
        axs[0, 0].plot(np.degrees(pk), float(tire.fy_pure(pk, fz)), "o", color=line.get_color(), ms=4)
        axs[0, 1].plot(k, tire.fx_pure(k, fz), color=line.get_color(), label=f"{fz:.0f} N")
    axs[0, 0].set_xlabel("slip angle [deg]")
    axs[0, 0].set_ylabel("Fy [N] (0 camber)")
    axs[0, 0].set_title("lateral force (dots: peak)")
    axs[0, 1].set_xlabel("slip ratio [-]")
    axs[0, 1].set_ylabel("Fx [N]")
    axs[0, 1].set_title("longitudinal force")
    fzs = np.linspace(0.2 * fz_ref, 3.0 * fz_ref, 60)
    ax = axs[0, 2]
    ax.plot(fzs, tire.mu_x(fzs), label="mu_x (peak Fx / Fz)")
    ax.plot(fzs, tire.mu_y(fzs), label="mu_y (peak Fy / Fz)")
    ax.axvline(fz_ref, color="gray", lw=0.8, ls="--")
    ax.set_xlabel("vertical load [N]")
    ax.set_ylabel("friction coefficient")
    ax2 = ax.twinx()
    ax2.plot(fzs, np.degrees(tire.peak_slip_at(fzs)), color="tab:red", ls=":", label="peak slip angle")
    ax2.set_ylabel("peak slip angle [deg]", color="tab:red")
    ax.set_title("load sensitivity")
    ax = axs[1, 0]
    g = np.linspace(-4.0, 8.0, 121)
    for fz in loads:
        _, fy = tire.capacity(np.full(g.shape, fz), np.radians(g), np.radians(-g))
        base = float(tire.capacity(np.asarray(fz))[1])
        ax.plot(g, (fy / base - 1.0) * 100.0, label=f"{fz:.0f} N")
    ax.axhline(0.0, color="k", lw=0.6)
    ax.set_xlabel("lean towards the turn centre [deg]")
    ax.set_ylabel("lateral capacity change [%]")
    ax.set_title("camber (thrust + friction loss)")
    ax = axs[1, 1]
    fz = fz_ref
    al = np.linspace(0.0, 3.0 * float(tire.peak_slip_at(fz)), 7)
    kk = np.linspace(0.0, 3.0 * float(tire.peak_slip_ratio_at(fz)), 200)
    dx = float(tire.mu_x(fz)) * fz
    dy = float(tire.mu_y(fz)) * fz
    for aa in al:
        fx, fy = tire.combined_forces(np.full(kk.shape, aa), kk, fz)
        ax.plot(fx / dx, fy / dy, color="tab:blue", lw=0.8)
    th = np.linspace(0.0, 0.5 * np.pi, 100)
    n = tire.combined_exponent
    c, s = np.cos(th), np.sin(th)
    r = 1.0 / (c ** n + s ** n) ** (1.0 / n)
    ax.plot(r * c, r * s, "k--", label=f"|ux|^n + |uy|^n = 1, n = {n:.2f}")
    ax.plot(c, s, ":", color="gray", label="friction circle")
    ax.set_xlabel("Fx / Dx")
    ax.set_ylabel("Fy / Dy")
    ax.set_aspect("equal")
    ax.set_xlim(0, 1.1)
    ax.set_ylim(0, 1.1)
    ax.set_title(f"combined slip at {fz:.0f} N (lines: const. slip angle)")
    axs[1, 2].axis("off")
    axs[1, 2].text(0.0, 1.0, tire.describe(), va="top", family="monospace", fontsize=7.5, wrap=True)
    for ax in (axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1], axs[0, 2]):
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="lower right")
    fig.suptitle(f"Magic Formula tyre - {tire.source}")
    fig.tight_layout()
    _save(fig, path)


def plot_racing_line(res, path, title=None):
    """Track edges, input line and the optimised line (coloured by speed); speed and curvature along the lap."""
    (lx, ly), (rx, ry) = res.edge_lines()
    x, y = res.line_xy()
    lap, ref_lap = res.lap, res.reference_lap
    fig = plt.figure(figsize=(15, 8.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1.0])
    ax = fig.add_subplot(gs[:, 0])
    closed = res.ref.closed

    def cl(a):
        return np.append(a, a[0]) if closed else a

    ax.plot(cl(lx), cl(ly), color="0.35", lw=1.0)
    ax.plot(cl(rx), cl(ry), color="0.35", lw=1.0, label="track edges")
    ax.plot(cl(res.ref.x), cl(res.ref.y), "--", color="tab:blue", lw=0.8, label="input line")
    lx2, ly2 = lap.x, lap.y
    pts = np.column_stack([lx2, ly2]).reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="viridis", linewidths=2.2)
    lc.set_array(0.5 * (lap.v[1:] + lap.v[:-1]) * 3.6)
    ax.add_collection(lc)
    fig.colorbar(lc, ax=ax, orientation="horizontal", fraction=0.04, pad=0.07,
                 label="speed on the optimised line [km/h]")
    ax.plot(lx2[0], ly2[0], "ko", ms=5, label="start")
    ax.set_aspect("equal")
    ax.legend(loc="best", fontsize=8)
    fig.suptitle(title or f"{lap.track_name}: {ref_lap.lap_time:.3f} s -> {lap.lap_time:.3f} s "
                 f"({res.gain_s:+.3f} s)")
    a1 = fig.add_subplot(gs[0, 1])
    a1.plot(ref_lap.s / ref_lap.s[-1] * 100.0, ref_lap.v * 3.6, color="tab:blue", lw=1.0,
            label=f"input line {ref_lap.lap_time:.2f} s")
    a1.plot(lap.s / lap.s[-1] * 100.0, lap.v * 3.6, color="k", lw=1.2, label=f"optimised {lap.lap_time:.2f} s")
    a1.set_ylabel("speed [km/h]")
    a1.legend(fontsize=8)
    a1.grid(alpha=0.3)
    a2 = fig.add_subplot(gs[1, 1], sharex=a1)
    a2.plot(ref_lap.s / ref_lap.s[-1] * 100.0, np.abs(ref_lap.kappa), color="tab:blue", lw=0.8, label="input line")
    a2.plot(lap.s / lap.s[-1] * 100.0, np.abs(lap.kappa), color="k", lw=1.0, label="optimised")
    a2.set_ylabel("|curvature| [1/m]")
    a2.set_xlabel("distance [% of the lap]")
    a2.legend(fontsize=8)
    a2.grid(alpha=0.3)
    _save(fig, path)


def plot_energy_strategy(st, path):
    """Flat-out / power-cap / energy-optimal laps: speed and battery power, coasting shaded; time-energy fronts."""
    flat = st.stages[0].dp.flat if st.stages else None
    opt, cap = st.result, st.cap.result
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.2, 1.0, 1.1])
    a1 = fig.add_subplot(gs[0, :])
    if flat is not None:
        a1.plot(flat.s, flat.v * 3.6, color="0.6", lw=1.0, label=f"flat out {flat.lap_time:.2f} s")
    if cap is not None:
        lab = "power cap" + (f" {st.cap.power_cap_kW:.1f} kW" if st.cap.power_cap_kW else "")
        a1.plot(cap.s, cap.v * 3.6, color="tab:red", lw=1.0, label=f"{lab} {cap.lap_time:.2f} s")
    a1.plot(opt.s, opt.v * 3.6, color="k", lw=1.3, label=f"energy-optimal {opt.lap_time:.2f} s")
    for p in st.lifts:
        a1.axvspan(p.s_lift, p.s_brake, color="tab:green", alpha=0.18, lw=0)
    a1.set_ylabel("speed [km/h]")
    a1.set_title(f"{st.track_name}: {st.laps} laps, endurance {st.cap.total_time_s:.1f} s (power cap) -> "
                 f"{st.total_s:.1f} s (energy-optimal, {st.gain_s:+.1f} s); green = lift and coast")
    a1.legend(loc="lower right", fontsize=8)
    a1.grid(alpha=0.3)
    a2 = fig.add_subplot(gs[1, :], sharex=a1)
    for res, color, lab in ((flat, "0.6", "flat out"), (cap, "tab:red", "power cap"), (opt, "k", "energy-optimal")):
        if res is not None and "battery_power_kW" in res.channels:
            a2.plot(res.s, res.channels["battery_power_kW"], color=color, lw=0.9, label=lab)
    for p in st.lifts:
        a2.axvspan(p.s_lift, p.s_brake, color="tab:green", alpha=0.18, lw=0)
    a2.set_ylabel("battery power [kW]")
    a2.set_xlabel("distance [m]")
    a2.legend(loc="upper right", fontsize=8)
    a2.grid(alpha=0.3)
    a3 = fig.add_subplot(gs[2, 0])
    if st.pareto_opt:
        e, t = zip(*sorted(st.pareto_opt))
        a3.plot(e, t, "o-", color="k", label="energy-optimal driving")
    if st.pareto_cap:
        e, t, _ = zip(*sorted(st.pareto_cap))
        a3.plot(e, t, "s--", color="tab:red", label="power cap")
    if st.usable_kWh is not None:
        a3.axvline(st.usable_kWh / st.laps, color="tab:blue", lw=1.0, label="usable energy / laps")
    a3.set_xlabel("battery energy per lap [kWh]")
    a3.set_ylabel("lap time [s]")
    a3.set_title("time-energy trade-off (first SOC stage)")
    a3.legend(fontsize=8)
    a3.grid(alpha=0.3)
    a4 = fig.add_subplot(gs[2, 1])
    keys = (("braking_heat_kWh", "brake heat"), ("aero_drag_kWh", "aero drag"), ("rolling_kWh", "rolling"),
            ("powertrain_losses_kWh", "powertrain"), ("battery_loss_kWh", "pack R I^2"))
    from .analysis.energy import energy_breakdown

    b_opt = energy_breakdown(st.stages[0].dp.car, opt) if st.stages else {}
    b_cap = st.cap.breakdown
    x = np.arange(len(keys))
    a4.bar(x - 0.2, [b_cap.get(k, 0.0) for k, _ in keys], 0.4, color="tab:red", label="power cap")
    a4.bar(x + 0.2, [b_opt.get(k, 0.0) for k, _ in keys], 0.4, color="k", label="energy-optimal")
    a4.set_xticks(x)
    a4.set_xticklabels([lab for _, lab in keys], fontsize=8)
    a4.set_ylabel("energy per lap [kWh]")
    a4.set_title("where the energy goes")
    a4.legend(fontsize=8)
    a4.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    _save(fig, path)
