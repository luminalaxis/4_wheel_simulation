"""Aero operating envelope of a lap: where on the aero map the car actually runs.

From the lap channels (ride heights, roll, yaw, speed, ClA, CdA, aero balance, clearance and the
flags) this reports, distance-weighted:

* the range (5 / 50 / 95 %) of front / rear ride height, rake, roll, yaw and speed;
* the aero balance (front share of downforce) per situation - braking, cornering, accelerating -
  and its migration with speed / pitch: a balance that moves forward under braking makes the
  car loose on corner entry;
* the share and track positions where the map is extrapolated (outside its data), the floor
  touches the ground, or a device is choked / stalled (low clearance);

and draws the operating points on the ClA / balance contours of the aero model.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..solvers.base import LapResult


def _wq(x, w, q):
    """Weighted quantile."""
    ok = np.isfinite(x)
    x, w = x[ok], w[ok]
    if x.size == 0:
        return float("nan")
    o = np.argsort(x)
    cw = np.cumsum(w[o])
    return float(np.interp(q * cw[-1], cw, x[o]))


def _segments(mask, s, min_len=2.0):
    """Track intervals [s0, s1] where ``mask`` holds (merged, at least ``min_len`` m)."""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = s[i]
        if (not m or i == len(mask) - 1) and start is not None:
            end = s[i]
            if end - start >= min_len:
                out.append((float(start), float(end)))
            start = None
    return out


@dataclass
class AeroEnvelope:
    track: str
    ranges: dict[str, tuple[float, float, float]]  # name -> (5 %, 50 %, 95 %)
    balance: dict[str, float]  # situation -> mean front share
    balance_vs_speed: list[tuple[float, float]]  # (km/h, front share)
    shares: dict[str, float]  # flag -> share of the lap distance
    where: dict[str, list[tuple[float, float]]] = field(default_factory=dict)
    balance_pitch_slope: float = float("nan")  # d(front share) / d(front ride height) [1/mm]
    notes: list[str] = field(default_factory=list)

    def report(self) -> str:
        lines = [f"aero operating envelope - {self.track} (distance weighted)", "",
                 f"{'':22s} {'5 %':>9s} {'50 %':>9s} {'95 %':>9s}"]
        for k, (a, b, c) in self.ranges.items():
            lines.append(f"{k:22s} {a:9.2f} {b:9.2f} {c:9.2f}")
        lines += ["", "aero balance (front share of downforce):"]
        for k, v in self.balance.items():
            lines.append(f"  {k:20s} {v:7.3f}")
        lines.append("  vs speed: " + ", ".join(f"{v:.0f} km/h {b:.3f}" for v, b in self.balance_vs_speed))
        if np.isfinite(self.balance_pitch_slope):
            lines.append(f"  d(front share)/d(front ride height) = {self.balance_pitch_slope * 10:+.4f} per 10 mm "
                         f"({'moves forward' if self.balance_pitch_slope < 0 else 'moves rearward'} as the nose drops)")
        lines += ["", "share of the lap distance:"]
        for k, v in self.shares.items():
            where = self.where.get(k) or []
            at = ", ".join(f"{a:.0f}-{b:.0f} m" for a, b in where[:6]) + (" ..." if len(where) > 6 else "")
            lines.append(f"  {k:30s} {v * 100:6.1f} %" + (f"   at {at}" if at else ""))
        if self.notes:
            lines += [""] + [f"NOTE: {n}" for n in self.notes]
        return "\n".join(lines)


def aero_envelope(res: LapResult, vehicle=None, min_clear_mm: float | None = None) -> AeroEnvelope:
    ch = res.channels
    s = res.s
    w = np.gradient(s)  # distance weight per sample
    v_kmh = res.v * 3.6
    rh_f, rh_r = ch["rh_front_mm"], ch["rh_rear_mm"]
    rake_deg = np.degrees(np.arctan((rh_r - rh_f) / (1000.0 * (vehicle.L if vehicle is not None else 1.55))))
    ranges = {"speed [km/h]": v_kmh, "front ride height [mm]": rh_f, "rear ride height [mm]": rh_r,
              "rake [deg]": rake_deg, "roll [deg]": np.abs(ch["roll_deg"]), "yaw [deg]": np.abs(ch["yaw_deg"]),
              "floor clearance [mm]": ch["clearance_mm"], "ClA [m2]": ch["cla"], "CdA [m2]": ch["cda"],
              "front share": ch["cop_front"]}
    rng = {k: (_wq(x, w, 0.05), _wq(x, w, 0.5), _wq(x, w, 0.95)) for k, x in ranges.items()}

    cop = ch["cop_front"]
    braking = res.ax < -2.0
    accel = res.ax > 2.0
    corner = (np.abs(res.ay) > 5.0) & ~braking & ~accel
    straight = ~braking & ~accel & ~corner

    def mean(mask):
        m = mask & np.isfinite(cop)
        return float(np.sum(cop[m] * w[m]) / np.sum(w[m])) if m.any() else float("nan")

    balance = {"braking": mean(braking), "cornering": mean(corner), "accelerating": mean(accel),
               "steady / straight": mean(straight)}
    edges = np.arange(20.0, max(float(v_kmh.max()), 30.0) + 20.0, 20.0)
    bvs = []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (v_kmh >= a) & (v_kmh < b) & np.isfinite(cop)
        if m.sum() > 3:
            bvs.append((0.5 * (a + b), float(np.sum(cop[m] * w[m]) / np.sum(w[m]))))
    ok = np.isfinite(cop) & np.isfinite(rh_f) & (res.v > 10.0)
    slope = float(np.polyfit(rh_f[ok], cop[ok], 1)[0]) if ok.sum() > 10 and np.ptp(rh_f[ok]) > 0.5 else float("nan")

    flags = {"aero map extrapolated": ch.get("aero_out_of_range"), "floor touching (bottoming)": ch.get("bottoming")}
    if vehicle is not None:
        low = []
        for comp in vehicle.aero.components:
            dev = getattr(comp, "dev", None)
            if dev is None:
                continue
            h = dev.height + vehicle.aero.geometry.local_change(dev.x, rh_f, rh_r)
            if hasattr(dev, "stall") and not hasattr(dev, "cl_max"):
                flags[f"{comp.name}: choked (< {dev.stall:.0f} mm)"] = h < dev.stall
            elif getattr(dev, "ground_effect", False):
                flags[f"{comp.name}: ground stall (< {dev.h_stall:.0f} mm)"] = h < dev.h_stall
            low.append(comp.name)
    if min_clear_mm is not None:
        flags[f"clearance below {min_clear_mm:g} mm"] = ch["clearance_mm"] < min_clear_mm
    shares, where = {}, {}
    for k, f in flags.items():
        if f is None:
            continue
        f = np.asarray(f, dtype=bool)
        shares[k] = float(np.sum(w[f]) / np.sum(w))
        where[k] = _segments(f, s)

    env = AeroEnvelope(res.track_name, rng, balance, bvs, shares, where, slope)
    if np.isfinite(balance["braking"]) and np.isfinite(balance["steady / straight"]) and \
            balance["braking"] > balance["steady / straight"] + 0.01:
        env.notes.append(f"the aero balance moves forward under braking ({balance['steady / straight']:.3f} -> "
                         f"{balance['braking']:.3f}): less rear downforce on corner entry (entry oversteer risk)")
    if shares.get("aero map extrapolated", 0.0) > 0.02:
        env.notes.append("more than 2 % of the lap is outside the aero map data: extend the CFD / wind-tunnel map")
    if shares.get("floor touching (bottoming)", 0.0) > 0.0:
        env.notes.append("the floor touches the ground: raise the static ride height or stiffen heave")
    return env


def operating_grid(vehicle, mode: int, rh_f, rh_r):
    """ClA and front share of the aero model on a ride-height grid (roll = yaw = 0)."""
    F, R = np.meshgrid(rh_f, rh_r, indexing="ij")
    cf, cr, cd, _ = vehicle.aero.coefficients(mode, F, R, 0.0, 0.0)
    tot = cf + cr
    with np.errstate(invalid="ignore", divide="ignore"):
        share = np.where(np.abs(tot) > 1e-9, cf / tot, np.nan)
    return F, R, tot, share, cd
