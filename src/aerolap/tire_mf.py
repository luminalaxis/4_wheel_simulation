"""Pacejka Magic Formula tyre from a ``.tir`` file (MF 5.2 / PAC2002 / MF 6.1 coefficient sets).

Vehicle file::

    tires:
      radius_mm: 203                 # rolling radius, vertical rate and rolling resistance stay config values
      mu_x: 1.50                     # friction level of the track surface at ref_load_N (see mf_mu_scaling)
      mu_y: 1.55
      ref_load_N: 650
      mf_file: ../tires/fsae_16x7.5-10_synthetic.tir
      pressure_kPa: 83               # inflation pressure (MF 6.1 pressure terms); null -> INFLPRES / NOMPRES
      mf_mu_scaling: true            # scale MF friction so mu(ref_load, 0 camber, NOMPRES) = mu_x / mu_y
      combined_exponent: null        # friction-ellipse exponent; null -> fitted to the MF combined-slip terms
      mf: {PDY2: -0.3}               # optional coefficient overrides (upper-case .tir names)

What the 4-wheel model takes from the Magic Formula (ISO-W / TYDEX sign convention, SI units):

* peak forces ``Dx = mu_x(Fz, gamma, p) Fz``, ``Dy = mu_y(Fz, gamma, p) Fz`` with
  ``mu = (PD1 + PD2 dfz)(1 + PP3 dpi + PP4 dpi^2)(1 - PD3 gamma^2) LMU``, ``dfz = (Fz - Fz0')/Fz0'``,
  ``dpi = (p - NOMPRES)/NOMPRES`` (pressure terms are zero in MF 5.2 files);
* camber thrust towards the side the wheel leans to, ``-(PVY3 + PVY4 dfz) Fz gamma``: a wheel leaning
  towards the turn centre gains lateral capacity, the ``PDY3`` term loses it -> the camber optimum
  follows from the data;
* cornering stiffness ``Ky = PKY1 Fz0' (1 + PPY1 dpi)(1 - PKY3|gamma|) sin(PKY4 atan(Fz / ((PKY2 + PKY5
  gamma^2)(1 + PPY2 dpi) Fz0')))`` and the slip curve ``Fy/Dy = sin(Cy atan(By a - Ey(By a - atan By a)))``,
  ``By = Ky/(Cy Dy)``: the peak slip angle grows with load, the force falls to ``sin(Cy pi/2)`` of the
  peak when sliding;
* combined slip: the friction ellipse exponent is fitted to the envelope of the MF combined-slip
  weighting functions ``Gx_alpha``, ``Gy_kappa`` (``RBX*``, ``RCX1``, ``RBY*``, ``RCY1``);
* vertical stiffness: the configured ``vertical_rate_N_per_mm`` (at NOMPRES) times ``1 + PFZ1 dpi``.

The model is symmetric left / right (like the rest of the vehicle model): conicity / ply-steer offsets
(``PHX*``, ``PVX*``, ``PHY1-2``, ``PVY1-2``, ``PEY3-4``, ``PEX4``) are left out.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

# coefficients read from a .tir file (missing -> 0); scaling factors default to 1
_SCALING = ("LFZO", "LCX", "LMUX", "LEX", "LKX", "LCY", "LMUY", "LEY", "LKY", "LKYC", "LXAL", "LYKA")
_DEFAULTS = {k: 1.0 for k in _SCALING}
_DEFAULTS.update({"PKY4": 2.0})  # MF 5.2 has the 2 built in


class TirError(ValueError):
    pass


def read_tir(path: str | Path) -> dict:
    """``KEY = value`` pairs of a ``.tir`` file (upper-case keys, numbers as float; ``$`` / ``!`` comments)."""
    out: dict = {}
    section = ""
    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.split("$", 1)[0].split("!", 1)[0].strip()
            if not line:
                continue
            if line.startswith("[") and "]" in line:
                section = line[1:line.index("]")].strip().upper()
                continue
            if "=" not in line or section in ("SHAPE",):
                continue
            key, val = line.split("=", 1)
            key = key.strip().upper()
            val = val.strip().strip("'\"").strip()
            try:
                out[key] = float(val)
            except ValueError:
                out[key] = val
    return out


def _unit_factor(params: dict, key: str, allowed: dict[str, float], default: str) -> float:
    unit = str(params.get(key, default)).strip().lower()
    if unit not in allowed:
        raise TirError(f".tir [UNITS] {key} = {unit!r} is not supported (use {', '.join(sorted(set(allowed)))})")
    return allowed[unit]


class MagicFormulaTire:
    """Magic Formula tyre with the interface of :class:`aerolap.tire.Tire`."""

    model = "mf"

    def __init__(self, coef: dict, *, radius: float, vertical_rate: float, rolling_resistance: float,
                 fz_ref: float, mu_x: float, mu_y: float, pressure: float | None = None, scale_mu: bool = True,
                 combined_exponent: float | None = None, source: str = ""):
        p = dict(_DEFAULTS)
        p.update({k.upper(): float(v) for k, v in coef.items() if isinstance(v, (int, float))})
        self.p = p
        self.source = source
        self.radius = radius
        self.rolling_resistance = rolling_resistance
        self.fz_ref = fz_ref
        fnom = p.get("FNOMIN", 0.0)
        if fnom <= 0.0:
            raise TirError("the .tir file has no FNOMIN (nominal load) > 0")
        self.fz0 = fnom * p["LFZO"]
        nom = p.get("NOMPRES", 0.0)
        self.nompres = nom if nom > 0.0 else None
        if pressure is None:
            pressure = p.get("INFLPRES") or nom or None
        self.pressure = None if pressure is None else float(pressure)  # Pa
        self.dpi = 0.0 if (self.nompres is None or self.pressure is None) else (self.pressure - nom) / nom
        # MF 6.1: vertical stiffness grows with pressure (PFZ1); the configured rate is the one at NOMPRES
        self.vertical_rate = vertical_rate * max(1.0 + p.get("PFZ1", 0.0) * self.dpi, 0.2)
        for key in ("PCX1", "PDX1", "PKX1", "PCY1", "PDY1", "PKY1"):
            if p.get(key, 0.0) == 0.0:
                raise TirError(f"the .tir file needs a non-zero {key}")
        self._sx = 1.0 if p["PDX1"] > 0 else -1.0
        self._sy = 1.0 if p["PDY1"] > 0 else -1.0
        self.cx = p["PCX1"] * p["LCX"]
        self.cy = p["PCY1"] * p["LCY"]
        self._peak_x = 1.0 if self.cx >= 1.0 else math.sin(self.cx * math.pi / 2.0)
        self._peak_y = 1.0 if self.cy >= 1.0 else math.sin(self.cy * math.pi / 2.0)
        # friction level: scale MF mu so that mu(fz_ref, no camber, nominal pressure) = mu_x / mu_y
        self.kx = self.ky = 1.0
        self.scale_mu = bool(scale_mu)
        if self.scale_mu:
            self.kx = mu_x / float(self._mu_x(np.asarray(fz_ref), 0.0, 0.0))
            self.ky = mu_y / float(self._mu_y(np.asarray(fz_ref), 0.0, 0.0))
        # constants of the zero-camber lateral curve (scalar fast path of the transient solver)
        self._sc = (self.ky * self._sy * p["LMUY"] * (1.0 + p.get("PPY3", 0.0) * self.dpi
                                                      + p.get("PPY4", 0.0) * self.dpi ** 2),
                    p["PDY1"], p.get("PDY2", 0.0), self.ky * 0.15 * abs(p["PDY1"]) * p["LMUY"],
                    abs(p["PKY1"] * self.fz0 * (1.0 + p.get("PPY1", 0.0) * self.dpi) * p["LKY"]),
                    p["PKY4"], p.get("PKY2", 0.0) * (1.0 + p.get("PPY2", 0.0) * self.dpi) * self.fz0,
                    p.get("PEY1", 0.0) * p["LEY"], p.get("PEY2", 0.0) * p["LEY"])
        self._zpk_cache: dict = {}
        self.combined_exponent = (float(combined_exponent) if combined_exponent is not None
                                  else self.fit_combined_exponent())
        self.combined_fitted = combined_exponent is None
        self.peak_slip = float(self.peak_slip_at(fz_ref))
        self.uses_camber = True

    # ------------------------------------------------------------------ construction
    @classmethod
    def from_config(cls, t: dict, base_dir: str | Path = ".") -> "MagicFormulaTire":
        from .config import ConfigError

        coef: dict = {}
        source = "inline tires.mf"
        f = t.get("mf_file")
        if f:
            path = Path(f)
            if not path.is_absolute():
                path = Path(base_dir) / path
            if not path.exists():
                raise ConfigError(path="tires.mf_file", value=f, condition="있는 .tir 파일",
                                  hint="차량 YAML 기준 상대 경로 또는 절대 경로로 Magic Formula .tir 파일을 지정하세요")
            try:
                coef = read_tir(path)
                lf = _unit_factor(coef, "LENGTH", {"meter": 1.0, "m": 1.0, "millimeter": 1e-3, "mm": 1e-3}, "meter")
                ff = _unit_factor(coef, "FORCE", {"newton": 1.0, "n": 1.0, "kilonewton": 1e3, "kn": 1e3}, "newton")
                _unit_factor(coef, "ANGLE", {"radian": 1.0, "radians": 1.0, "rad": 1.0}, "radians")
                pf = _unit_factor(coef, "PRESSURE", {"pascal": 1.0, "pa": 1.0, "kpa": 1e3, "kilopascal": 1e3,
                                                     "bar": 1e5, "psi": 6894.757}, "pascal")
            except (OSError, TirError) as exc:
                raise ConfigError(path="tires.mf_file", value=f, condition="읽을 수 있는 Magic Formula .tir (SI 단위)",
                                  message=str(exc)) from None
            for k in ("FNOMIN",):
                if isinstance(coef.get(k), float):
                    coef[k] *= ff
            for k in ("UNLOADED_RADIUS", "WIDTH", "RIM_RADIUS"):
                if isinstance(coef.get(k), float):
                    coef[k] *= lf
            for k in ("NOMPRES", "INFLPRES"):
                if isinstance(coef.get(k), float):
                    coef[k] *= pf
            source = path.name
        coef.update({str(k).upper(): v for k, v in (t.get("mf") or {}).items()})
        p_kpa = t.get("pressure_kPa")
        n = t.get("combined_exponent")
        from .tire import _exponent

        try:
            return cls(coef, radius=float(t["radius_mm"]) / 1000.0,
                       vertical_rate=float(t["vertical_rate_N_per_mm"]) * 1000.0,
                       rolling_resistance=float(t["rolling_resistance"]), fz_ref=float(t["ref_load_N"]),
                       mu_x=float(t["mu_x"]), mu_y=float(t["mu_y"]),
                       pressure=None if p_kpa is None else float(p_kpa) * 1000.0,
                       scale_mu=bool(t.get("mf_mu_scaling", True)),
                       combined_exponent=None if n is None else _exponent(n, 2.0), source=source)
        except TirError as exc:
            raise ConfigError(path="tires.mf_file", value=f, message=str(exc)) from None

    # ------------------------------------------------------------------ friction / stiffness
    def _dfz(self, fz):
        return (np.asarray(fz, dtype=float) - self.fz0) / self.fz0

    def _mu_x(self, fz, gamma, dpi):
        p = self.p
        dfz = self._dfz(fz)
        mu = (self._sx * (p["PDX1"] + p.get("PDX2", 0.0) * dfz) * (1.0 + p.get("PPX3", 0.0) * dpi
                                                                      + p.get("PPX4", 0.0) * dpi * dpi)
              * (1.0 - p.get("PDX3", 0.0) * np.asarray(gamma) ** 2) * p["LMUX"])
        return np.maximum(mu, 0.15 * abs(p["PDX1"]) * p["LMUX"])

    def _mu_y(self, fz, gamma, dpi):
        p = self.p
        dfz = self._dfz(fz)
        mu = (self._sy * (p["PDY1"] + p.get("PDY2", 0.0) * dfz) * (1.0 + p.get("PPY3", 0.0) * dpi
                                                                      + p.get("PPY4", 0.0) * dpi * dpi)
              * (1.0 - p.get("PDY3", 0.0) * np.asarray(gamma) ** 2) * p["LMUY"])
        return np.maximum(mu, 0.15 * abs(p["PDY1"]) * p["LMUY"])

    def mu_x(self, fz, gamma=0.0):
        """Longitudinal friction coefficient (scaled) at the configured pressure."""
        return self.kx * self._mu_x(fz, gamma, self.dpi)

    def mu_y(self, fz, gamma=0.0):
        return self.ky * self._mu_y(fz, gamma, self.dpi)

    def cornering_stiffness(self, fz, gamma=0.0):
        """|dFy/dalpha| at zero slip, N/rad."""
        p = self.p
        fz = np.maximum(np.asarray(fz, dtype=float), 1.0)
        g = np.asarray(gamma, dtype=float)
        den = (p.get("PKY2", 0.0) + p.get("PKY5", 0.0) * g * g) * (1.0 + p.get("PPY2", 0.0) * self.dpi) * self.fz0
        den = np.where(np.abs(den) > 1e-9, den, 1e-9)
        k = (p["PKY1"] * self.fz0 * (1.0 + p.get("PPY1", 0.0) * self.dpi) * (1.0 - p.get("PKY3", 0.0) * np.abs(g))
             * np.sin(p["PKY4"] * np.arctan(fz / den)) * p["LKY"])
        return np.abs(k)

    def slip_stiffness(self, fz):
        """|dFx/dkappa| at zero slip, N per unit slip ratio."""
        p = self.p
        fz = np.maximum(np.asarray(fz, dtype=float), 1.0)
        dfz = self._dfz(fz)
        dpi = self.dpi
        k = (fz * (p["PKX1"] + p.get("PKX2", 0.0) * dfz) * np.exp(p.get("PKX3", 0.0) * dfz)
             * (1.0 + p.get("PPX1", 0.0) * dpi + p.get("PPX2", 0.0) * dpi * dpi) * p["LKX"])
        return np.abs(k)

    def camber_thrust(self, fz, inclination):
        """Lateral force towards the side the wheel leans to (positive = towards the turn centre), N."""
        p = self.p
        dfz = self._dfz(fz)
        # ISO: positive camber tilts the top to -y and PVY3 < 0 pushes to -y -> thrust towards the lean
        return (-(p.get("PVY3", 0.0) + p.get("PVY4", 0.0) * dfz) * np.asarray(fz, dtype=float)
                * np.asarray(inclination, dtype=float) * p["LKYC"] * p["LMUY"] * self.ky)

    # ------------------------------------------------------------------ interface of aerolap.tire.Tire
    def capacity(self, fz, inclination=None, camber=None):
        """Peak longitudinal and lateral force of each tyre (friction-ellipse semi-axes)."""
        fz = np.asarray(fz, dtype=float)
        if inclination is None or camber is None:
            fx = self.mu_x(fz) * fz * self._peak_x
            fy = self.mu_y(fz) * fz * self._peak_y
            return fx, fy
        gx = np.asarray(camber, dtype=float)
        gy = np.asarray(inclination, dtype=float)
        fx = self.mu_x(fz, gx) * fz * self._peak_x
        fy = self.mu_y(fz, gy) * fz * self._peak_y + self.camber_thrust(fz, gy)
        return fx, np.maximum(fy, 0.0)

    def _shape_y(self, fz):
        """By, Ey of the (symmetric, zero camber) lateral slip curve at load ``fz``."""
        p = self.p
        fz = np.maximum(np.asarray(fz, dtype=float), 1.0)
        dy = self.mu_y(fz) * fz
        b = self.cornering_stiffness(fz) / (self.cy * dy)
        e = np.minimum((p.get("PEY1", 0.0) + p.get("PEY2", 0.0) * self._dfz(fz)) * p["LEY"], 0.99)
        return b, e

    def _shape_x(self, fz):
        p = self.p
        fz = np.maximum(np.asarray(fz, dtype=float), 1.0)
        dx = self.mu_x(fz) * fz
        b = self.slip_stiffness(fz) / (self.cx * dx)
        dfz = self._dfz(fz)
        e = np.minimum((p.get("PEX1", 0.0) + p.get("PEX2", 0.0) * dfz + p.get("PEX3", 0.0) * dfz * dfz) * p["LEX"],
                       0.99)
        return b, e

    def _peak_z(self, c, e):
        """Normalised slip ``z = B x`` of the curve maximum (``inf`` for C <= 1)."""
        if c <= 1.0:
            return np.full(np.shape(e), np.inf)
        return _solve_z(np.full(np.shape(e), math.tan(math.pi / (2.0 * c))), e)

    def peak_slip_at(self, fz):
        """Slip angle of the peak lateral force at load ``fz`` (rad)."""
        b, e = self._shape_y(fz)
        z = self._peak_z(self.cy, e)
        return np.where(np.isfinite(z), z / b, 0.35)

    def peak_slip_ratio_at(self, fz):
        b, e = self._shape_x(fz)
        z = self._peak_z(self.cx, e)
        return np.where(np.isfinite(z), z / b, 0.3)

    def slip_angle(self, utilization, fz=None):
        """Slip angle of one tyre at ``Fy = u * Fy_peak`` (stable branch)."""
        fz = self.fz_ref if fz is None else fz
        u = np.clip(np.asarray(utilization, dtype=float), 0.0, 0.999999) * self._peak_y
        b, e = self._shape_y(fz)
        phi = np.tan(np.arcsin(u) / self.cy)
        return _solve_z(phi, e) / b

    def axle_slip_angle(self, utilization, fz_a=None, fz_b=None):
        """Common slip angle of an axle's two tyres (load-weighted mean of their slip angles at ``u``)."""
        if fz_a is None or fz_b is None:
            return self.slip_angle(utilization)
        if np.size(utilization) == 1 and np.size(fz_a) == 1 and np.size(fz_b) == 1:  # transient: scalar path
            u = min(max(float(np.ravel(utilization)[0]), 0.0), 0.999999) * self._peak_y
            phi = math.tan(math.asin(u) / self.cy)
            fa, fb = max(float(np.ravel(fz_a)[0]), 1.0), max(float(np.ravel(fz_b)[0]), 1.0)
            ba, ea = self._by_ey(fa)
            bb, eb = self._by_ey(fb)
            val = (fa * _solve_z1(phi, ea) / ba + fb * _solve_z1(phi, eb) / bb) / (fa + fb)
            shape = np.broadcast_shapes(np.shape(utilization), np.shape(fz_a), np.shape(fz_b))
            return val if shape == () else np.full(shape, val)
        fa = np.maximum(np.asarray(fz_a, dtype=float), 1.0)
        fb = np.maximum(np.asarray(fz_b, dtype=float), 1.0)
        return (fa * self.slip_angle(utilization, fa) + fb * self.slip_angle(utilization, fb)) / (fa + fb)

    def axle_peak_slip(self, fz_a=None, fz_b=None) -> float:
        if fz_a is None or fz_b is None:
            return self.peak_slip
        fa, fb = max(float(fz_a), 1.0), max(float(fz_b), 1.0)
        return (fa * self._peak_slip1(fa) + fb * self._peak_slip1(fb)) / (fa + fb)

    # ---- scalar fast path (transient solver: one call per axle and time step) ----------------
    def _by_ey(self, fz: float) -> tuple[float, float]:
        """By, Ey of the zero-camber lateral curve for one load (pure Python)."""
        kmu, d1, d2, mu_min, k1, k4, k2, e1, e2 = self._sc
        dfz = (fz - self.fz0) / self.fz0
        dy = max(kmu * (d1 + d2 * dfz), mu_min) * fz
        ky = k1 * abs(math.sin(k4 * math.atan(fz / k2))) if k2 != 0.0 else k1
        return ky / (self.cy * dy), min(e1 + e2 * dfz, 0.99)

    def _peak_slip1(self, fz: float) -> float:
        b, e = self._by_ey(fz)
        if self.cy <= 1.0:
            return 0.35
        key = round(e, 4)
        z = self._zpk_cache.get(key)
        if z is None:
            z = _solve_z1(math.tan(math.pi / (2.0 * self.cy)), e)
            self._zpk_cache[key] = z
        return z / b

    def _shape1(self, alpha: float, fz: float) -> float:
        b, e = self._by_ey(max(fz, 1.0))
        z = b * abs(alpha)
        y = math.sin(self.cy * math.atan(z - e * (z - math.atan(z)))) / self._peak_y
        return y if alpha >= 0.0 else -y

    def lateral_shape(self, alpha, fz):
        """Fy / Fy_peak of one tyre at slip angle ``alpha`` (odd; falls to sin(Cy pi/2) when sliding)."""
        b, e = self._shape_y(fz)
        z = b * np.abs(np.asarray(alpha, dtype=float))
        y = np.sin(self.cy * np.arctan(z - e * (z - np.arctan(z)))) / self._peak_y
        return np.sign(alpha) * y

    def axle_lateral_force(self, alpha: float, caps, fzs=None) -> float:
        if fzs is None:
            fzs = (self.fz_ref, self.fz_ref)
        a = float(alpha)
        return float(caps[0]) * self._shape1(a, float(fzs[0])) + float(caps[1]) * self._shape1(a, float(fzs[1]))

    # ------------------------------------------------------------------ curves for reports / plots
    def fy_pure(self, alpha, fz, inclination=0.0):
        """Lateral force (N, towards the side the slip pushes) of one tyre, camber thrust included."""
        fz = np.asarray(fz, dtype=float)
        dy = self.mu_y(fz, inclination) * fz
        return dy * self.lateral_shape(alpha, fz) * self._peak_y + self.camber_thrust(fz, inclination)

    def fx_pure(self, kappa, fz):
        fz = np.asarray(fz, dtype=float)
        b, e = self._shape_x(fz)
        z = b * np.abs(np.asarray(kappa, dtype=float))
        return np.sign(kappa) * self.mu_x(fz) * fz * np.sin(self.cx * np.arctan(z - e * (z - np.arctan(z))))

    def combined_forces(self, alpha, kappa, fz):
        """(|Fx|, |Fy|) of the MF combined-slip model (symmetric part), zero camber."""
        p = self.p
        fz = np.asarray(fz, dtype=float)
        a = np.abs(np.asarray(alpha, dtype=float))
        k = np.abs(np.asarray(kappa, dtype=float))
        dfz = self._dfz(fz)
        fx0 = np.abs(self.fx_pure(k, fz))
        fy0 = np.abs(self.fy_pure(a, fz))
        bxa = p.get("RBX1", 0.0) * np.cos(np.arctan(p.get("RBX2", 0.0) * k)) * p["LXAL"]
        exa = np.minimum(p.get("REX1", 0.0) + p.get("REX2", 0.0) * dfz, 0.99)
        gxa = np.cos(p.get("RCX1", 0.0) * np.arctan(bxa * a - exa * (bxa * a - np.arctan(bxa * a))))
        byk = p.get("RBY1", 0.0) * np.cos(np.arctan(p.get("RBY2", 0.0) * a)) * p["LYKA"]
        eyk = np.minimum(p.get("REY1", 0.0) + p.get("REY2", 0.0) * dfz, 0.99)
        gyk = np.cos(p.get("RCY1", 0.0) * np.arctan(byk * k - eyk * (byk * k - np.arctan(byk * k))))
        return np.maximum(gxa, 0.0) * fx0, np.maximum(gyk, 0.0) * fy0

    def has_combined_terms(self) -> bool:
        p = self.p
        return all(abs(p.get(k, 0.0)) > 0.0 for k in ("RBX1", "RCX1", "RBY1", "RCY1"))

    def fit_combined_exponent(self) -> float:
        """Exponent n of ``|Fx/Dx|^n + |Fy/Dy|^n = 1`` fitted to the MF combined-slip envelope (2 without terms)."""
        if not self.has_combined_terms():
            return 2.0
        from scipy.optimize import minimize_scalar

        fz = self.fz_ref
        a_pk = float(self.peak_slip_at(fz))
        k_pk = float(self.peak_slip_ratio_at(fz))
        a = np.linspace(0.0, 3.0 * a_pk, 90)
        k = np.linspace(0.0, 3.0 * k_pk, 90)
        A, K = np.meshgrid(a, k, indexing="ij")
        fx, fy = self.combined_forces(A, K, fz)
        x = (fx / (self.mu_x(fz) * fz * self._peak_x)).ravel()
        y = (fy / (self.mu_y(fz) * fz * self._peak_y)).ravel()
        r = np.hypot(x, y)
        th = np.arctan2(y, x)
        bins = np.linspace(0.0, 0.5 * math.pi, 31)
        idx = np.digitize(th, bins) - 1
        ex, ey = [], []
        for j in range(1, len(bins) - 2):  # leave out the pure-slip ends (they are 1 by definition)
            sel = np.flatnonzero(idx == j)
            if sel.size:
                m = sel[np.argmax(r[sel])]
                ex.append(x[m])
                ey.append(y[m])
        if len(ex) < 5:
            return 2.0
        ex, ey = np.maximum(np.array(ex), 1e-6), np.maximum(np.array(ey), 1e-6)

        def cost(n):
            return float(np.sum((np.log(ex ** n + ey ** n) / n) ** 2))

        res = minimize_scalar(cost, bounds=(1.2, 5.0), method="bounded")
        return float(min(max(res.x, 1.2), 5.0))

    def describe(self) -> str:
        fz = self.fz_ref
        lines = [f"Magic Formula tyre ({self.source}): FNOMIN {self.p['FNOMIN']:.0f} N"
                 + (f", pressure {self.pressure / 1000:.0f} kPa (nominal {self.nompres / 1000:.0f})"
                    if self.pressure and self.nompres else "")]
        mux, muy = float(self.mu_x(fz)), float(self.mu_y(fz))
        k2 = (float(self.mu_y(2.0 * fz)) / muy - 1.0) * 100.0
        lines.append(f"  at {fz:.0f} N: mu_x {mux:.3f}, mu_y {muy:.3f}"
                     + (f" (MF friction scaled x{self.kx:.3f} / x{self.ky:.3f} to the configured mu)"
                        if self.scale_mu else " (MF friction as in the file)")
                     + f"; mu_y at 2x load {k2:+.1f} %")
        lines.append(f"  cornering stiffness {float(self.cornering_stiffness(fz)) * math.pi / 180:.0f} N/deg, peak slip "
                     f"{math.degrees(float(self.peak_slip_at(fz))):.1f} deg ({math.degrees(float(self.peak_slip_at(0.5 * fz))):.1f}"
                     f" at half, {math.degrees(float(self.peak_slip_at(2.0 * fz))):.1f} at double load), sliding "
                     f"{self._sliding_ratio() * 100:.0f} % of peak")
        g_opt, gain = self.camber_optimum(fz)
        lines.append(f"  best lean towards the turn centre {math.degrees(g_opt):.1f} deg (+{gain * 100:.1f} % lateral "
                     f"capacity); combined-slip exponent {self.combined_exponent:.2f}"
                     + (" (fitted to the MF combined terms)" if self.combined_fitted and self.has_combined_terms()
                        else ""))
        if self.combined_exponent > 2.5:
            lines.append("  WARNING: combined-slip envelope well outside the friction ellipse - check the R* coefficients "
                         "or set tires.combined_exponent")
        if g_opt < 0.0:
            lines.append("  WARNING: leaning away from the turn helps this tyre - check the camber sign convention "
                         "(ISO: PVY3 < 0)")
        return "\n".join(lines)

    def _sliding_ratio(self) -> float:
        return math.sin(self.cy * math.pi / 2.0) / self._peak_y if self.cy > 1.0 else 1.0

    def camber_optimum(self, fz) -> tuple[float, float]:
        """Lean towards the turn centre (rad) with the largest lateral capacity, and the relative gain."""
        g = np.radians(np.linspace(-6.0, 10.0, 321))
        cap = self.mu_y(fz, g) * fz * self._peak_y + self.camber_thrust(fz, g)
        i = int(np.argmax(cap))
        base = float(self.mu_y(fz) * fz * self._peak_y)
        return float(g[i]), float(cap[i] / base - 1.0)


def _solve_z1(phi: float, e: float, iters: int = 8) -> float:
    """Scalar version of :func:`_solve_z`."""
    z = phi / max(1.0 - min(e, 0.0) * 0.5, 0.5)
    for _ in range(iters):
        f = (1.0 - e) * z + e * math.atan(z) - phi
        d = (1.0 - e) + e / (1.0 + z * z)
        z = max(z - f / d, 0.0)
    return z


def _solve_z(phi, e, iters: int = 8):
    """Solve (1 - E) z + E atan(z) = phi for z >= 0 (monotonic for E < 1), vectorised Newton."""
    phi = np.asarray(phi, dtype=float)
    e = np.asarray(e, dtype=float)
    z = phi / np.maximum(1.0 - np.minimum(e, 0.0) * 0.5, 0.5)
    for _ in range(iters):
        f = (1.0 - e) * z + e * np.arctan(z) - phi
        d = (1.0 - e) + e / (1.0 + z * z)
        z = np.maximum(z - f / d, 0.0)
    return z
