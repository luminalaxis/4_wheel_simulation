"""Road surface profiles (left / right wheel tracks) for the transient solver.

* :meth:`RoadProfile.iso8608` - random profile with the ISO 8608 displacement PSD
  ``Gd(n) = Gd(n0) (n / n0)^-2`` (``n0`` = 0.1 cycles/m) for classes A (very good) to D (poor).
  Wavelengths shorter than ``1/n_max`` (default 0.33 m) are left out: the tyre contact patch
  envelopes them.
  A tarmac FSAE site is typically class A-B. Left and right tracks are correlated at long
  wavelengths (coherence ``exp(-n / n_c)``).
* :meth:`RoadProfile.from_csv` - measured profile ``s_m, z_left_mm, z_right_mm`` (or ``z_mm``).

Heights are positive upwards, in metres. Closed tracks wrap around.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

ISO_CLASSES = {"A": 16e-6, "B": 64e-6, "C": 256e-6, "D": 1024e-6, "E": 4096e-6}  # Gd(n0), m^3
N0 = 0.1  # cycles/m


class RoadProfile:
    def __init__(self, s: np.ndarray, z_left: np.ndarray, z_right: np.ndarray, periodic: bool, name: str = "road"):
        self.s = np.asarray(s, dtype=float)
        self.z_left = np.asarray(z_left, dtype=float)
        self.z_right = np.asarray(z_right, dtype=float)
        self.periodic = periodic
        self.name = name
        self.length = float(self.s[-1])
        self.ds = float(self.s[1] - self.s[0])
        self._zl = self.z_left.tolist()
        self._zr = self.z_right.tolist()
        self._n = len(self._zl)

    # ------------------------------------------------------------------ builders
    @classmethod
    def flat(cls, length: float) -> "RoadProfile":
        return cls(np.array([0.0, max(length, 1.0)]), np.zeros(2), np.zeros(2), True, "flat")

    @classmethod
    def iso8608(cls, length: float, road_class: str = "A", seed: int = 0, periodic: bool = True,
                n_min: float = 0.02, n_max: float = 3.0, coherence_n: float = 0.3, scale: float = 1.0,
                ds: float = 0.02) -> "RoadProfile":
        """Random profile of an ISO 8608 class over ``length`` metres (periodic if ``periodic``)."""
        key = str(road_class).upper()
        if key not in ISO_CLASSES:
            raise ValueError(f"road class must be one of {', '.join(ISO_CLASSES)}")
        gd0 = ISO_CLASSES[key] * scale ** 2
        rng = np.random.default_rng(seed)
        span = length if periodic else length + 40.0  # extra length for open tracks (car overhang)
        n_pts = int(2 ** math.ceil(math.log2(max(span / ds, 64))))
        dn = 1.0 / span
        freq = np.arange(n_pts // 2 + 1) * dn
        band = (freq >= n_min) & (freq <= n_max)
        gd = np.zeros_like(freq)
        gd[band] = gd0 * (freq[band] / N0) ** -2
        amp = np.sqrt(2.0 * gd * dn)  # amplitude of each cosine

        def spectrum():
            phase = rng.uniform(0.0, 2.0 * np.pi, freq.size)
            return amp * np.exp(1j * phase)

        a_left, a_ind = spectrum(), spectrum()
        rho = np.exp(-freq / coherence_n)
        a_right = rho * a_left + np.sqrt(1.0 - rho ** 2) * a_ind
        # sum_k amp_k cos(2 pi n_k s + phi_k) == irfft of (amp/2 * N) spectrum
        z_l = np.fft.irfft(a_left * n_pts / 2.0, n_pts)
        z_r = np.fft.irfft(a_right * n_pts / 2.0, n_pts)
        s = np.arange(n_pts + 1) * (span / n_pts)
        z_l = np.append(z_l, z_l[0])
        z_r = np.append(z_r, z_r[0])
        return cls(s, z_l, z_r, periodic, f"ISO 8608 class {key} (seed {seed})")

    @classmethod
    def from_csv(cls, path: str | Path, periodic: bool = False, ds: float = 0.02) -> "RoadProfile":
        with open(path, encoding="utf-8") as fh:
            lines = [ln for ln in fh if ln.strip() and not ln.lstrip().startswith("#")]
        rows = list(csv.DictReader(lines))
        cols = {c.strip().lower(): c for c in rows[0]}
        s = np.array([float(r[cols["s_m"]]) for r in rows])
        if "z_left_mm" in cols:
            zl = np.array([float(r[cols["z_left_mm"]]) for r in rows]) / 1000.0
            zr = np.array([float(r[cols["z_right_mm"]]) for r in rows]) / 1000.0
        elif "z_mm" in cols:
            zl = zr = np.array([float(r[cols["z_mm"]]) for r in rows]) / 1000.0
        else:
            raise ValueError(f"{path}: road profile needs s_m and z_mm or z_left_mm/z_right_mm columns")
        s = s - s[0]
        grid = np.arange(0.0, s[-1] + 0.5 * ds, ds)
        return cls(grid, np.interp(grid, s, zl), np.interp(grid, s, zr), periodic, Path(path).stem)

    @classmethod
    def from_spec(cls, spec: str | None, length: float, periodic: bool, seed: int = 0,
                  scale: float = 1.0) -> "RoadProfile | None":
        """``None``/"flat" -> None, "A".."E" -> ISO class, otherwise a CSV path."""
        if spec is None or str(spec).lower() in ("", "none", "flat", "smooth"):
            return None
        if str(spec).upper() in ISO_CLASSES:
            return cls.iso8608(length, spec, seed=seed, periodic=periodic, scale=scale)
        return cls.from_csv(spec, periodic=periodic)

    # ------------------------------------------------------------------ lookup
    def at(self, s: float) -> tuple[float, float, float, float]:
        """Heights and slopes (dz/ds) of the left and right track at ``s``."""
        if self.periodic:
            s = s % self.length
        x = s / self.ds
        if x <= 0.0:
            return self._zl[0], self._zr[0], 0.0, 0.0
        if x >= self._n - 1:
            return self._zl[-1], self._zr[-1], 0.0, 0.0
        i = int(x)
        t = x - i
        zl0, zl1 = self._zl[i], self._zl[i + 1]
        zr0, zr1 = self._zr[i], self._zr[i + 1]
        return (zl0 + (zl1 - zl0) * t, zr0 + (zr1 - zr0) * t, (zl1 - zl0) / self.ds, (zr1 - zr0) / self.ds)

    def rms(self) -> float:
        return float(np.sqrt(0.5 * (np.mean(self.z_left ** 2) + np.mean(self.z_right ** 2))))
