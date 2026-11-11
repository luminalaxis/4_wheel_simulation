"""Active aero / DRS strategies for the quasi-steady-state solver.

* ``fixed``  – one aero mode for the whole lap.
* ``rule``   – the flap opens on (near) straights longer than ``min_length_m`` whose
  radius exceeds ``open_radius_m``, never while braking or at a cornering limit, and
  only above ``min_speed_kmh`` (at low speed the rear downforce is worth more for
  traction than the drag saving), and needs ``open_delay_s`` / ``close_delay_s`` to move. While moving it is treated as
  still open (low downforce), which is conservative.
* ``ideal``  – at every point the mode that gives the best acceleration, braking or
  cornering limit is used (upper bound of what active aero can gain).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StrategyConfig:
    type: str
    open_mode: int
    closed_mode: int
    open_radius: float
    min_length: float
    min_speed: float
    open_delay: float
    close_delay: float
    close_anticipation: bool
    max_iterations: int

    @classmethod
    def from_config(cls, s: dict, aero_model) -> "StrategyConfig":
        closed = aero_model.mode_index(s.get("closed_mode"), aero_model.default_mode)
        others = [i for i in range(len(aero_model.mode_names)) if i != closed]
        open_default = others[0] if others else closed
        opened = aero_model.mode_index(s.get("open_mode"), open_default)
        kind = s.get("type", "fixed")
        if kind != "fixed" and opened == closed:
            from ..config import ConfigError

            raise ConfigError(path="aero.strategy.type", value=kind,
                              condition="액티브 에어로(rule / ideal)는 aero map에 모드가 2개 이상이어야 함",
                              hint="aero.strategy.type을 fixed로 두거나 DRS 모드가 있는 aero map을 쓰세요")
        return cls(
            type=kind,
            open_mode=opened,
            closed_mode=closed,
            open_radius=float(s["open_radius_m"]),
            min_length=float(s["min_length_m"]),
            min_speed=float(s.get("min_speed_kmh", 0.0)) / 3.6,
            open_delay=float(s["open_delay_s"]),
            close_delay=float(s["close_delay_s"]),
            close_anticipation=bool(s["close_anticipation"]),
            max_iterations=int(s["max_iterations"]),
        )

    @classmethod
    def fixed(cls, mode: int) -> "StrategyConfig":
        return cls("fixed", mode, mode, np.inf, np.inf, 0.0, 0.0, 0.0, False, 1)

    # ------------------------------------------------------------------ rule
    def candidate_mask(self, kappa: np.ndarray, s: np.ndarray) -> np.ndarray:
        """Geometric eligibility: runs of radius > open_radius at least min_length long."""
        elig = np.abs(kappa) < 1.0 / self.open_radius
        out = np.zeros_like(elig)
        for i0, i1 in _runs(elig):
            if s[i1 - 1] - s[i0] >= self.min_length:
                out[i0:i1] = True
        return out

    def rule_modes(self, candidate: np.ndarray, v: np.ndarray, braking: np.ndarray,
                   apex: np.ndarray, s: np.ndarray) -> np.ndarray:
        """Mode per node given the latest speed profile."""
        dt = 2.0 * np.diff(s) / np.maximum(v[:-1] + v[1:], 1e-3)
        t = np.concatenate([[0.0], np.cumsum(dt)])
        want_open = candidate & ~braking & ~apex & (v >= self.min_speed)
        is_open = np.zeros_like(want_open)
        for i0, i1 in _runs(want_open):
            t_ready = t[i0] + self.open_delay
            is_open[i0:i1] = t[i0:i1] >= t_ready
        if not self.close_anticipation and self.close_delay > 0.0:
            # The flap only starts closing when the driver brakes: it is still (partly) open
            # for close_delay seconds into the braking zone.
            start_brake = np.flatnonzero(braking[1:] & ~braking[:-1]) + 1
            for k in start_brake:
                if not is_open[k - 1]:
                    continue
                j = k
                while j < len(t) and t[j] - t[k] < self.close_delay:
                    is_open[j] = True
                    j += 1
        return np.where(is_open, self.open_mode, self.closed_mode).astype(int)


def _runs(mask: np.ndarray):
    """Yield (start, stop) index pairs of contiguous True runs."""
    m = np.concatenate([[False], np.asarray(mask, dtype=bool), [False]])
    d = np.diff(m.astype(np.int8))
    starts = np.flatnonzero(d == 1)
    stops = np.flatnonzero(d == -1)
    return list(zip(starts.tolist(), stops.tolist()))
