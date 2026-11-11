"""Multi-element wing sections from the geometry of each element (main + flap1 + flap2 ...).

A wing in ``aero.components`` with an ``elements`` mapping is described element by element, with
the conventions of AERO_AUTOMATION (https://github.com/luminalaxis/AERO_AUTOMATION)::

    elements:
      main:  {chord_mm: 175, airfoil: "4412"}                       # angle = the wing's aoa_deg
      flap1: {chord_mm: 80, deflection_deg: 15, gap_mm: 6, overlap_mm: 5, airfoil: "4412"}
      flap2: {chord_mm: 60, deflection_deg: 35, gap_mm: 5, overlap_mm: 3}

* ``aoa_deg`` of the wing = angle of the MAIN element to the car's horizontal,
* ``deflection_deg`` of a flap = its angle to the MAIN chord line (not to the flap before it),
* ``gap_mm`` / ``overlap_mm`` = slot to the element before it,
* coefficients per main chord, downforce positive (as ``cl`` in ``summary.csv``).

The 2-D model (this module) gives cl / cd of the section; :class:`aerolap.aero.devices.Wing` turns
it into the 3-D wing with the same lifting-line / ground-effect path as 2-D CFD data. Per element:

* lift: ``cl_i = cl0_i + a (k_i alpha + eta_i (delta_i - delta_{i-1}))`` with the camber lift
  ``cl0`` (NACA 4-digit ``airfoil`` or ``camber``), ``a = 0.9 * 2 pi``; a flap sees part of the
  angle of attack (``k = 0.6``) and part of its turning relative to the element before it
  (``eta = 0.6 (0.5 + 0.5 q)``, ``q`` = slot quality, 1 at the best gap / overlap),
* slot quality ``q``: best gap 3.5 %, best overlap 2 % of the main chord; away from them the
  flap turns the flow less and stalls earlier,
* circulation effect: the flaps' lift raises the main element's lift (``+ 0.35 sum c_f/c_main cl_f``)
  and its maximum lift (``x (1 + 0.6 sum q c_f/c_main)``) - why a multi-element wing stalls later,
* stall per element (smooth roll-over at ``cl_max``, drop past it): a too-large flap angle
  separates the flap, a too-large aoa stalls the main element,
* drag: profile drag per element (thickness, lift), separation drag past stall, slot drag,
* ground effect (when the wing runs close to the ground): ``1 + 0.16 / sqrt(h / c_main)``,
  ground stall below 10 % of the main chord and an earlier stall close to the ground.

The numbers are a physically shaped engineering model, not CFD: use them to compare element
angles / gaps and to set up the car, and replace them with AERO_AUTOMATION results
(``section_data``) once those exist - the same element values then pick the CFD geometry.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

A2D = 0.9 * 2.0 * math.pi  # 2-D lift slope per rad (viscous)
AOA_K_FLAP = 0.6  # share of the angle of attack a flap sees
GAP_BEST, GAP_WIDTH = 0.035, 0.025  # per main chord
OVERLAP_BEST, OVERLAP_WIDTH = 0.02, 0.03
AOA_GRID = np.arange(-15.0, 30.0001, 0.5)


def naca4(code: str) -> tuple[float, float]:
    """(max camber, thickness) of a NACA 4-digit aerofoil, e.g. '4412' -> (0.04, 0.12)."""
    s = str(code).strip().upper().removeprefix("NACA").strip()
    if s.endswith(".0"):  # read from a CSV as a number
        s = s[:-2]
    if len(s) != 4 or not s.isdigit():
        raise ValueError(f"airfoil {code!r}: only NACA 4-digit codes (e.g. '4412') are known - "
                         "give camber (max camber / chord, e.g. 0.04) and thickness instead")
    return int(s[0]) / 100.0, int(s[2:]) / 100.0


@dataclass(frozen=True)
class Element:
    name: str
    chord_mm: float
    deflection: float = 0.0  # deg to the main chord line (main element: 0)
    gap_mm: float = 0.0
    overlap_mm: float = 0.0
    camber: float = 0.04
    thickness: float = 0.12
    airfoil: str = ""

    @property
    def cl0(self) -> float:  # thin-aerofoil camber lift (4412 -> 0.47)
        return 11.75 * self.camber

    @property
    def cl_max_base(self) -> float:  # single element, clean (4412 -> 1.54)
        return 1.30 + 6.0 * self.camber

    @property
    def cd0(self) -> float:
        return 0.006 + 0.03 * self.thickness


@dataclass(frozen=True)
class ElementSection:
    """2-D multi-element section: per-element geometry -> cl, cd (per main chord)."""

    elements: tuple[Element, ...]

    @property
    def c0(self) -> float:
        return self.elements[0].chord_mm

    @property
    def flaps(self) -> tuple[Element, ...]:
        return self.elements[1:]

    def slot_quality(self, i: int) -> float:
        e = self.elements[i]
        g = (e.gap_mm / self.c0 - GAP_BEST) / GAP_WIDTH
        o = (e.overlap_mm / self.c0 - OVERLAP_BEST) / OVERLAP_WIDTH
        return (0.4 + 0.6 * math.exp(-0.5 * g * g)) * (0.85 + 0.15 * math.exp(-0.5 * o * o))

    def _local(self, aoa_deg):
        """Per element (cl_lin, cl, cl_max, over) on its own chord; main element first."""
        from .devices import _soft_stall

        a = np.radians(np.asarray(aoa_deg, dtype=float))
        flaps, boost, dump = [], 0.0, 0.0
        prev = 0.0
        for i, f in enumerate(self.flaps, 1):
            q = self.slot_quality(i)
            eta = 0.6 * (0.5 + 0.5 * q)
            lin = f.cl0 + A2D * (AOA_K_FLAP * a + eta * math.radians(f.deflection - prev))
            cmax = f.cl_max_base * 1.3 * (0.7 + 0.3 * q)
            cl, over = _soft_stall(lin, cmax)
            flaps.append((lin, cl, cmax, over))
            r = f.chord_mm / self.c0
            boost = boost + 0.35 * r * cl
            dump += 0.6 * r * q
            prev = f.deflection
        m = self.elements[0]
        lin = m.cl0 + A2D * a + boost
        cmax = m.cl_max_base * (1.0 + dump)
        cl, over = _soft_stall(lin, cmax)
        return [(lin, cl, cmax, over)] + flaps

    def _ground(self, h_mm, x_main):
        hc = np.maximum(np.asarray(h_mm, dtype=float), 1.0) / self.c0
        g = 1.0 + 0.16 / np.sqrt(hc)
        g = g * (1.0 - 2.5 * np.maximum(0.10 - hc, 0.0))  # ground stall
        g = g * (1.0 - 0.6 * np.maximum(x_main - 0.9, 0.0) * np.sqrt(0.25 / hc))  # earlier stall near the ground
        return np.maximum(g, 0.2)

    def coefficients(self, aoa_deg, h_mm=None):
        """(cl, cd) per main chord at angle(s) ``aoa_deg`` and ground clearance(s) ``h_mm`` (None = free stream)."""
        loc = self._local(aoa_deg)
        g = 1.0 if h_mm is None else self._ground(h_mm, loc[0][0] / loc[0][2])
        cl = cd = 0.0
        for i, (e, (_, c, _, over)) in enumerate(zip(self.elements, loc)):
            r = e.chord_mm / self.c0
            cl = cl + r * c
            cd = cd + r * (e.cd0 + 0.008 * (c * g) ** 2 + 0.35 * over ** 2)
            if i:
                cd = cd + 0.002 * (2.0 - self.slot_quality(i))
        return cl * g, cd

    def states(self, aoa_deg: float) -> list[dict]:
        """Per element at one angle: own-chord cl, its maximum, the stall ratio and its angle to the car."""
        out = []
        for e, (lin, c, cmax, _) in zip(self.elements, self._local(aoa_deg)):
            out.append({"name": e.name, "angle_deg": float(aoa_deg) + e.deflection, "cl": float(c),
                        "cl_max": float(cmax), "ratio": float(lin / cmax)})
        return out

    def params(self) -> dict:
        """The element values under AERO_AUTOMATION's ``summary.csv`` column names."""
        m = self.elements[0]
        p = {"main_airfoil": m.airfoil or None, "main_chord": m.chord_mm}
        for i, f in enumerate(self.flaps, 1):
            p.update({f"flap{i}_airfoil": f.airfoil or None, f"flap{i}_chord": f.chord_mm,
                      f"flap{i}_deflection": f.deflection, f"flap{i}_gap": f.gap_mm, f"flap{i}_overlap": f.overlap_mm})
        return p

    def label(self) -> str:
        m = self.elements[0]
        parts = [f"{len(self.elements)}el", f"{m.airfoil or 'c'}c{m.chord_mm:g}"]
        parts += [f"f{f.chord_mm:g}d{f.deflection:g}g{f.gap_mm:g}o{f.overlap_mm:g}" for f in self.flaps]
        return "_".join(parts)


@lru_cache(maxsize=256)
def element_polar(section: ElementSection, ground: bool, h_static: float):
    """The section as a :class:`SectionPolar` (aoa [x clearance] grid) for the 3-D wing path."""
    from .section_data import SectionPolar

    if ground:
        hs = np.geomspace(0.05 * section.c0, max(8.0 * section.c0, 3.0 * h_static + 50.0), 60)
        a, h = np.meshgrid(AOA_GRID, hs, indexing="ij")
        cl, cd = section.coefficients(a, h)
    else:
        hs = None
        cl, cd = section.coefficients(AOA_GRID)
    pol = SectionPolar(section.label(), section.params(), AOA_GRID.copy(), hs, None, np.asarray(cl, dtype=float),
                       np.asarray(cd, dtype=float), section.c0 / 1000.0, "element model", 0)
    pol.model = section
    return pol


ELEMENT_KEYS = {"chord_mm", "deflection_deg", "gap_mm", "overlap_mm", "airfoil", "camber", "thickness"}


def parse_elements(raw, pre: str) -> ElementSection:
    """``elements`` mapping (main, flap1, flap2 ...) -> :class:`ElementSection`; errors name the key."""
    from ..config import ConfigError

    if isinstance(raw, list):
        raw = {("main" if i == 0 else f"flap{i}"): e for i, e in enumerate(raw)}
    if not isinstance(raw, dict) or not raw:
        raise ConfigError(path=pre, value=raw, condition="main, flap1, flap2 ... 을 키로 하는 매핑",
                          hint="예: elements: {main: {chord_mm: 175}, flap1: {chord_mm: 80, deflection_deg: 15, "
                               "gap_mm: 6, overlap_mm: 5}}")
    names = list(raw)
    want = ["main"] + [f"flap{i}" for i in range(1, len(names))]
    if names != want:
        raise ConfigError(path=pre, value=", ".join(map(str, names)), condition="순서대로 " + ", ".join(want),
                          hint="첫 엘리먼트는 main, 그 뒤는 flap1, flap2 ... (앞에서 뒤로)")
    out = []
    for i, name in enumerate(names):
        e = raw[name] or {}
        p = f"{pre}.{name}"
        if not isinstance(e, dict):
            raise ConfigError(path=p, value=e, condition="chord_mm 등을 가진 매핑")
        bad = [k for k in e if k not in ELEMENT_KEYS]
        if bad:
            raise ConfigError(path=f"{p}.{bad[0]}", value=e[bad[0]], condition="알려진 키: " + ", ".join(sorted(ELEMENT_KEYS)))
        if i == 0 and "deflection_deg" in e:
            raise ConfigError(path=f"{p}.deflection_deg", value=e["deflection_deg"],
                              condition="main 엘리먼트에는 없음", hint="메인 각도는 윙의 aoa_deg (차체 수평 기준)입니다")
        vals = {}
        for k in ("chord_mm", "deflection_deg", "gap_mm", "overlap_mm", "camber", "thickness"):
            if k in e:
                try:
                    vals[k] = float(e[k])
                except (TypeError, ValueError):
                    raise ConfigError(path=f"{p}.{k}", value=e[k], condition="숫자여야 함") from None
        if "chord_mm" not in vals:
            raise ConfigError(path=f"{p}.chord_mm", has_value=False, condition="엘리먼트마다 필수 (mm)")
        if vals["chord_mm"] <= 0:
            raise ConfigError(path=f"{p}.chord_mm", value=e["chord_mm"], condition="> 0 mm")
        if i and not 0.0 <= vals.get("deflection_deg", 0.0) <= 90.0:
            raise ConfigError(path=f"{p}.deflection_deg", value=e["deflection_deg"], condition="0 .. 90 deg (메인 코드선 기준)")
        if i and vals.get("gap_mm", 0.0) < 0:
            raise ConfigError(path=f"{p}.gap_mm", value=e["gap_mm"], condition=">= 0 mm")
        if i and "deflection_deg" not in vals:
            raise ConfigError(path=f"{p}.deflection_deg", has_value=False, condition="플랩마다 필수 (메인 코드선 기준 deg)")
        airfoil = "" if e.get("airfoil") in (None, "") else str(e["airfoil"])
        camber, thick = 0.04, 0.12
        if airfoil:
            try:
                camber, thick = naca4(airfoil)
            except ValueError as err:
                if "camber" not in vals:
                    raise ConfigError(str(err), path=f"{p}.airfoil", value=airfoil, condition="NACA 4자리 (예: 4412)") from None
        camber, thick = vals.get("camber", camber), vals.get("thickness", thick)
        c0 = out[0].chord_mm if out else vals["chord_mm"]  # missing slot values: the best ones
        out.append(Element(name, vals["chord_mm"], vals.get("deflection_deg", 0.0) if i else 0.0,
                           vals.get("gap_mm", GAP_BEST * c0) if i else 0.0,
                           vals.get("overlap_mm", OVERLAP_BEST * c0) if i else 0.0, camber, thick, airfoil))
    return ElementSection(tuple(out))


def select_cfd(sset, section: ElementSection):
    """The geometry of an AERO_AUTOMATION file that matches the element values (error lists the closest)."""
    want = section.params()
    n_el = len(section.elements)
    keys = list(sset.keys)

    def value_ok(k, have):
        if k not in want:  # an element this wing does not have
            return have in (None, "")
        w = want[k]
        if w is None:  # airfoil not given: any
            return True
        try:
            return abs(float(have) - float(w)) < 1e-6
        except (TypeError, ValueError):
            return str(have).strip() == str(w).strip()

    scored = []
    for s in sset.sections.values():
        diff = [k for k in keys if k != "case_set" and not value_ok(k, s.params.get(k))]
        scored.append((len(diff), s, diff))
    hits = [s for n, s, _ in scored if n == 0]
    if len(hits) == 1:
        return hits[0]
    if len(hits) > 1:
        raise ValueError(f"the element values match {len(hits)} geometries of {sset.path.name} "
                         f"({', '.join(h.geometry_id for h in hits[:4])}); add 'section: <id>' to pick one")
    scored.sort(key=lambda t: t[0])
    near = "; ".join(f"{s.geometry_id} (differs in " + ", ".join(f"{k}={s.params.get(k)!r} vs {want.get(k)!r}"
                                                             for k in d[:3]) + ")" for _, s, d in scored[:3])
    raise ValueError(f"no geometry of {sset.path.name} has these {n_el}-element values "
                     f"({section.label()}); closest: {near}")
