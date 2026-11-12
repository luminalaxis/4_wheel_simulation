"""The aero design space of a car: every parameter of every device, and the on / off options.

``aero-split`` optimises the angles (and scales). This module lists *everything* the aero package
is made of, so that ``auto-aero`` can free all of it except what the user wants to keep:

continuous variables (group in brackets)
    wing      ``aoa_deg``, flap ``deflection_deg`` [angle]; ``gap_mm`` / ``overlap_mm`` [slot];
              element ``chord_mm`` or ``chord_m`` [chord]; ``span_m`` [size];
              ``height_mm`` / ``x_mm`` [position]
    floor     ``scale`` [size]; ``height_mm`` / ``x_mm`` [position]
    sidepod   ``exit_area_m2`` / ``core_area_m2`` [cooling]
    simple    ``scale`` [size]

options (on / off) [option]
    ``<device>.enabled`` (wings, floors, simple components - a sidepod only when asked for by name)
    and ``<wing>.elements.<flap>.enabled``.

Names are the config paths without ``aero.components.``: ``front.aoa_deg``,
``rear.elements.flap2.gap_mm``, ``side.enabled``.

Keeping parameters ("fixed") - patterns, from ``--fix`` and from the vehicle file::

    aero:
      design:
        fixed: [rear, "*.span_m", front.elements.main.chord_mm, "@position"]
        bounds: {front.height_mm: [50, 120], rear.elements.flap1.gap_mm: [4, 10]}
        options: false        # never search the on / off options (or a list: [sidepod.enabled] adds that one)

A pattern is a full name, a device (``rear`` = everything of the rear wing), a shell pattern
(``*.span_m``, ``front.elements.*``), the last key alone (``x_mm``), the ``aero-split`` name of a
flap (``front.flap1``) or a group (``@angle @slot @chord @size @position @cooling @option``).
A pattern that matches nothing is an error - a misspelt name must not silently free a parameter.

Default ranges (override with ``bounds``): angles - the device ``range`` / flap ``range``;
chords +-25 %; gap 1 .. 7 % and overlap -1 .. 6 % of the main chord; span 80 .. 100 % (the present
span is taken as the rules / packaging limit); height 70 .. 130 %; x +-100 mm; radiator core
+-30 %. They are engineering defaults, not the rules: check them for your car.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from fnmatch import fnmatch

from ..aero.devices import _enabled
from ..aero.maps import parse_components
from ..config import get_path

PREFIX = "aero.components."
GROUPS = ("angle", "slot", "chord", "size", "position", "cooling", "option")


@dataclass
class DesignVar:
    name: str
    lo: float
    hi: float
    value: float
    unit: str
    group: str
    fixed: str = ""  # why it is kept ("" = free)

    @property
    def path(self) -> str:
        return PREFIX + self.name

    @property
    def device(self) -> str:
        return self.name.split(".")[0]


@dataclass
class Option:
    name: str  # ...enabled
    value: bool
    fixed: str = ""
    group: str = "option"

    @property
    def path(self) -> str:
        return PREFIX + self.name

    @property
    def device(self) -> str:
        return self.name.split(".")[0]


@dataclass
class DesignSpace:
    variables: list[DesignVar] = field(default_factory=list)
    options: list[Option] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def free(self) -> list[DesignVar]:
        return [v for v in self.variables if not v.fixed]

    @property
    def free_options(self) -> list[Option]:
        return [o for o in self.options if not o.fixed]

    def table(self) -> str:
        lines = [f"{'parameter':44s} {'group':9s} {'value':>10s} {'range':>22s}  state"]
        for v in self.variables:
            lines.append(f"{v.name:44s} {v.group:9s} {v.value:10.4g} {'[' + f'{v.lo:.4g}, {v.hi:.4g}' + ']':>22s}  "
                         + (f"kept ({v.fixed})" if v.fixed else "free"))
        for o in self.options:
            lines.append(f"{o.name:44s} {'option':9s} {'on' if o.value else 'off':>10s} {'[off, on]':>22s}  "
                         + (f"kept ({o.fixed})" if o.fixed else "free"))
        return "\n".join(lines + self.notes)


def _short(name: str) -> str:
    name = name.strip()
    return name[len(PREFIX):] if name.startswith(PREFIX) else name


def matches(name: str, group: str, pattern: str) -> bool:
    """Does a fix / bounds pattern select this parameter? (see the module text)"""
    pat = _short(pattern)
    if pat.startswith("@"):
        return pat[1:] == group
    plain = name.replace(".elements.", ".")  # aero-split style: front.flap1
    return (name == pat or fnmatch(name, pat) or name.startswith(pat + ".") or name.rsplit(".", 1)[-1] == pat
            or plain.startswith(pat + ".") or fnmatch(plain, pat))


def _around(value: float, lo: float, hi: float) -> tuple[float, float]:
    lo, hi = min(lo, hi), max(lo, hi)
    return min(lo, value), max(hi, value)


def design_space(cfg: dict, fixed: list[str] | None = None, bounds: dict | None = None,
                 options: bool = True) -> DesignSpace:
    """All aero design parameters of ``cfg`` with their ranges; ``fixed`` / ``bounds`` add to ``aero.design``."""
    aero = cfg.get("aero") or {}
    comps = aero.get("components") or {}
    design = aero.get("design") or {}
    space = DesignSpace()
    devices = {c.name: getattr(c, "dev", c) for c in parse_components(comps, cfg.get("_base_dir", "."))}

    def add(name, lo, hi, unit, group):
        value = get_path(cfg, PREFIX + name, None)
        if isinstance(value, (int, float)) and not isinstance(value, bool) and hi > lo:
            space.variables.append(DesignVar(name, float(lo), float(hi), float(value), unit, group))

    for n, c in comps.items():
        if not isinstance(c, dict):
            continue
        kind = str(c.get("type", "")).lower()
        asked = design.get("options") if isinstance(design.get("options"), (list, tuple)) else []
        if kind != "sidepod" or any(_short(str(p)) == f"{n}.enabled" for p in asked):
            space.options.append(Option(f"{n}.enabled", _enabled(c)))
        dev = devices.get(n)
        if dev is None:  # switched off: its parameters are not on the car
            continue
        for _suffix, key, rng, unit, _val in dev.design_variables():
            group = "angle" if unit == "deg" else ("cooling" if kind == "sidepod" else "size")
            add(f"{n}.{key}", rng[0], rng[1], unit, group)
        if kind == "wing":
            el = c.get("elements") if isinstance(c.get("elements"), dict) else None
            if el is not None and dev.cfd:
                space.notes.append(f"note: {n} uses one CFD geometry ({dev.section.geometry_id}); its element values "
                                   "are kept - add 'section_fit: true' to move them over the CFD data, or use "
                                   "rank-sections to compare the CFD geometries")
            elif el is not None:
                c0 = float(el["main"]["chord_mm"])
                for name, e in el.items():
                    if not isinstance(e, dict):
                        continue
                    if name != "main":
                        space.options.append(Option(f"{n}.elements.{name}.enabled", _enabled(e)))
                    if not _enabled(e):
                        continue
                    ch = float(e["chord_mm"])
                    add(f"{n}.elements.{name}.chord_mm", 0.75 * ch, 1.25 * ch, "mm", "chord")
                    if name != "main":
                        if isinstance(e.get("gap_mm"), (int, float)):
                            add(f"{n}.elements.{name}.gap_mm", *_around(e["gap_mm"], max(0.01 * c0, 1.0), 0.07 * c0),
                                "mm", "slot")
                        if isinstance(e.get("overlap_mm"), (int, float)):
                            add(f"{n}.elements.{name}.overlap_mm", *_around(e["overlap_mm"], -0.01 * c0, 0.06 * c0),
                                "mm", "slot")
            elif not c.get("section_data") and isinstance(c.get("chord_m"), (int, float)):
                add(f"{n}.chord_m", 0.75 * c["chord_m"], 1.25 * c["chord_m"], "m", "chord")
            if isinstance(c.get("span_m"), (int, float)):
                add(f"{n}.span_m", 0.8 * c["span_m"], c["span_m"], "m", "size")
        if kind == "sidepod" and isinstance(c.get("core_area_m2"), (int, float)):
            add(f"{n}.core_area_m2", 0.7 * c["core_area_m2"], 1.3 * c["core_area_m2"], "m2", "cooling")
        if kind in ("wing", "floor"):
            if isinstance(c.get("height_mm"), (int, float)):
                add(f"{n}.height_mm", max(0.7 * c["height_mm"], 20.0), max(1.3 * c["height_mm"], 25.0), "mm", "position")
            if isinstance(c.get("x_mm"), (int, float)):
                add(f"{n}.x_mm", c["x_mm"] - 100.0, c["x_mm"] + 100.0, "mm", "position")
    if not options or design.get("options") is False:
        for o in space.options:
            o.fixed = "options not searched"

    all_bounds = {**(design.get("bounds") or {}), **(bounds or {})}
    for pat, rng in all_bounds.items():
        hits = [v for v in space.variables if matches(v.name, v.group, str(pat))]
        if not hits:
            raise ValueError(f"aero design bounds {pat!r} match no parameter; have "
                             + ", ".join(v.name for v in space.variables))
        try:
            lo, hi = float(rng[0]), float(rng[1])
        except (TypeError, ValueError, IndexError):
            raise ValueError(f"aero design bounds {pat!r}: need [lo, hi], got {rng!r}") from None
        if hi <= lo:
            raise ValueError(f"aero design bounds {pat!r}: upper bound {hi:g} must exceed lower bound {lo:g}")
        for v in hits:
            v.lo, v.hi = lo, hi
    items = space.variables + space.options
    for src, pats in (("vehicle file", design.get("fixed") or []), ("--fix", fixed or [])):
        for pat in pats:
            hits = [x for x in items if matches(x.name, x.group, str(pat))]
            if not hits:
                raise ValueError(f"fixed parameter {pat!r} ({src}) matches no aero design parameter; have "
                                 + ", ".join(x.name for x in items) + "; groups " + ", ".join("@" + g for g in GROUPS))
            for x in hits:
                x.fixed = x.fixed or f"{src}: {pat}"
    return space
