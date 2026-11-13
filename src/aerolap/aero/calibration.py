"""Element model corrected with 2-D CFD: a wing section that is continuous in its geometry.

AERO_AUTOMATION (``runs/summary.csv``) gives cl / cd of a set of discrete geometries; the element
model (:mod:`aerolap.aero.elements`) gives cl / cd of any geometry, with the right trends but not
the right numbers. ``section_fit: true`` on a wing with ``elements`` and ``section_data`` combines
them (a two-fidelity surrogate):

    cl(geometry, aoa, h) = cl_model(geometry, aoa, h) + sum_g w_g (cl_CFD,g - cl_model,g)(aoa, h)

* ``g`` runs over the CFD geometries with the same number of elements; the correction of each is
  taken on its own (aoa, ground clearance) range - outside it the value at the edge is kept, so the
  model gives the shape there,
* the weights ``w_g ~ 1 / d_g^4`` (normalised) fall with the distance in the geometry variables the
  CFD varies (main / flap chords, flap angles, gaps, overlaps, camber, thickness - each scaled by its
  range in the file). At a CFD geometry the result is exactly the CFD; far from all of them it is the
  model shifted by the mean correction,
* a variable the CFD never varies is left to the model alone.

So the optimisers (``aero-split``, ``auto-aero``) can move every element value continuously over
the CFD data, and ``aerolap cfd-plan`` asks AERO_AUTOMATION for new geometries around the optimum
they find - each batch makes the surrogate exact where the car wants to be.
"""

from __future__ import annotations

from functools import lru_cache

import numpy as np

from .elements import AOA_GRID, GAP_BEST, OVERLAP_BEST, Element, ElementSection, naca4
from .section_data import SectionPolar, load_sections


def section_from_params(params: dict) -> ElementSection | None:
    """The :class:`ElementSection` of a CFD geometry (``summary.csv`` columns); None without a main chord."""
    def num(key):
        v = params.get(key)
        return float(v) if isinstance(v, (int, float)) else None

    def foil(key):
        v = params.get(key)
        code = "" if v in (None, "") else (f"{v:g}" if isinstance(v, float) else str(v))
        try:
            return (*naca4(code), code) if code else (0.04, 0.12, "")
        except ValueError:  # a coordinate file: typical values
            return 0.04, 0.12, code

    c0 = num("main_chord")
    if c0 is None and num("chord") is not None:
        c0 = 1000.0 * num("chord")
    if c0 is None or c0 <= 0:
        return None
    camber, thick, code = foil("main_airfoil")
    out = [Element("main", c0, 0.0, 0.0, 0.0, camber, thick, code)]
    i = 1
    while num(f"flap{i}_chord"):
        camber, thick, code = foil(f"flap{i}_airfoil")
        gap, overlap = num(f"flap{i}_gap"), num(f"flap{i}_overlap")
        out.append(Element(f"flap{i}", num(f"flap{i}_chord"), num(f"flap{i}_deflection") or 0.0,
                           GAP_BEST * c0 if gap is None else gap, OVERLAP_BEST * c0 if overlap is None else overlap,
                           camber, thick, code))
        i += 1
    return ElementSection(tuple(out))


def geometry_vector(sec: ElementSection) -> dict[str, float]:
    """The numbers that describe a section's geometry (the axes of the correction's interpolation)."""
    out = {}
    for e in sec.elements:
        out[f"{e.name}_chord"] = e.chord_mm
        out[f"{e.name}_camber"] = 100.0 * e.camber
        out[f"{e.name}_thickness"] = 100.0 * e.thickness
        if e.name != "main":
            out.update({f"{e.name}_deflection": e.deflection, f"{e.name}_gap": e.gap_mm,
                        f"{e.name}_overlap": e.overlap_mm})
    return out


@lru_cache(maxsize=16)
def _entries(path: str, mtime: float, n_el: int):
    """(sections, polars, geometry vectors, scales) of the CFD geometries with ``n_el`` elements."""
    sset = load_sections(path)
    secs, pols = [], []
    for pol in sset.sections.values():
        sec = section_from_params(sset.full_params(pol))
        if sec is not None and len(sec.elements) == n_el:
            secs.append(sec)
            pols.append(pol)
    if not secs:
        have = sorted({len(s.elements) for s in (section_from_params(sset.full_params(p)) for p in sset.sections.values())
                       if s is not None})
        raise ValueError(f"section_fit: {sset.path.name} has no {n_el}-element geometry "
                         f"(it has {', '.join(map(str, have)) or 'no usable'}-element ones)")
    vecs = [geometry_vector(s) for s in secs]
    keys = [k for k in vecs[0] if max(v[k] for v in vecs) - min(v[k] for v in vecs) > 1e-9]
    scale = {k: max(v[k] for v in vecs) - min(v[k] for v in vecs) for k in keys}
    return sset, secs, pols, vecs, scale


def weights(vec: dict, vecs: list[dict], scale: dict) -> np.ndarray:
    """Inverse-distance weights of the CFD geometries for a geometry (sum 1; ~1 at a CFD geometry)."""
    d2 = np.array([sum(((vec[k] - v[k]) / s) ** 2 for k, s in scale.items()) for v in vecs])
    w = 1.0 / (d2 * d2 + 1e-18)
    return w / w.sum()


@lru_cache(maxsize=512)
def _calibrated(path: str, mtime: float, section: ElementSection, h_static: float, speed: float | None,
                ground: bool | None) -> SectionPolar:
    sset, secs, pols, vecs, scale = _entries(path, mtime, len(section.elements))
    cfd_ground = pols[0].has_ground
    if cfd_ground:
        ground = True
    elif ground is None:
        ground = h_static < sum(e.chord_mm for e in section.elements)
    if ground:
        hs = np.geomspace(0.05 * section.c0, max(8.0 * section.c0, 3.0 * h_static + 50.0), 60)
        a, h = np.meshgrid(AOA_GRID, hs, indexing="ij")
        cl, cd = section.coefficients(a, h)
    else:
        hs, a, h = None, AOA_GRID, None
        cl, cd = section.coefficients(a)
    w = weights(geometry_vector(section), vecs, scale)
    d_cl = d_cd = 0.0
    for wg, sec_g, pol in zip(w, secs, pols):
        if wg < 1e-6:
            continue
        ac = np.clip(a, pol.aoa[0], pol.aoa[-1])
        if pol.has_ground:
            hc = np.clip(h, pol.heights[0], pol.heights[-1])
            m_cl, m_cd = sec_g.coefficients(ac, hc)
            c_cl, c_cd = pol.cl_at(ac, hc, speed), pol.cd_at(ac, hc, speed)
        else:  # free-stream CFD: the correction does not depend on the clearance
            m_cl, m_cd = sec_g.coefficients(ac)
            c_cl, c_cd = pol.cl_at(ac, None, speed), pol.cd_at(ac, None, speed)
        d_cl = d_cl + wg * (c_cl - m_cl)
        d_cd = d_cd + wg * (c_cd - m_cd)
    cl = np.asarray(cl + d_cl, dtype=float)
    cd = np.maximum(np.asarray(cd + d_cd, dtype=float), 1e-4)
    k = int(np.argmax(w))
    out = SectionPolar(section.label(), section.params(), AOA_GRID.copy(), hs, None, cl, cd, section.c0 / 1000.0,
                       f"element model x CFD {sset.path.name}", sum(p.n_cases for p in pols), sset.synthetic)
    out.model = section
    out.calibrated = len(pols)
    out.nearest = (pols[k].geometry_id, float(w[k]))
    return out


def calibrated_polar(sset, section: ElementSection, h_static: float, speed: float | None = None,
                     ground: bool | None = None) -> SectionPolar:
    """The section's polar: element model + the CFD correction interpolated to its geometry."""
    return _calibrated(str(sset.path), sset.path.stat().st_mtime, section, float(h_static),
                       None if speed is None else float(speed), ground)
