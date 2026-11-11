"""Importers for files from other tools: Assetto Corsa AI lines and OptimumLap tracks / vehicles.

Assetto Corsa AI spline (``<track>/ai/fast_lane.ai``, also ``ideal_line.ai``)
-------------------------------------------------------------------------------
Binary, little-endian (layout as read by Content Manager / AcTools ``AiSpline``)::

    int32 version, int32 n_points, int32 lap_time, int32 sample_count
    n_points x { float32 x, y, z;  float32 length;  int32 id }                     20 bytes
    int32 n_extra (== n_points)
    n_extra x { float32 speed, gas, brake, obsolete_lat_g, radius, side_left, side_right,
                camber, direction, normal[3], length, forward[3], tag, grade }      72 bytes
    (a grid section may follow - ignored)

Assetto Corsa is y-up; the plan view is (x, -z) (``map.png`` is drawn with x to the right and
z downwards). ``fast_lane.ai`` is the AI racing line, i.e. the path a driver actually takes, which
is what a lap simulation should follow. The extra block (track edges ``side_left/right``) is
optional and only used for information. Elevation is read (``z_m`` / grade) but the lap
solvers do not use it yet.

The layout above is the community-documented one; the reader checks it against the file size
and fails with a clear message when a file does not match. Checked on a real FSK 2026 file
(``data/import/fsk2026_fast_lane.ai``: version 7, 872 points 1 m apart, edges, 8000-byte grid).

OptimumLap track (``.oltra``) and vehicle (``.olveh``), OptimumLap 1.4 / file version 1.0.5
-------------------------------------------------------------------------------------------
.NET ``BinaryWriter``, little-endian; strings are a 7-bit-encoded length + UTF-8 (layout read from
real files, not from a specification)::

    byte flag; strings: version, created, modified, GUID, name, ...
    .oltra: "Closed Circuit" | "Open Circuit", direction, type, 8 bytes, "", country, city, 4 bytes,
            3 x { int32 n; float64[n] }: segment length [m], radius [m] (0 = straight),
                                         direction (+1 left, -1 right, 0 straight)
    .olveh: colour, category, "", transmission, 4 strings
    property section (scalars): int16 n, n x { 6 strings: key, key, label, unit kind, flag, section;
                                              float64 value }
    property section (arrays):  int16 n, n x { 6 strings; int32 count; float64[count] }
    20-byte hash

Values are SI; ``SpeedAngular`` is rad/s. "+1 = left" is confirmed by the file's own
``percentleft`` statistic (share of the length with direction +1).
"""

from __future__ import annotations

import copy
import csv
import math
import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .track import Track

_POINT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("length", "<f4"), ("id", "<i4")])
_EXTRA_FIELDS = ["speed", "gas", "brake", "obsolete_lat_g", "radius", "side_left", "side_right", "camber",
                 "direction", "normal_x", "normal_y", "normal_z", "length", "forward_x", "forward_y",
                 "forward_z", "tag", "grade"]
_EXTRA = np.dtype([(f, "<f4") for f in _EXTRA_FIELDS])


@dataclass
class AcSpline:
    version: int
    x: np.ndarray  # plan view, m
    y: np.ndarray  # plan view, m (= -z of Assetto Corsa)
    elevation: np.ndarray  # m (Assetto Corsa y)
    length: np.ndarray  # cumulative distance stored in the file, m
    side_left: np.ndarray | None = None  # distance to the track edges, m
    side_right: np.ndarray | None = None
    lap_time_ms: int = 0

    @property
    def n(self) -> int:
        return len(self.x)


def read_ac_ai(path: str | Path) -> AcSpline:
    path = Path(path)
    raw = path.read_bytes()
    if len(raw) < 16:
        raise ValueError(f"{path}: too short for an Assetto Corsa AI spline")
    version, n, lap_time, _samples = struct.unpack_from("<4i", raw, 0)
    end_points = 16 + n * _POINT.itemsize
    if not (0 < version < 100) or n < 3 or end_points > len(raw):
        raise ValueError(f"{path}: not an Assetto Corsa AI spline (version {version}, {n} points, "
                         f"{len(raw)} bytes)")
    pts = np.frombuffer(raw, dtype=_POINT, count=n, offset=16)
    x = pts["x"].astype(float)
    elev = pts["y"].astype(float)
    y = -pts["z"].astype(float)
    step = np.hypot(np.diff(x), np.diff(y))
    if not np.all(np.isfinite(step)) or np.median(step) <= 0.01 or np.median(step) > 50.0:
        raise ValueError(f"{path}: point spacing (median {np.median(step):.3g} m) does not look like an AI line")
    out = AcSpline(version, x, y, elev, pts["length"].astype(float), lap_time_ms=lap_time)
    off = end_points
    if off + 4 <= len(raw):
        (n_extra,) = struct.unpack_from("<i", raw, off)
        if n_extra == n and off + 4 + n * _EXTRA.itemsize <= len(raw):
            ex = np.frombuffer(raw, dtype=_EXTRA, count=n, offset=off + 4)
            left, right = ex["side_left"].astype(float), ex["side_right"].astype(float)
            if np.all(np.isfinite(left)) and np.all(np.isfinite(right)) and 0 < np.median(left + right) < 100:
                out.side_left, out.side_right = left, right
    return out


def track_from_ac(path: str | Path, name: str | None = None, closed: bool | None = None, ds: float = 0.5,
                  cutoff_m: float = 10.0, reverse: bool = False) -> Track:
    """Track from an Assetto Corsa AI spline (``cutoff_m``: smoothing wavelength, see ``Track.from_xy``)."""
    sp = read_ac_ai(path)
    x, y = sp.x, sp.y
    if reverse:
        x, y = x[::-1], y[::-1]
    return Track.from_xy(x, y, name=name or Path(path).parent.parent.name or Path(path).stem,
                         closed=closed, ds=ds, cutoff_m=cutoff_m)


def write_xy_csv(sp: AcSpline, path: str | Path, name: str, closed: bool, cutoff_m: float) -> Path:
    """Centre-line CSV readable by ``Track.from_file`` (extra columns are kept for reference)."""
    path = Path(path)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        fh.write(f"# name: {name}\n# closed: {'true' if closed else 'false'}\n# cutoff_m: {cutoff_m:g}\n")
        fh.write("# source: Assetto Corsa AI spline, plan view (x, -z)\n")
        w = csv.writer(fh)
        cols = ["x_m", "y_m", "z_m"] + (["width_left_m", "width_right_m"] if sp.side_left is not None else [])
        w.writerow(cols)
        for i in range(sp.n):
            row = [f"{sp.x[i]:.3f}", f"{sp.y[i]:.3f}", f"{sp.elevation[i]:.3f}"]
            if sp.side_left is not None:
                row += [f"{sp.side_left[i]:.3f}", f"{sp.side_right[i]:.3f}"]
            w.writerow(row)
    return path


# ====================================================================== OptimumLap
class _Bin:
    """Minimal .NET BinaryReader."""

    def __init__(self, raw: bytes, pos: int = 0):
        self.raw, self.pos = raw, pos

    def string(self) -> str:
        n, shift = 0, 0
        while True:
            if self.pos >= len(self.raw):
                raise ValueError("unexpected end of file in a string")
            b = self.raw[self.pos]
            self.pos += 1
            n |= (b & 0x7F) << shift
            shift += 7
            if b < 0x80:
                break
            if shift > 28:
                raise ValueError("bad string length")
        if self.pos + n > len(self.raw):
            raise ValueError("string runs past the end of the file")
        out = self.raw[self.pos:self.pos + n].decode("utf-8")
        self.pos += n
        return out

    def unpack(self, fmt: str):
        vals = struct.unpack_from(fmt, self.raw, self.pos)
        self.pos += struct.calcsize(fmt)
        return vals

    def doubles(self) -> np.ndarray:
        (n,) = self.unpack("<i")
        if n < 0 or self.pos + 8 * n > len(self.raw):
            raise ValueError(f"bad array length {n}")
        return np.array(self.unpack(f"<{n}d"))


@dataclass
class OlProperties:
    scalars: dict[str, float] = field(default_factory=dict)
    arrays: dict[str, np.ndarray] = field(default_factory=dict)
    units: dict[str, str] = field(default_factory=dict)
    labels: dict[str, str] = field(default_factory=dict)


def _record_start(raw: bytes, start: int) -> int:
    """Offset of the first property section (int16 count followed by a record whose two keys match)."""
    for o in range(start, min(len(raw) - 4, start + 4096)):
        n = struct.unpack_from("<h", raw, o)[0]
        if not 0 < n < 1000:
            continue
        try:
            r = _Bin(raw, o + 2)
            a, b = r.string(), r.string()
        except (ValueError, UnicodeDecodeError):
            continue
        if a and a == b and a.isascii() and a.replace("_", "").isalnum():
            return o
    raise ValueError("no OptimumLap property section found")


def _read_properties(raw: bytes, start: int) -> OlProperties:
    r = _Bin(raw, _record_start(raw, start))
    props = OlProperties()
    for section in ("scalar", "array"):
        if r.pos + 2 > len(raw):
            break
        (n,) = r.unpack("<h")
        for _ in range(n):
            key, _key2, label, unit, _flag, _sec = (r.string() for _ in range(6))
            props.units[key], props.labels[key] = unit, label
            if section == "scalar":
                props.scalars[key] = r.unpack("<d")[0]
            else:
                props.arrays[key] = r.doubles()
    return props


def _header_strings(raw: bytes, count: int) -> list[str]:
    r = _Bin(raw, 1)
    return [r.string() for _ in range(count)]


@dataclass
class OlTrack:
    name: str
    closed: bool
    lengths: np.ndarray
    radii: np.ndarray
    directions: np.ndarray  # +1 left, -1 right, 0 straight
    header: list[str]
    props: OlProperties

    @property
    def length(self) -> float:
        return float(self.lengths.sum())

    def segments(self) -> list[tuple[str, float, float, str]]:
        out = []
        for length, r, d in zip(self.lengths, self.radii, self.directions):
            if d == 0 or r <= 0:
                out.append(("straight", float(length), 0.0, ""))
            else:
                out.append(("corner", float(length), float(r), "left" if d > 0 else "right"))
        return out


def _find_segment_arrays(raw: bytes, start: int) -> tuple[int, int]:
    """Offset and count of the three consecutive (length, radius, direction) arrays."""
    for o in range(start, len(raw) - 12):
        n = struct.unpack_from("<i", raw, o)[0]
        if not 0 < n < 1_000_000:
            continue
        o2, o3 = o + 4 + 8 * n, o + 8 + 16 * n
        if o3 + 4 + 8 * n > len(raw):
            continue
        if struct.unpack_from("<i", raw, o2)[0] != n or struct.unpack_from("<i", raw, o3)[0] != n:
            continue
        d = np.array(struct.unpack_from(f"<{n}d", raw, o3 + 4))
        lengths = np.array(struct.unpack_from(f"<{n}d", raw, o + 4))
        if np.all(np.isin(d, (-1.0, 0.0, 1.0))) and np.all(lengths > 0):
            return o, n
    raise ValueError("no OptimumLap segment arrays found")


def read_optimumlap_track(path: str | Path) -> OlTrack:
    path = Path(path)
    raw = path.read_bytes()
    try:
        header = _header_strings(raw, 8)
        if not header[0].count(".") >= 1:
            raise ValueError
    except (ValueError, UnicodeDecodeError, IndexError):
        raise ValueError(f"{path}: not an OptimumLap track file") from None
    o, n = _find_segment_arrays(raw, 1)
    r = _Bin(raw, o)
    lengths, radii, dirs = r.doubles(), r.doubles(), r.doubles()
    try:
        props = _read_properties(raw, r.pos)
    except ValueError:
        props = OlProperties()
    closed = not any("open" in h.lower() for h in header[5:8])
    tr = OlTrack(header[4], closed, lengths, radii, dirs, header, props)
    total = props.scalars.get("totallength")
    if total and abs(total - tr.length) > 1e-6 * max(total, 1.0):
        raise ValueError(f"{path}: segment lengths sum to {tr.length:.3f} m but the file says {total:.3f} m")
    if np.any((dirs != 0) & (radii <= 0)):
        raise ValueError(f"{path}: corner segment with a non-positive radius")
    return tr


def track_from_optimumlap(path: str | Path, ds: float = 0.5, closed: bool | None = None) -> Track:
    ol = read_optimumlap_track(path)
    return Track.from_segments(ol.segments(), name=ol.name, closed=ol.closed if closed is None else closed, ds=ds)


def write_segment_csv(segments, path: str | Path, name: str, closed: bool, source: str = "") -> Path:
    path = Path(path)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        fh.write(f"# name: {name}\n# closed: {'true' if closed else 'false'}\n")
        if source:
            fh.write(f"# source: {source}\n")
        w = csv.writer(fh)
        w.writerow(["type", "length_m", "radius_m", "direction"])
        for kind, length, radius, direction in segments:
            if kind == "straight":
                w.writerow(["straight", f"{length:.6f}", "", ""])
            else:
                w.writerow(["corner", f"{length:.6f}", f"{radius:.6f}", direction])
    return path


@dataclass
class OlVehicle:
    name: str
    header: list[str]
    props: OlProperties

    def get(self, key: str, default: float | None = None) -> float | None:
        return self.props.scalars.get(key, default)

    @property
    def gear_ratios(self) -> np.ndarray:
        return self.props.arrays.get("gearratiodata", np.array([1.0]))

    @property
    def torque_curve(self) -> tuple[np.ndarray, np.ndarray]:
        """(rpm, Nm) of the engine / motor curve."""
        a = self.props.arrays
        return a["rpmdata"] * 60.0 / (2.0 * math.pi), a["torquedata"]

    @property
    def rev_limit_rpm(self) -> float:
        return float(self.get("revlimit", 0.0)) * 60.0 / (2.0 * math.pi)

    @property
    def transmission(self) -> str:
        return next((h for h in self.header if "gearbox" in h.lower() or "transmission" in h.lower()), "")


def read_optimumlap_vehicle(path: str | Path) -> OlVehicle:
    path = Path(path)
    raw = path.read_bytes()
    try:
        header = _header_strings(raw, 5)
        props = _read_properties(raw, 1)
    except (ValueError, UnicodeDecodeError) as exc:
        raise ValueError(f"{path}: not an OptimumLap vehicle file ({exc})") from None
    # header strings up to the property section (count differs between file versions)
    r, hdr = _Bin(raw, 1), []
    end = _record_start(raw, 1)
    while r.pos < end:
        try:
            hdr.append(r.string())
        except (ValueError, UnicodeDecodeError):
            break
    for key in ("mass", "tireradius", "torquedata", "rpmdata"):
        if key not in props.scalars and key not in props.arrays:
            raise ValueError(f"{path}: OptimumLap vehicle without '{key}'")
    return OlVehicle(header[4], hdr or header, props)


# keys taken from the OptimumLap file; everything else comes from the base vehicle (assumed)
OL_VEHICLE_KEYS = {
    "mass": "mass.total_kg",
    "weightdrivenwheel": "mass.weight_dist_front (= 1 - value)",
    "tireradius": "tires.radius_mm",
    "longfriction": "tires.mu_x (x gripfactor)",
    "latfriction": "tires.mu_y (x gripfactor)",
    "tirerollingdrag": "tires.rolling_resistance",
    "liftcoefficient": "aero.cl (x aerofactor)",
    "dragcoefficient": "aero.cd (x aerofactor)",
    "frontarea": "aero.frontal_area_m2",
    "airdensity": "environment.air_density_kg_m3",
    "driveefficiency": "powertrain.efficiency",
    "finaldriveratio": "powertrain.ev.reduction_ratio / ice.final_drive (x gear ratio)",
    "revlimit": "powertrain.ev.max_rpm / ice.rev_limit_rpm",
    "torquedata": "powertrain.ev.motor_torque_curve / ice.torque_curve (x powerfactor)",
}


# groups of vehicle keys an OptimumLap file can supply (``take`` of vehicle_cfg_from_optimumlap)
OL_GROUPS = ("environment", "mass", "tires", "aero", "powertrain")
_OL_GROUP_KEYS = {"environment": ("airdensity",), "mass": ("mass", "weightdrivenwheel"),
                  "tires": ("tireradius", "longfriction", "latfriction", "tirerollingdrag"),
                  "aero": ("liftcoefficient", "dragcoefficient", "frontarea"),
                  "powertrain": ("driveefficiency", "finaldriveratio", "revlimit", "torquedata")}


def vehicle_cfg_from_optimumlap(ol: OlVehicle, base_raw: dict, drive: str = "rwd", full: bool = False,
                                motor: dict | None = None, take=OL_GROUPS,
                                ) -> tuple[dict, list[str], list[str], list[str]]:
    """Raw aerolap vehicle dict (before defaults), the keys taken from the file, the assumed sections
    (copied from ``base_raw``) and warnings.

    ``full=False``: OptimumLap-equivalent car (constant aero, the file's torque curve, no rotor
    inertia, no battery). ``full=True``: same tyres / mass / aero, but the powertrain is the ``motor``
    model (e.g. EMRAX 228) with a battery-side power limit; suspension, geometry, CG, battery are
    taken from ``base_raw`` and marked as assumed. ``take``: the groups of ``OL_GROUPS`` read from
    the file; the other groups are kept from ``base_raw`` unchanged.
    """
    take = tuple(take)
    bad = [t for t in take if t not in OL_GROUPS]
    if bad:
        raise ValueError(f"optimumlap.take: unknown group(s) {bad}; allowed: {list(OL_GROUPS)}")
    cfg, taken, assumed, warnings = _vehicle_cfg_all(ol, base_raw, drive, full, motor)
    if set(take) == set(OL_GROUPS):
        return cfg, taken, assumed, warnings
    for grp in OL_GROUPS:
        if grp in take:
            continue
        if grp in base_raw:
            cfg[grp] = copy.deepcopy(base_raw[grp])
        else:
            cfg.pop(grp, None)
        if grp == "powertrain" and "battery" in base_raw:
            cfg["battery"] = copy.deepcopy(base_raw["battery"])
        if grp == "mass" and "aero" in take:  # aero balance follows the (kept) weight distribution
            cfg["aero"]["cop_front"] = round(float(cfg["mass"].get("weight_dist_front", 0.5)), 4)
    keep = {k for grp in take for k in _OL_GROUP_KEYS[grp]}
    taken = [t for t in taken if t.split(" ->")[0] in keep]
    if "powertrain" not in take:
        assumed = [a for a in assumed if not a.startswith(("powertrain", "battery"))]
    warnings = [w for w in warnings if "tires" in take or "friction" not in w]
    return cfg, taken, assumed, warnings


def _vehicle_cfg_all(ol: OlVehicle, base_raw: dict, drive: str, full: bool,
                     motor: dict | None) -> tuple[dict, list[str], list[str], list[str]]:
    g = ol.get
    cfg = copy.deepcopy(base_raw)
    warnings = []
    grip = float(g("gripfactor", 1.0) or 1.0)
    aero_f = float(g("aerofactor", 1.0) or 1.0)
    power_f = float(g("powerfactor", 1.0) or 1.0)
    cfg["name"] = f"{ol.name} (OptimumLap import{', full model' if full else ''})"
    cfg.setdefault("environment", {})
    cfg["environment"] = {"air_density_kg_m3": float(g("airdensity", 1.225))}
    wd = g("weightdrivenwheel")
    mass = cfg.setdefault("mass", {})
    mass["total_kg"] = float(g("mass"))
    if wd is not None and 0.0 < wd < 1.0:
        if drive == "rwd":
            mass["weight_dist_front"] = 1.0 - float(wd)
    tires = cfg.setdefault("tires", {})
    tires.update({"radius_mm": float(g("tireradius")) * 1000.0, "mu_x": float(g("longfriction", 1.0)) * grip,
                  "mu_y": float(g("latfriction", 1.0)) * grip, "load_sensitivity": 0.0,
                  "rolling_resistance": float(g("tirerollingdrag", 0.015))})
    for k in ("sensitivitylatfriction", "sensitivitylongfriction", "masslatfriction", "masslongfriction"):
        if g(k):
            warnings.append(f"OptimumLap '{k}' = {g(k):g} is not mapped (set tires.load_sensitivity by hand)")
    cop = float(mass.get("weight_dist_front", 0.5))
    cfg["aero"] = {"type": "constant", "cl": float(g("liftcoefficient", 0.0)) * aero_f,
                   "cd": float(g("dragcoefficient", 0.0)) * aero_f, "frontal_area_m2": float(g("frontarea", 1.0)),
                   "cop_front": round(cop, 4)}
    for k in ("drag_height_mm", "floor_width_mm", "bottoming_clearance_mm"):
        if k in base_raw.get("aero", {}):
            cfg["aero"][k] = base_raw["aero"][k]
    rpm, tq = ol.torque_curve
    gears = [float(x) for x in ol.gear_ratios]
    final = float(g("finaldriveratio", 1.0))
    rev = ol.rev_limit_rpm or float(rpm.max())
    curve = [[round(float(r), 3), round(float(t) * power_f, 4)] for r, t in zip(rpm, tq)]
    pt = {"type": "ev", "drive": drive, "differential": "lsd", "efficiency": float(g("driveefficiency", 0.9))}
    if len(gears) > 1:
        pt["type"] = "ice"
        pt["ice"] = {"torque_curve": curve, "gear_ratios": gears, "final_drive": final, "rev_limit_rpm": rev,
                     "shift_time_s": 0.0}
    elif full and motor:
        base_ev = (base_raw.get("powertrain") or {}).get("ev", {})
        ev = {k: base_ev[k] for k in ("drivetrain_efficiency", "inverter_efficiency", "power_limit_kW",
                                      "n_motors") if k in base_ev}
        ev.update(motor)
        ev["reduction_ratio"] = final * gears[0]
        pt["ev"] = ev
    else:
        pt["ev"] = {"reduction_ratio": final * gears[0], "motor_torque_curve": curve, "max_rpm": rev, "n_motors": 1}
    cfg["powertrain"] = pt
    if not full:
        cfg.pop("battery", None)
    taken = [f"{k} -> {v}" for k, v in OL_VEHICLE_KEYS.items() if k in ol.props.scalars or k in ol.props.arrays]
    if pt["type"] == "ev" and full and motor:  # motor model: the file's torque curve / rev limit are not used
        taken = [t for t in taken if not t.startswith(("torquedata", "revlimit"))]
    assumed = [k for k in ("mass.cg_height_mm / unsprung / inertias", "geometry", "tires (other than mu, radius, "
               "rolling)", "suspension", "brakes", "aero.cop_front, drag_height_mm, floor_width_mm")]
    if full:
        assumed += ["powertrain.ev motor settings (" + ", ".join(sorted(motor or {})) + ")",
                    "powertrain.ev power_limit_kW / efficiencies", "battery"]
    return cfg, taken, assumed, warnings


DEFAULT_OL_BASE = Path(__file__).resolve().parents[2] / "data" / "vehicles" / "fsae_ev_aero.yaml"
_LINK_KEYS = {"file", "take", "full", "drive", "base"}


def _motor_from(raw: dict) -> dict:
    ev = (raw.get("powertrain") or {}).get("ev") or {}
    return {k: ev[k] for k in ("motor", "motor_variant", "motor_cooling", "dc_voltage_V") if k in ev}


def raw_from_optimumlap_file(path: str | Path, base: str | Path | None = None, full: bool = False,
                             drive: str = "rwd") -> tuple[dict, Path, list[str]]:
    """Raw vehicle dict built from an ``.olveh`` file alone (everything the file lacks comes from
    ``base``, default ``data/vehicles/fsae_ev_aero.yaml``). Returns (raw, base file, notes)."""
    import yaml

    base_path = Path(base) if base else DEFAULT_OL_BASE
    with open(base_path, encoding="utf-8") as fh:
        base_raw = yaml.safe_load(fh) or {}
    ol = read_optimumlap_vehicle(path)
    motor = None
    if full:
        motor = _motor_from(base_raw) or {"motor": "emrax_228", "motor_variant": "MV", "motor_cooling": "LC",
                                          "dc_voltage_V": 550.0}
    raw, _taken, assumed, warnings = vehicle_cfg_from_optimumlap(ol, base_raw, drive=drive, full=full, motor=motor)
    notes = [f"{Path(path).name}: OptimumLap vehicle read directly; values not in the file ("
             + ", ".join(assumed) + f") are ASSUMED from {base_path.name}"] + warnings
    return raw, base_path, notes


def resolve_optimumlap_link(raw: dict, base_dir: str | Path) -> tuple[dict, list[str]]:
    """Apply an ``optimumlap:`` section of a vehicle YAML.

    ``optimumlap: {file: x.olveh, take: [mass, tires, aero, powertrain, environment], full: false,
    drive: rwd}`` reads the listed groups from the OptimumLap file (path relative to the YAML) and
    keeps everything else - CG, geometry, suspension, aero devices, battery ... - from the YAML
    itself, so one OptimumLap file can stay the single source of mass / grip / aero / torque.
    ``full: true`` keeps the YAML's motor model (``powertrain.ev``) instead of the file's torque curve.
    """
    link = raw.get("optimumlap")
    if link is None:
        return raw, []
    if isinstance(link, str):
        link = {"file": link}
    if not isinstance(link, dict) or "file" not in link:
        raise ValueError("optimumlap: needs 'file: <vehicle>.olveh' (or just the file name)")
    unknown = set(link) - _LINK_KEYS
    if unknown:
        raise ValueError(f"optimumlap: unknown key(s) {sorted(unknown)}; allowed: {sorted(_LINK_KEYS)}")
    base_dir = Path(base_dir)
    olfile = Path(link["file"])
    if not olfile.is_absolute():
        olfile = base_dir / olfile
    if not olfile.exists():
        raise ValueError(f"optimumlap.file: {olfile} not found")
    own = {k: v for k, v in raw.items() if k != "optimumlap"}
    name = own.get("name")  # the YAML's own name only (not the base's)
    if link.get("base"):
        import yaml

        bp = Path(link["base"])
        bp = bp if bp.is_absolute() else base_dir / bp
        with open(bp, encoding="utf-8") as fh:
            base_raw = yaml.safe_load(fh) or {}
        from .config import deep_merge

        own = deep_merge(base_raw, own)
    take = link.get("take", OL_GROUPS)
    if isinstance(take, str):
        take = [t.strip() for t in take.split(",") if t.strip()]
    full = bool(link.get("full", False))
    drive = str(link.get("drive", (own.get("powertrain") or {}).get("drive", "rwd")))
    motor = _motor_from(own) if full else None
    if full and not (motor or {}).get("motor"):
        raise ValueError("optimumlap.full: true needs powertrain.ev.motor in the YAML (e.g. emrax_228)")
    ol = read_optimumlap_vehicle(olfile)
    cfg, taken, _assumed, warnings = vehicle_cfg_from_optimumlap(ol, own, drive=drive, full=full, motor=motor,
                                                                 take=take)
    if name:
        cfg["name"] = name
    notes = [f"{olfile.name}: " + "; ".join(t.split(" ->")[0] for t in taken) + " read from OptimumLap"]
    return cfg, notes + warnings
