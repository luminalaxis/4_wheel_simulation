"""Command line interface: ``aerolap <command> ...`` (run ``aerolap -h``)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from . import plotting
from .analysis.cornering import (BRAKING_COLUMNS, CORNERING_COLUMNS, braking_sweep, cornering_sweep,
                                 format_table)
from .analysis.events import events_table, run_events
from .analysis.ladder import effect_ladder, ladder_table
from .analysis.objectives import Objective
from .analysis.optimize import OptimizationSpec, run_optimization
from .analysis.sweep import parse_values, sensitivities, sensitivity_table, sweep
from .config import dump_config, load_vehicle_config, parse_override, unknown_keys, with_overrides
from .features import LEVELS, features_for_level
from .solvers.ggv import SolverOptions
from .solvers.qss import QSSSolver
from .solvers.transient import TransientOptions, TransientSolver
from .track import Track
from .vehicle import VehicleModel


def _vehicle_cfg(args) -> dict:
    cfg = load_vehicle_config(args.vehicle)
    over = dict(parse_override(s) for s in (args.set or []))
    if not over:
        return cfg
    out = with_overrides(cfg, over, "명령줄 --set")
    for msg in unknown_keys(out):  # a misspelt --set path would silently change nothing
        if msg not in cfg.get("_warnings", []):
            print(f"warning: --set: {msg}", file=sys.stderr)
    return out


def _opts(args) -> SolverOptions:
    opts = SolverOptions.coarse() if getattr(args, "coarse", False) else SolverOptions()
    if getattr(args, "ds", None):
        opts.ds = args.ds
    return opts


def _out(args, default: str) -> Path:
    out = Path(args.out or default)
    out.mkdir(parents=True, exist_ok=True)
    return out


def _write(path: Path, text: str) -> None:
    path.write_text(text + "\n", encoding="utf-8")


def _summary_text(res) -> str:
    lines = [f"vehicle: {res.vehicle_name}", f"track:   {res.track_name}", f"model:   {res.level}", ""]
    for k, v in res.summary().items():
        lines.append(f"{k:24s} {v:12.4f}")
    lim = res.channels.get("limit")
    if lim is not None:
        ds = np.diff(res.s)
        lines.append("")
        lines.append("limiting factor (share of distance):")
        for name in sorted(set(lim[:-1])):
            share = ds[lim[:-1] == name].sum() / ds.sum() * 100.0
            lines.append(f"  {name:22s} {share:6.1f} %")
    for flag in ("bottoming", "aero_out_of_range", "wheel_lift", "not_converged"):
        n = int(res.channels[flag].sum())
        if n:
            lines.append(f"WARNING: {flag} at {n} track points")
    return "\n".join(lines)


# ---------------------------------------------------------------------- commands
def cmd_run(args) -> int:
    cfg = _vehicle_cfg(args)
    car = VehicleModel(cfg, features_for_level(args.level))
    track = Track.from_file(args.track)
    start = args.start or ("flying" if track.closed else "standing")
    res = QSSSolver(_opts(args)).solve(car, track, start=start)
    out = _out(args, "out/run")
    text = _summary_text(res)
    from .analysis.cooling import cooling_balance

    cb = cooling_balance(car, res)
    if cb is not None:
        text += "\n" + cb.report()
    if getattr(car.tire, "model", "simple") != "simple":
        text += "\n\n" + car.tire.describe()
    print(text)
    _write(out / "summary.txt", text)
    res.to_csv(out / "trace.csv")
    plotting.plot_lap(res, out / "lap.png")
    plotting.plot_track_map(res, out / "track_map.png")
    extra = ""
    if args.mat:
        from .export import lap_to_mat

        lap_to_mat(res, out / "trace.mat")
        extra = ", trace.mat"
    print(f"\nwritten: {out}/summary.txt, trace.csv{extra}, lap.png, track_map.png")
    return 0


def cmd_transient(args) -> int:
    cfg = _vehicle_cfg(args)
    car = VehicleModel(cfg, features_for_level(args.level))
    track = Track.from_file(args.track)
    start = args.start or ("flying" if track.closed else "standing")
    opts = TransientOptions(dt=args.dt, iterations=args.iterations, max_jerk=args.max_jerk,
                            turn_in_length=args.turn_in, road=args.road, road_seed=args.road_seed,
                            road_scale=args.road_scale, lateral_dof=not args.no_lateral, qss=_opts(args))
    solver = TransientSolver(opts)
    tr = solver.solve(car, track, start=start)
    qss = solver.qss_reference
    sq, st = qss.summary(), tr.summary()
    keys = ["lap_time_s", "v_max_kmh", "v_min_kmh", "drive_energy_kWh", "min_clearance_mm", "min_rh_front_mm",
            "min_rh_rear_mm", "bottoming_m", "max_roll_deg", "max_yaw_deg", "non_default_mode_pct"]
    if "max_path_error_m" in st:
        keys.append("max_path_error_m")
        sq.setdefault("max_path_error_m", 0.0)
    lines = [f"vehicle: {car.name}   model: {car.features.label}", f"track:   {track.name} ({start})",
             f"road:    {tr.info['road']}  (rms {tr.info['road_rms_mm']:.2f} mm)   tyre load fluctuation "
             f"(std around 0.25 s mean): {tr.info['tyre_load_std_N']:.1f} N   wheel lift at "
             f"{int(tr.channels['wheel_lift'].sum())} track points",
             f"QSS on the original segment line: {tr.info['qss_lap_time_s']:.3f} s "
             f"(the comparison below uses the {args.turn_in:g} m turn-in-smoothed line)", "",
             f"{'':24s} {'QSS':>12s} {'transient':>12s} {'change':>10s}", "-" * 62]
    for k in keys:
        lines.append(f"{k:24s} {sq[k]:12.4f} {st[k]:12.4f} {st[k] - sq[k]:+10.4f}")
    if not args.no_lateral:
        lines.append(f"{'spins':24s} {0:12d} {int(tr.channels['spin'].sum()):12d}")
    lines += ["", "learning laps (driver lowers its target where the tyres were over-used, the car slid "
              "or ran off the line):"]
    for h in tr.info["iterations"]:
        lines.append(f"  #{h['iteration']}  target {h['reference_lap_s']:8.3f} s   driven {h['lap_s']:8.3f} s   "
                     f"max learning signal {h['max_lateral_excess'] * 100:6.2f} %")
    lines.append("")
    lines.append("Absolute transient lap times include the simple driver model (jerk-limited, no "
                 "optimal trail braking);\ncompare setups with the same settings rather than reading "
                 "the QSS/transient gap as exact.")
    text = "\n".join(lines)
    print(text)
    out = _out(args, "out/transient")
    _write(out / "transient.txt", text)
    tr.to_csv(out / "trace_transient.csv")
    qss.to_csv(out / "trace_qss.csv")
    plotting.plot_dynamics(qss, tr, out / "dynamics.png")
    plotting.plot_track_map(tr, out / "track_map.png")
    print(f"\nwritten: {out}/transient.txt, dynamics.png, track_map.png, trace_transient.csv, trace_qss.csv")
    return 0


def cmd_compare(args) -> int:
    cfg = _vehicle_cfg(args)
    track = Track.from_file(args.track)
    start = args.start or ("flying" if track.closed else "standing")
    levels = args.levels.split(",") if args.levels else list(LEVELS)
    steps = effect_ladder(cfg, track, start=start, opts=_opts(args), levels=[lv.upper() for lv in levels])
    out = _out(args, "out/compare")
    text = f"vehicle: {cfg.get('name')}\ntrack:   {track.name} ({start})\n\n" + ladder_table(steps)
    print(text)
    _write(out / "ladder.txt", text)
    plotting.plot_ladder(steps, out / "ladder.png")
    plotting.plot_compare([s.result for s in steps], [s.level for s in steps], out / "compare.png",
                          title=f"{track.name}: OptimumLap-equivalent -> full model")
    for s in steps:
        s.result.to_csv(out / f"trace_{s.level}.csv")
    print(f"\nwritten: {out}/ladder.txt, ladder.png, compare.png, trace_*.csv")
    return 0


def cmd_cornering(args) -> int:
    cfg = _vehicle_cfg(args)
    car = VehicleModel(cfg, features_for_level(args.level))
    mode = car.aero.mode_index(args.mode, car.default_mode)
    speeds = parse_values(args.speeds) / 3.6
    opts = _opts(args)
    corner = cornering_sweep(car, speeds, mode, opts)
    brake = braking_sweep(car, speeds, mode, opts)
    ff, fr = car.ride_frequencies()
    out = _out(args, "out/cornering")
    text = (f"vehicle: {car.name}   aero mode: {car.mode_names[mode]}   model: {car.features.label}\n"
            f"static weight front: {car.wd_front * 100:.1f} %   ride frequency front/rear: "
            f"{ff:.2f} / {fr:.2f} Hz\n\n"
            "steady-state cornering limit (ax = 0):\n" + format_table(corner, CORNERING_COLUMNS)
            + "\n\nstraight-line braking limit:\n" + format_table(brake, BRAKING_COLUMNS))
    print(text)
    _write(out / "cornering.txt", text)
    plotting.plot_cornering(corner, out / "cornering.png",
                            title=f"{car.name} - steady-state cornering ({car.mode_names[mode]})")
    print(f"\nwritten: {out}/cornering.txt, cornering.png")
    return 0


def cmd_ggv(args) -> int:
    cfg = _vehicle_cfg(args)
    car = VehicleModel(cfg, features_for_level(args.level))
    solver = QSSSolver(_opts(args))
    modes = range(len(car.mode_names)) if args.all_modes else [car.default_mode]
    ggvs = [solver.ggv(car, m) for m in modes]
    out = _out(args, "out/ggv")
    plotting.plot_ggv(ggvs, car.mode_names, out / "ggv.png")
    for g in ggvs:
        v = g.v[:: max(1, len(g.v) // 12)]
        print(f"mode {car.mode_names[g.mode]}:  v[km/h]  ay_max[g]  ax_acc[g]  ax_brk[g]")
        for vi in v:
            print(f"    {vi * 3.6:8.1f} {g.ay_max_at(vi) / 9.80665:9.3f} {g.acc_at(vi, 0.0) / 9.80665:9.3f} "
                  f"{g.brk_at(vi, 0.0) / 9.80665:9.3f}")
    files = ["ggv.png"]
    if args.export:
        from .export import export_ggv

        meta = {"vehicle": car.name, "model": car.features.label, "mass_kg": cfg["mass"]["total_kg"],
                "source": cfg.get("_source", "")}
        files += [p.name for p in export_ggv(ggvs, car.mode_names, out, meta)]
    print(f"\nwritten: {out}/" + ", ".join(files))
    return 0


def _constraints_from_args(args) -> dict:
    c = {}
    if args.min_clearance_mm is not None:
        c["min_clearance_mm"] = args.min_clearance_mm
    if args.max_roll_deg is not None:
        c["max_roll_deg"] = args.max_roll_deg
    if args.understeer:
        c["understeer_index"] = [float(x) for x in args.understeer.split(",")]
    return c


def _objective_from_args(args, opts) -> Objective:
    spec = {"type": args.objective, "level": args.level}
    if args.objective == "lap_time":
        if not args.track:
            raise SystemExit("--track is required for --objective lap_time")
        spec["track"] = str(Path(args.track).resolve())
        if args.start:
            spec["start"] = args.start
        spec["solver"] = args.solver
        if args.road:
            spec["road"] = args.road if args.road.upper() in "ABCDE" else str(Path(args.road).resolve())
    if args.objective == "ay_max":
        spec["speeds_kmh"] = [float(x) for x in args.speeds_kmh.split(",")]
    return Objective(spec, ".", _constraints_from_args(args), opts)


def cmd_sweep(args) -> int:
    cfg = _vehicle_cfg(args)
    opts = _opts(args)
    obj = _objective_from_args(args, opts)
    paths, values = [], []
    for p in args.param:
        path, vals = p.split("=", 1)
        paths.append(path)
        values.append(parse_values(vals))
    if len(paths) > 2:
        raise SystemExit("at most two --param for a sweep (use 'optimize' for more variables)")

    def progress(k, total, over, ev):
        txt = ", ".join(f"{p.split('.')[-1]}={v:g}" for p, v in over.items())
        print(f"  [{k}/{total}] {txt}: {ev.value:.4f} {obj.unit}", flush=True)

    sw = sweep(cfg, paths, values, obj, progress)
    out = _out(args, "out/sweep")
    text = f"objective: {obj.label} [{obj.unit}]\n\n" + sw.table()
    print("\n" + text)
    _write(out / "sweep.txt", text)
    plotting.plot_sweep(sw, out / "sweep.png", obj.unit)
    print(f"\nwritten: {out}/sweep.txt, sweep.png")
    return 0


def cmd_sensitivity(args) -> int:
    cfg = _vehicle_cfg(args)
    obj = _objective_from_args(args, _opts(args))
    paths = [p.strip() for p in args.params.split(",") if p.strip()]
    base, rows = sensitivities(cfg, paths, obj, rel_step=args.rel_step)
    text = f"objective: {obj.label}\n" + sensitivity_table(base, rows, obj.unit)
    print(text)
    out = _out(args, "out/sensitivity")
    _write(out / "sensitivity.txt", text)
    return 0


def cmd_optimize(args) -> int:
    spec = OptimizationSpec.from_file(args.spec)
    if args.method:
        spec.method = args.method
    if args.max_evals:
        spec.max_evals = args.max_evals
    over = dict(parse_override(s) for s in (args.set or []))
    if over:
        spec.vehicle_cfg = with_overrides(spec.vehicle_cfg, over)

    def progress(n, x, ev, err):
        txt = ", ".join(f"{'.'.join(p.split('.')[-2:])}={v:.4g}" for p, v in x.items())
        if ev is None:
            print(f"  #{n:3d} invalid setup ({err}): {txt}", flush=True)
        else:
            flag = "" if ev.feasible else "  (constraint violated)"
            print(f"  #{n:3d} {ev.value:10.4f}  penalty {ev.penalty:8.3f}  {txt}{flag}", flush=True)

    print(f"optimising {len(spec.variables)} variables with {spec.method} (max {spec.max_evals} evaluations)")
    res = run_optimization(spec, progress)
    out = _out(args, "out/optimize")
    text = res.report()
    print("\n" + text)
    _write(out / "optimization.txt", text)
    dump_config(res.best_cfg, out / "optimized_vehicle.yaml")
    plotting.plot_optimization(res, out / "optimization.png")
    print(f"\nwritten: {out}/optimization.txt, optimization.png, optimized_vehicle.yaml")
    return 0


def cmd_events(args) -> int:
    cfg = _vehicle_cfg(args)
    car = VehicleModel(cfg, features_for_level(args.level))
    auto = Track.from_file(args.autocross) if args.autocross else None
    endu = Track.from_file(args.endurance, closed=True) if args.endurance else None
    events = run_events(car, auto, endu, args.distance_km, _opts(args))
    out = _out(args, "out/events")
    text = f"vehicle: {car.name}   model: {car.features.label}\n\n" + events_table(events)
    print(text)
    _write(out / "events.txt", text)
    for e in events:
        e.result.to_csv(out / f"trace_{e.name}.csv")
    print(f"\nwritten: {out}/events.txt, trace_*.csv")
    return 0


def cmd_energy(args) -> int:
    from .analysis.energy import endurance_energy, energy_sweep, energy_sweep_table

    cfg = _vehicle_cfg(args)
    feats = features_for_level(args.level)
    track = Track.from_file(args.track, closed=True)
    out = _out(args, "out/energy")
    en = endurance_energy(cfg, track, args.distance_km, feats, _opts(args))
    text = f"vehicle: {cfg.get('name')}   model: {feats.label}\n\n" + en.report()
    if args.param:
        p, raw = args.param.split("=", 1)
        values = parse_values(raw)

        def progress(k, n, x, e):
            cap = "-" if e.power_cap_kW is None else f"{e.power_cap_kW:.1f} kW"
            print(f"  [{k}/{n}] {p} = {x:g}: flat out {e.unlimited_required_kWh:.2f} kWh, cap {cap}, "
                  f"endurance {e.total_time_s:.1f} s", flush=True)

        rows = energy_sweep(cfg, track, p, values, args.distance_km, feats, _opts(args), progress,
                            relative=args.relative)
        text += "\n\n" + energy_sweep_table(p, rows)
        plotting.plot_energy_sweep(p, rows, out / "energy_sweep.png")
    print(text)
    _write(out / "energy.txt", text)
    en.result.to_csv(out / "trace_endurance_lap.csv")
    print(f"\nwritten: {out}/energy.txt, trace_endurance_lap.csv" + (", energy_sweep.png" if args.param else ""))
    return 0


def cmd_import_track(args) -> int:
    src = Path(args.file)
    if src.suffix.lower() == ".oltra":
        return _import_optimumlap_track(args, src)
    from .importers import read_ac_ai, track_from_ac, write_xy_csv

    sp = read_ac_ai(src)
    closed = None if args.closed is None else args.closed == "yes"
    name = args.name or src.parent.parent.name or src.stem
    track = track_from_ac(src, name=name, closed=closed, cutoff_m=args.cutoff, reverse=args.reverse)
    if args.reverse:
        sp.x, sp.y, sp.elevation = sp.x[::-1], sp.y[::-1], sp.elevation[::-1]
        if sp.side_left is not None:
            sp.side_left, sp.side_right = sp.side_right[::-1], sp.side_left[::-1]
    out = Path(args.out or f"{src.stem}.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    write_xy_csv(sp, out, name, track.closed, args.cutoff)
    k = np.abs(track.kappa)
    step = np.hypot(np.diff(sp.x), np.diff(sp.y))
    lines = [f"Assetto Corsa AI spline v{sp.version}: {sp.n} points, spacing {np.median(step):.2f} m (median)",
             f"track '{name}': length {track.length:.1f} m, {'closed' if track.closed else 'open'}, "
             f"min radius {1.0 / max(k.max(), 1e-9):.1f} m, elevation range {np.ptp(sp.elevation):.1f} m"]
    if sp.side_left is not None:
        w = sp.side_left + sp.side_right
        lines.append(f"track width {w.min():.1f} .. {w.max():.1f} m (median {np.median(w):.1f} m)")
    if sp.lap_time_ms:
        lines.append(f"lap time stored in the file: {sp.lap_time_ms / 1000.0:.3f} s")
    lines.append(f"written: {out}   (curvature cut-off {args.cutoff:g} m; check it with "
                 f"'aerolap run VEHICLE {out}' -> track_map.png)")
    print("\n".join(lines))
    return 0


def _import_optimumlap_track(args, src: Path) -> int:
    from .importers import read_optimumlap_track, write_segment_csv

    ol = read_optimumlap_track(src)
    segs = ol.segments()
    if args.reverse:
        segs = [(k, length, r, {"left": "right", "right": "left"}.get(d, d)) for k, length, r, d in segs[::-1]]
    closed = ol.closed if args.closed is None else args.closed == "yes"
    name = args.name or ol.name
    out = Path(args.out or f"{src.stem}.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    track = Track.from_segments(segs, name=name, closed=closed)
    x, y = track.xy()
    if args.smooth:
        # coordinates of the segment line; Track.from_file smooths them with the given cut-off wavelength
        with open(out, "w", encoding="utf-8") as fh:
            fh.write(f"# name: {name}\n# closed: {'true' if closed else 'false'}\n# cutoff_m: {args.smooth:g}\n"
                     f"# source: OptimumLap track {src.name}, segment line resampled every 0.5 m\n")
            fh.write("x_m,y_m\n")
            n = len(x) - 1 if closed else len(x)
            for i in range(n):
                fh.write(f"{x[i]:.4f},{y[i]:.4f}\n")
        sm = Track.from_file(out)
        from scipy.spatial import cKDTree

        off, _ = cKDTree(np.c_[x, y]).query(np.c_[sm.x, sm.y])
        k_raw, k_sm = np.abs(track.kappa), np.abs(sm.kappa)
        print(f"smoothed line (cut-off {args.smooth:g} m): offset from the segment line max {off.max():.2f} m, "
              f"mean {off.mean():.2f} m; min radius {1 / k_raw.max():.2f} -> {1 / k_sm.max():.2f} m; "
              f"length {track.length:.1f} -> {sm.length:.1f} m")
    else:
        write_segment_csv(segs, out, name, closed, f"OptimumLap track {src.name}")
    corner = ol.directions != 0
    share = {d: float(ol.lengths[ol.directions == v].sum() / ol.length) for d, v in (("left", 1), ("right", -1),
                                                                                       ("straight", 0))}
    p = ol.props.scalars
    lines = [f"OptimumLap track '{ol.name}': {len(segs)} segments, {ol.length:.1f} m, "
             f"{'closed' if closed else 'open'} ({', '.join(ol.header[5:8])})",
             f"min corner radius {ol.radii[corner].min():.2f} m, left / right / straight "
             f"{share['left']:.1%} / {share['right']:.1%} / {share['straight']:.1%}"
             + (f"  (file: {p['percentleft']:.1%} / {p['percentright']:.1%} / {p['percentstraight']:.1%})"
                if "percentleft" in p else ""),
             f"heading change {np.degrees(np.sum(ol.lengths[corner] / ol.radii[corner] * ol.directions[corner])):.1f}"
             f" deg, closure gap {np.hypot(x[-1] - x[0], y[-1] - y[0]):.3f} m",
             f"written: {out}"]
    print("\n".join(lines))
    return 0


def cmd_import_vehicle(args) -> int:
    from .config import load_yaml, vehicle_config_from_dict
    from .importers import read_optimumlap_vehicle, vehicle_cfg_from_optimumlap

    import yaml

    src = Path(args.file)
    ol = read_optimumlap_vehicle(src)
    base_path = Path(args.base)
    base_raw = load_yaml(base_path)
    motor = None
    if args.full:
        motor = {"motor": args.motor, "motor_variant": args.motor_variant, "motor_cooling": args.motor_cooling,
                 "dc_voltage_V": args.dc_voltage}
    raw, taken, assumed, warnings = vehicle_cfg_from_optimumlap(ol, base_raw, drive=args.drive, full=args.full,
                                                                motor=motor)
    out = Path(args.out or f"{src.stem}.yaml")
    out.parent.mkdir(parents=True, exist_ok=True)
    # resolve relative paths of the base (e.g. aero map files) against the base file's folder
    vehicle_config_from_dict(raw, base_path.resolve().parent)
    head = [f"# Imported from OptimumLap vehicle '{ol.name}' ({src.name}) with 'aerolap import-vehicle'.",
            "# " + ("Full model: OptimumLap mass / tyres / aero + " + str(args.motor) + " motor model."
                    if args.full else "OptimumLap-equivalent (use with --level L0 to compare with OptimumLap)."),
            "# Taken from the OptimumLap file:"]
    head += [f"#   {t}" for t in taken]
    try:
        base_txt = base_path.resolve().relative_to(Path.cwd()).as_posix()
    except ValueError:
        base_txt = base_path.as_posix()
    head += [f"# ASSUMED - not in the OptimumLap file, copied from {base_txt}; replace with real values:"]
    head += [f"#   {a}" for a in assumed]
    head += [f"# WARNING: {w}" for w in warnings]
    text = "\n".join(head) + "\n" + yaml.safe_dump(raw, sort_keys=False, allow_unicode=True)
    out.write_text(text, encoding="utf-8")
    g = ol.get
    rpm, tq = ol.torque_curve
    ratio = g("finaldriveratio", 1.0) * float(ol.gear_ratios[-1])
    v_top = ol.rev_limit_rpm * 2 * np.pi / 60.0 / ratio * g("tireradius")
    lines = [f"OptimumLap vehicle '{ol.name}' ({ol.transmission or 'transmission ?'}, {len(ol.gear_ratios)} gear(s))",
             f"mass {g('mass'):.1f} kg, mu x/y {g('longfriction'):.2f}/{g('latfriction'):.2f}, "
             f"ClA {g('liftcoefficient') * g('frontarea'):.3f} m^2, CdA {g('dragcoefficient') * g('frontarea'):.3f} m^2, "
             f"rho {g('airdensity'):.3f}",
             f"torque {tq.max():.1f} Nm max, {(tq * rpm).max() * 2 * np.pi / 60 / 1000:.1f} kW max at the shaft, "
             f"rev limit {ol.rev_limit_rpm:.0f} rpm, final drive {g('finaldriveratio'):g}, tyre radius "
             f"{g('tireradius') * 1000:.0f} mm -> top speed {v_top * 3.6:.1f} km/h"
             + (f" (file: {g('topSpeed') * 3.6:.1f} km/h)" if g("topSpeed") else "")]
    lines += [f"WARNING: {w}" for w in warnings]
    lines.append(f"written: {out}")
    print("\n".join(lines))
    return 0


def cmd_aero_split(args) -> int:
    from .analysis.aero_split import aero_split

    cfg = _vehicle_cfg(args)
    spec = {"type": args.objective, "level": args.level, "track": str(Path(args.track).resolve())}
    if args.objective == "endurance_time":
        spec["distance_km"] = args.distance_km
    elif args.start:
        spec["start"] = args.start
    constraints = {"understeer_index": [float(x) for x in args.understeer.split(",")]} if args.understeer else {}
    if args.min_clearance_mm is not None:
        constraints["min_clearance_mm"] = args.min_clearance_mm
    bounds = tuple(float(x) for x in args.bounds.split(",")) if args.bounds else None
    names = args.components.split(",") if args.components else None
    res = aero_split(cfg, spec, ".", constraints, names, bounds, args.points, not args.no_joint,
                     args.max_evals, SolverOptions.coarse(), lambda m: print("  " + m, flush=True))
    out = _out(args, "out/aero_split")
    text = f"vehicle: {cfg.get('name')}\n" + res.report(cfg)
    print("\n" + text)
    _write(out / "aero_split.txt", text)
    plotting.plot_aero_split(res, out / "aero_split.png")
    files = "aero_split.txt, aero_split.png"
    if res.joint is not None:
        dump_config(res.joint.best_cfg, out / "optimized_vehicle.yaml")
        files += ", optimized_vehicle.yaml"
    print(f"\nwritten: {out}/{files}")
    return 0


def cmd_aero_exchange(args) -> int:
    from .gui_tasks import task_exchange

    res = task_exchange(_vehicle_cfg(args), args.track, args.level, args.coarse, _out(args, "out/aero_exchange"),
                        log=lambda m: print("  " + m.splitlines()[0], flush=True), objective=args.objective,
                        distance_km=args.distance_km)
    print("\n" + res.text)
    _write(res.out_dir / "aero_exchange.txt", res.text)
    print(f"\nwritten: {res.out_dir}/aero_exchange.txt, aero_exchange.png")
    return 0


def cmd_aero_envelope(args) -> int:
    from .gui_tasks import task_envelope

    res = task_envelope(_vehicle_cfg(args), args.track, args.level, args.coarse, _out(args, "out/aero_envelope"))
    print(res.text)
    _write(res.out_dir / "aero_envelope.txt", res.text)
    print(f"\nwritten: {res.out_dir}/aero_envelope.txt, aero_envelope.png")
    return 0


def cmd_rank_sections(args) -> int:
    from .gui_tasks import task_rank_sections

    us = tuple(float(x) for x in args.understeer.split(",")) if args.understeer else None
    res = task_rank_sections(_vehicle_cfg(args), args.track, args.level, True, _out(args, "out/rank_sections"),
                             log=lambda m: print("  " + m, flush=True), section_csv=args.summary,
                             section_component=args.component, objective=args.objective,
                             distance_km=args.distance_km, understeer=us, max_geometries=args.max_geometries)
    print("\n" + res.text)
    _write(res.out_dir / "section_rank.txt", res.text)
    print(f"\nwritten: {res.out_dir}/section_rank.txt, section_rank.png, best_vehicle.yaml")
    return 0


def cmd_auto_aero(args) -> int:
    from .gui_tasks import task_auto_aero

    us = tuple(float(x) for x in args.understeer.split(",")) if args.understeer else None
    bounds = {}
    for b in args.bounds or []:
        name, rng = b.split("=", 1)
        bounds[name.strip()] = [float(x) for x in rng.split(",")]
    fixed = [f.strip() for item in (args.fix or []) for f in item.split(",") if f.strip()]
    res = task_auto_aero(_vehicle_cfg(args), args.track, args.level, True, _out(args, "out/auto_aero"),
                         log=lambda m: print("  " + m, flush=True), objective=args.objective,
                         distance_km=args.distance_km, understeer=us, fixed=fixed, design_bounds=bounds,
                         options=not args.no_options, max_evals=args.max_evals, max_vars=args.max_vars,
                         min_clearance_mm=args.min_clearance_mm, cooling_margin_kW=args.cooling_margin_kw,
                         dry_run=args.dry_run)
    print("\n" + res.text)
    _write(res.out_dir / "auto_aero.txt", res.text)
    print(f"\nwritten: {res.out_dir}/auto_aero.txt" + ("" if args.dry_run else ", auto_aero.png, optimized_vehicle.yaml"))
    return 0


def cmd_cfd_plan(args) -> int:
    from .analysis.cfd_plan import cfd_plan

    cfg = _vehicle_cfg(args)
    speeds = [float(x) for x in args.speeds.split(",")] if args.speeds else None
    plan = cfd_plan(cfg, args.component, args.vary.split(",") if args.vary else None, args.aoa_span, args.aoa_step,
                    speeds)
    out = _out(args, f"out/cfd_plan_{args.component}")
    files = plan.write(out)
    print(plan.report())
    print("\nwritten: " + ", ".join(str(f) for f in files))
    print("copy them into an AERO_AUTOMATION project folder (with the journal and base cases), run "
          "'aero-auto screen / mesh / run / collect', then set on the wing\n"
          "  section_data: <folder>/runs/summary.csv\n  section_fit: true\nand run 'aerolap auto-aero' again")
    return 0


def cmd_energy_strategy(args) -> int:
    """Energy-optimal endurance driving (lift-and-coast, throttle) vs the power-cap derate."""
    from .analysis.energy_strategy import endurance_strategy

    cfg = _vehicle_cfg(args)
    track = Track.from_file(args.track, closed=True)
    st = endurance_strategy(cfg, track, args.distance_km, features_for_level(args.level), _opts(args), dv=args.dv,
                            target_kWh=args.target_kwh, pareto=not args.no_pareto, progress=print)
    out = _out(args, "out/energy_strategy")
    text = f"vehicle: {cfg.get('name')}\n" + st.report()
    print("\n" + text)
    _write(out / "energy_strategy.txt", text)
    st.result.to_csv(out / "trace_optimal.csv")
    if st.cap.result is not None:
        st.cap.result.to_csv(out / "trace_power_cap.csv")
    with open(out / "lift_points.csv", "w", encoding="utf-8") as fh:
        fh.write("lift_at_m,brake_at_m,coast_m,v_lift_kmh,v_end_kmh\n")
        for p in st.lifts:
            fh.write(f"{p.s_lift:.1f},{p.s_brake:.1f},{p.coast_m:.1f},{p.v_lift_kmh:.1f},{p.v_end_kmh:.1f}\n")
    plotting.plot_energy_strategy(st, out / "energy_strategy.png")
    print(f"\nwritten: {out}/energy_strategy.txt, energy_strategy.png, lift_points.csv, trace_optimal.csv, "
          "trace_power_cap.csv")
    return 0


def cmd_line(args) -> int:
    """Racing line inside the track width; writes it as a track CSV for every other command."""
    from .racing_line import DEFAULT_WEIGHTS, optimise_line

    cfg = _vehicle_cfg(args)
    car = VehicleModel(cfg, features_for_level(args.level))
    track = Track.from_file(args.track)
    if args.width is not None:
        track = track.with_width(args.width)
    weights = [float(w) for w in args.weights.split(",")] if args.weights else DEFAULT_WEIGHTS
    res = optimise_line(car, track, car_width=args.car_width, clearance=args.clearance, step=args.step,
                        weights=weights, opts=_opts(args), start=args.start,
                        progress=lambda k, n, w, t: print(f"  [{k}/{n}] length weight {w:g}: lap {t:.3f} s"))
    out = _out(args, "out/line")
    text = f"vehicle: {car.name}   model: {car.features.label}\n" + res.report()
    print(text)
    _write(out / "racing_line.txt", text)
    csv_path = res.write_csv(out / "racing_line.csv", name=f"{track.name} - racing line")
    res.lap.to_csv(out / "trace.csv")
    plotting.plot_racing_line(res, out / "racing_line.png")
    print(f"\nwritten: {out}/racing_line.txt, racing_line.png, trace.csv and the line as a track: {csv_path}")
    return 0


def cmd_tire(args) -> int:
    """Inspect a Magic Formula .tir file (or the tyre of a vehicle file)."""
    from .tire import tire_from_config

    if str(args.file).lower().endswith(".tir"):
        t = {"radius_mm": 203.0, "vertical_rate_N_per_mm": 100.0, "rolling_resistance": 0.015,
             "ref_load_N": args.ref_load, "mu_x": args.mu_x or 1.0, "mu_y": args.mu_y or 1.0,
             "mf_file": str(Path(args.file).resolve()), "pressure_kPa": args.pressure,
             "mf_mu_scaling": bool(args.mu_x or args.mu_y)}
        tire = tire_from_config(t)
    else:
        args.vehicle = args.file
        cfg = _vehicle_cfg(args)
        if args.pressure is not None:
            cfg = with_overrides(cfg, {"tires.pressure_kPa": args.pressure})
        tire = VehicleModel(cfg).tire
    print(tire.describe())
    if getattr(tire, "model", "simple") != "mf":
        print("(simple tyre model: give tires.mf_file in the vehicle file for the Magic Formula)")
        return 0
    loads = [float(x) for x in args.loads.split(",")] if args.loads else None
    rows = [f"{'Fz [N]':>8s} {'mu_x':>7s} {'mu_y':>7s} {'Ky [N/deg]':>11s} {'peak slip [deg]':>16s} "
            f"{'peak kappa':>11s} {'best lean [deg]':>16s}"]
    import math

    for fz in loads or [0.5 * tire.fz_ref, tire.fz_ref, 1.5 * tire.fz_ref, 2.0 * tire.fz_ref, 3.0 * tire.fz_ref]:
        g, _ = tire.camber_optimum(fz)
        rows.append(f"{fz:8.0f} {float(tire.mu_x(fz)):7.3f} {float(tire.mu_y(fz)):7.3f} "
                    f"{float(tire.cornering_stiffness(fz)) * math.pi / 180:11.0f} "
                    f"{math.degrees(float(tire.peak_slip_at(fz))):16.2f} {float(tire.peak_slip_ratio_at(fz)):11.3f} "
                    f"{math.degrees(g):16.1f}")
    text = tire.describe() + "\n\n" + "\n".join(rows)
    print("\n" + "\n".join(rows))
    out = _out(args, "out/tire")
    _write(out / "tire.txt", text)
    plotting.plot_tire(tire, out / "tire.png", loads)
    print(f"\nwritten: {out}/tire.txt, tire.png")
    return 0


def cmd_fit_map(args) -> int:
    from .aero.maps import ResponseSurfaceAero, grid_rows

    rs = ResponseSurfaceAero.from_csv(args.csv)
    for fit in rs.fits:
        print(f"mode {fit.mode}: {fit.n_points} points, variables {', '.join(fit.dims) or '-'}")
        for t in rs.TARGETS:
            print(f"    {t:6s} R^2 = {fit.r2[t]:.4f}   rms = {fit.rms[t]:.4f} m^2")
    if args.export_grid:
        rows = grid_rows(rs, step_mm=args.step_mm, step_deg=args.step_deg)
        with open(args.export_grid, "w", encoding="utf-8") as fh:
            fh.write(f"# grid exported from the response surface fitted to {args.csv}\n")
            fh.write("\n".join(rows) + "\n")
        print(f"written: {args.export_grid} (use with aero.type: table)")
    return 0


# ---------------------------------------------------------------------- parser
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="aerolap", description=(
        "FSAE quasi-steady-state four-wheel lap simulator with suspension-coupled aerodynamics"))
    sub = p.add_subparsers(dest="command", required=True)

    def common(sp, track=False, level=True):
        sp.add_argument("vehicle", help="vehicle YAML file or OptimumLap .olveh")
        if track:
            sp.add_argument("track", help="track CSV file")
            sp.add_argument("--start", choices=["flying", "standing"],
                            help="default: flying for closed tracks, standing for open ones")
        if level:
            sp.add_argument("--level", default="full",
                            help="model level L0 (OptimumLap-like) ... L5/full (default)")
        sp.add_argument("--set", action="append", metavar="PATH=VALUE",
                        help="override a vehicle parameter, e.g. aero.cop_shift=0.02 (repeatable)")
        sp.add_argument("--out", help="output directory")
        sp.add_argument("--ds", type=float, help="track step [m] (default 0.5)")
        sp.add_argument("--coarse", action="store_true", help="faster, slightly less accurate envelope")

    sp = sub.add_parser("run", help="simulate one lap")
    common(sp, track=True)
    sp.add_argument("--mat", action="store_true", help="also write the trace as MATLAB trace.mat")
    sp.set_defaults(func=cmd_run)

    sp = sub.add_parser("transient", help="time-domain lap (suspension / aero dynamics) vs QSS")
    common(sp, track=True)
    sp.add_argument("--dt", type=float, default=0.005, help="time step [s] (default 0.005)")
    sp.add_argument("--iterations", type=int, default=4, help="driver learning laps (default 4)")
    sp.add_argument("--max-jerk", type=float, default=150.0, help="longitudinal jerk limit [m/s^3]")
    sp.add_argument("--turn-in", type=float, default=3.0, help="curvature smoothing length [m]")
    sp.add_argument("--road", help="rough road: ISO 8608 class A..E or a profile CSV (s_m,z_left_mm,z_right_mm)")
    sp.add_argument("--road-seed", type=int, default=1, help="random seed of the ISO road")
    sp.add_argument("--road-scale", type=float, default=1.0, help="road amplitude factor")
    sp.add_argument("--no-lateral", action="store_true",
                    help="follow the line exactly (no lateral / yaw dynamics, faster)")
    sp.set_defaults(func=cmd_transient)

    sp = sub.add_parser("compare", help="effect ladder: OptimumLap-equivalent (L0) -> full model (L5)")
    common(sp, track=True, level=False)
    sp.add_argument("--levels", help="comma separated subset, e.g. L0,L3,L5")
    sp.set_defaults(func=cmd_compare)

    sp = sub.add_parser("cornering", help="steady-state cornering and braking limits vs speed")
    common(sp)
    sp.add_argument("--speeds", default="30:120:10", help="km/h, a:b:step or list (default 30:120:10)")
    sp.add_argument("--mode", help="aero mode (default: aero.default_mode)")
    sp.set_defaults(func=cmd_cornering)

    sp = sub.add_parser("ggv", help="g-g-V envelope")
    common(sp)
    sp.add_argument("--all-modes", action="store_true", help="plot every aero mode")
    sp.add_argument("--export", action="store_true",
                    help="write the envelope as ggv.csv and MATLAB ggv.mat (see matlab/aerolap_ggv_demo.m)")
    sp.set_defaults(func=cmd_ggv)

    def objective_args(sp):
        sp.add_argument("--objective", default="lap_time", choices=["lap_time", "skidpad_time", "ay_max"])
        sp.add_argument("--track", help="track CSV (lap_time objective)")
        sp.add_argument("--start", choices=["flying", "standing"])
        sp.add_argument("--speeds-kmh", default="40,60,80,100", help="ay_max objective speeds")
        sp.add_argument("--min-clearance-mm", type=float)
        sp.add_argument("--max-roll-deg", type=float)
        sp.add_argument("--understeer", help="allowed understeer index range lo,hi")
        sp.add_argument("--solver", choices=["qss", "transient"], default="qss",
                        help="lap_time objective: quasi-steady-state or transient (slow)")
        sp.add_argument("--road", help="transient: ISO 8608 class A..E or a road profile CSV")

    sp = sub.add_parser("sweep", help="1-D / 2-D parameter sweep")
    common(sp)
    sp.add_argument("--param", action="append", required=True, metavar="PATH=a:b:step",
                    help="parameter and values, e.g. aero.cop_shift=-0.04:0.04:0.01 (max two)")
    objective_args(sp)
    sp.set_defaults(func=cmd_sweep)

    sp = sub.add_parser("sensitivity", help="finite-difference sensitivity of the objective")
    common(sp)
    sp.add_argument("--params", required=True, help="comma separated parameter paths")
    sp.add_argument("--rel-step", type=float, default=0.1)
    objective_args(sp)
    sp.set_defaults(func=cmd_sensitivity)

    sp = sub.add_parser("optimize", help="setup optimisation from a spec YAML")
    sp.add_argument("spec", help="optimisation spec YAML (see data/optimize/)")
    sp.add_argument("--method", choices=["powell", "differential_evolution"])
    sp.add_argument("--max-evals", type=int)
    sp.add_argument("--set", action="append", metavar="PATH=VALUE")
    sp.add_argument("--out", help="output directory")
    sp.set_defaults(func=cmd_optimize)

    sp = sub.add_parser("events", help="FSAE acceleration, skidpad, autocross, endurance")
    common(sp)
    sp.add_argument("--autocross", help="autocross track CSV (standing start)")
    sp.add_argument("--endurance", help="endurance track CSV (closed, flying laps)")
    sp.add_argument("--distance-km", type=float, default=22.0, help="endurance distance")
    sp.set_defaults(func=cmd_events)

    sp = sub.add_parser("energy", help="endurance energy budget: battery, drag share, power cap")
    common(sp, track=True)
    sp.add_argument("--distance-km", type=float, default=22.0, help="endurance distance (default 22)")
    sp.add_argument("--param", metavar="PATH=a:b:step",
                    help="sweep one parameter, e.g. aero.cla_scale=0.6:1.4:0.2")
    sp.add_argument("--relative", action="store_true",
                    help="sweep values multiply the current value(s) instead of replacing them")
    sp.set_defaults(func=cmd_energy)

    sp = sub.add_parser("import-track",
                        help="convert an Assetto Corsa AI line (fast_lane.ai) or an OptimumLap track (.oltra) to CSV")
    sp.add_argument("file", help="<track>/ai/fast_lane.ai, ideal_line.ai or an OptimumLap .oltra")
    sp.add_argument("--out", help="output CSV (default: <file stem>.csv)")
    sp.add_argument("--name", help="track name")
    sp.add_argument("--cutoff", type=float, default=10.0,
                    help="smoothing wavelength [m]: shorter wiggles of the line are treated as noise (default 10)")
    sp.add_argument("--reverse", action="store_true", help="drive the line in the opposite direction")
    sp.add_argument("--closed", choices=["yes", "no"], help="override the closed-loop detection")
    sp.add_argument("--smooth", type=float, metavar="M",
                    help=".oltra: write the line as x-y coordinates smoothed with this cut-off wavelength [m] "
                         "(removes the curvature noise of cone-map tracks)")
    sp.set_defaults(func=cmd_import_track)

    sp = sub.add_parser("import-vehicle", help="convert an OptimumLap vehicle (.olveh) to an aerolap vehicle YAML")
    sp.add_argument("file", help="OptimumLap .olveh file")
    sp.add_argument("--out", help="output YAML (default: <file stem>.yaml)")
    sp.add_argument("--drive", choices=["rwd", "awd"], default="rwd",
                    help="driven axle (OptimumLap has no axle model; awd = traction on the whole car)")
    sp.add_argument("--base", default=str(Path(__file__).resolve().parents[2] / "data" / "vehicles" / "fsae_ev_aero.yaml"),
                    help="vehicle YAML that supplies everything the OptimumLap file lacks (marked ASSUMED)")
    sp.add_argument("--full", action="store_true",
                    help="use a motor model (--motor) with a battery-side power limit instead of the torque curve")
    sp.add_argument("--motor", default="emrax_228")
    sp.add_argument("--motor-variant", default="MV")
    sp.add_argument("--motor-cooling", default="LC")
    sp.add_argument("--dc-voltage", type=float, default=550.0, help="accumulator voltage under load [V]")
    sp.set_defaults(func=cmd_import_vehicle)

    sp = sub.add_parser("aero-split", help="best size of each aero device (front / side / rear) for a track")
    common(sp, track=True)
    sp.add_argument("--objective", choices=["lap_time", "endurance_time"], default="lap_time",
                    help="lap_time (one lap) or endurance_time (energy-limited, battery power cap)")
    sp.add_argument("--distance-km", type=float, default=22.0)
    sp.add_argument("--components", help="comma separated devices (all their variables) or variables such as "
                    "front.flap1 (default: all)")
    sp.add_argument("--bounds", help="range lo,hi of scale-type devices (default: each device's range; "
                                    "wings use their angle range in degrees)")
    sp.add_argument("--points", type=int, default=9, help="points per individual sweep")
    sp.add_argument("--understeer", default="1.0,1.3",
                    help="allowed understeer index lo,hi at the cornering limit ('' = no constraint)")
    sp.add_argument("--min-clearance-mm", type=float)
    sp.add_argument("--max-evals", type=int, default=80, help="joint optimisation evaluations")
    sp.add_argument("--no-joint", action="store_true", help="only the individual sweeps")
    sp.set_defaults(func=cmd_aero_split)

    sp = sub.add_parser("aero-exchange", help="what ClA / CdA / balance / mass / device steps are worth")
    common(sp, track=True)
    sp.add_argument("--objective", choices=["lap_time", "endurance_time"], default="lap_time")
    sp.add_argument("--distance-km", type=float, default=22.0)
    sp.set_defaults(func=cmd_aero_exchange)

    sp = sub.add_parser("aero-envelope", help="aero operating envelope of a lap: ride heights on the map, balance")
    common(sp, track=True)
    sp.set_defaults(func=cmd_aero_envelope)

    sp = sub.add_parser("rank-sections",
                        help="rank AERO_AUTOMATION 2-D wing sections (runs/summary.csv) by lap / endurance time")
    common(sp, track=True)
    sp.add_argument("summary", help="AERO_AUTOMATION runs/summary.csv (ground effect: front wing, free stream: rear)")
    sp.add_argument("--component", default="front", help="wing in aero.components to put the sections on (front / rear)")
    sp.add_argument("--objective", choices=["lap_time", "endurance_time"], default="lap_time")
    sp.add_argument("--distance-km", type=float, default=22.0)
    sp.add_argument("--understeer", default="1.0,1.3", help="allowed understeer index lo,hi ('' = none)")
    sp.add_argument("--max-geometries", type=int, help="only the first N sections (quick look)")
    sp.set_defaults(func=cmd_rank_sections)

    sp = sub.add_parser("auto-aero", help="optimise EVERY aero parameter and on/off option except the ones kept "
                                          "(--fix), for lap or endurance time")
    common(sp, track=True)
    sp.add_argument("--objective", choices=["endurance_time", "lap_time"], default="endurance_time",
                    help="default: energy-limited endurance time (track = endurance lap)")
    sp.add_argument("--distance-km", type=float, default=22.0)
    sp.add_argument("--fix", action="append", metavar="PATTERNS",
                    help="parameters to keep, comma separated: a device (rear), a name (front.elements.flap1.gap_mm), "
                         "a pattern (*.span_m) or a group (@position @chord @slot @size @angle @cooling @option); "
                         "adds to aero.design.fixed of the vehicle file")
    sp.add_argument("--bounds", action="append", metavar="NAME=lo,hi", help="range of a parameter (or pattern)")
    sp.add_argument("--no-options", action="store_true", help="keep every on/off option as it is")
    sp.add_argument("--understeer", default="1.0,1.3", help="allowed understeer index lo,hi ('' = none)")
    sp.add_argument("--min-clearance-mm", type=float)
    sp.add_argument("--cooling-margin-kw", type=float, help="radiators must reject the heat + this margin")
    sp.add_argument("--max-evals", type=int, default=300, help="search evaluations (coarse envelope)")
    sp.add_argument("--max-vars", type=int, default=24, help="search at most this many of the most important variables")
    sp.add_argument("--dry-run", action="store_true", help="only list the design space (free / kept, ranges)")
    sp.set_defaults(func=cmd_auto_aero)

    sp = sub.add_parser("cfd-plan", help="write the next AERO_AUTOMATION batch (project.toml + cases.csv) around "
                                         "a wing's element values")
    common(sp, level=False)
    sp.add_argument("--component", default="front", help="wing with 'elements' (front / rear / side)")
    sp.add_argument("--vary", help="element values to vary, e.g. flap1.deflection,flap1.gap (default: all)")
    sp.add_argument("--aoa-span", type=float, default=4.0, help="aoa range around the wing angle [deg]")
    sp.add_argument("--aoa-step", type=float, default=2.0)
    sp.add_argument("--speeds", help="speeds km/h, comma separated (default 40,70,100)")
    sp.set_defaults(func=cmd_cfd_plan)

    sp = sub.add_parser("energy-strategy", help="energy-optimal endurance driving (lift-and-coast, throttle) "
                                                "vs the power-cap derate, same battery energy")
    common(sp, track=True)
    sp.add_argument("--distance-km", type=float, default=22.0, help="endurance distance (default 22)")
    sp.add_argument("--target-kwh", type=float, help="energy budget for the whole distance [kWh] "
                                                     "(default: the usable battery energy)")
    sp.add_argument("--dv", type=float, default=0.1, help="speed grid of the dynamic programming [m/s] (0.1)")
    sp.add_argument("--no-pareto", action="store_true", help="skip the time-energy trade-off curves (faster)")
    sp.set_defaults(func=cmd_energy_strategy)

    sp = sub.add_parser("line", help="racing line inside the track width (minimum curvature, length weight "
                                     "chosen by lap time) -> track CSV")
    common(sp, track=True)
    sp.add_argument("--width", type=float,
                    help="track width [m] (default: the track's width_left_m / width_right_m columns)")
    sp.add_argument("--car-width", type=float, help="car width [m] (default: wider track + 0.2 m)")
    sp.add_argument("--clearance", type=float, default=0.2, help="distance kept from the track edges [m] (0.2)")
    sp.add_argument("--step", type=float, default=1.0, help="spacing of the line points [m] (1.0)")
    sp.add_argument("--weights", help="path-length weights to try, comma separated "
                                      "(default 0,0.002,0.005,0.01,0.02,0.05; 0 = minimum curvature)")
    sp.set_defaults(func=cmd_line)

    sp = sub.add_parser("tire", help="inspect a Magic Formula tyre (.tir file or the tyre of a vehicle file)")
    sp.add_argument("file", help="Pacejka .tir file, or a vehicle YAML with tires.mf_file")
    sp.add_argument("--set", action="append", metavar="PATH=VALUE")
    sp.add_argument("--out", help="output directory")
    sp.add_argument("--loads", help="vertical loads [N], comma separated")
    sp.add_argument("--pressure", type=float, help="inflation pressure [kPa] (MF 6.1 pressure terms)")
    sp.add_argument("--ref-load", type=float, default=650.0, help=".tir: reference load for --mu-x / --mu-y [N]")
    sp.add_argument("--mu-x", type=float, help=".tir: scale the friction to this mu_x at the reference load")
    sp.add_argument("--mu-y", type=float, help=".tir: scale the friction to this mu_y at the reference load")
    sp.set_defaults(func=cmd_tire, vehicle=None)

    sp = sub.add_parser("fit-map", help="fit a quadratic response surface to scattered CFD points")
    sp.add_argument("csv", help="CFD points CSV (aero map format)")
    sp.add_argument("--export-grid", help="write a full-factorial grid CSV for aero.type: table")
    sp.add_argument("--step-mm", type=float, default=5.0)
    sp.add_argument("--step-deg", type=float, default=0.5)
    sp.set_defaults(func=cmd_fit_map)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return int(args.func(args) or 0)
    except (ValueError, KeyError, FileNotFoundError) as exc:
        from .gui_tasks import error_text

        print("error:\n" + error_text(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
