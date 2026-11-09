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
from .config import dump_config, load_vehicle_config, parse_override, with_overrides
from .features import LEVELS, features_for_level
from .solvers.ggv import SolverOptions
from .solvers.qss import QSSSolver
from .solvers.transient import TransientOptions, TransientSolver
from .track import Track
from .vehicle import VehicleModel


def _vehicle_cfg(args) -> dict:
    cfg = load_vehicle_config(args.vehicle)
    over = dict(parse_override(s) for s in (args.set or []))
    return with_overrides(cfg, over) if over else cfg


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
    print(text)
    _write(out / "summary.txt", text)
    res.to_csv(out / "trace.csv")
    plotting.plot_lap(res, out / "lap.png")
    plotting.plot_track_map(res, out / "track_map.png")
    print(f"\nwritten: {out}/summary.txt, trace.csv, lap.png, track_map.png")
    return 0


def cmd_transient(args) -> int:
    cfg = _vehicle_cfg(args)
    car = VehicleModel(cfg, features_for_level(args.level))
    track = Track.from_file(args.track)
    start = args.start or ("flying" if track.closed else "standing")
    opts = TransientOptions(dt=args.dt, iterations=args.iterations, max_jerk=args.max_jerk,
                            turn_in_length=args.turn_in, qss=_opts(args))
    solver = TransientSolver(opts)
    tr = solver.solve(car, track, start=start)
    qss = solver.qss_reference
    sq, st = qss.summary(), tr.summary()
    keys = ["lap_time_s", "v_max_kmh", "v_min_kmh", "drive_energy_kWh", "min_clearance_mm", "min_rh_front_mm",
            "min_rh_rear_mm", "max_roll_deg", "max_yaw_deg", "non_default_mode_pct"]
    lines = [f"vehicle: {car.name}   model: {car.features.label}", f"track:   {track.name} ({start})",
             f"QSS on the original segment line: {tr.info['qss_lap_time_s']:.3f} s "
             f"(the comparison below uses the {args.turn_in:g} m turn-in-smoothed line)", "",
             f"{'':24s} {'QSS':>12s} {'transient':>12s} {'change':>10s}", "-" * 62]
    for k in keys:
        lines.append(f"{k:24s} {sq[k]:12.4f} {st[k]:12.4f} {st[k] - sq[k]:+10.4f}")
    lines += ["", "learning laps (driver lowers its target where the tyres were over-used):"]
    for h in tr.info["iterations"]:
        lines.append(f"  #{h['iteration']}  target {h['reference_lap_s']:8.3f} s   driven {h['lap_s']:8.3f} s   "
                     f"max tyre over-use {h['max_lateral_excess'] * 100:6.2f} %")
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
    print(f"\nwritten: {out}/ggv.png")
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
        sp.add_argument("vehicle", help="vehicle YAML file")
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
    sp.set_defaults(func=cmd_run)

    sp = sub.add_parser("transient", help="time-domain lap (suspension / aero dynamics) vs QSS")
    common(sp, track=True)
    sp.add_argument("--dt", type=float, default=0.005, help="time step [s] (default 0.005)")
    sp.add_argument("--iterations", type=int, default=4, help="driver learning laps (default 4)")
    sp.add_argument("--max-jerk", type=float, default=150.0, help="longitudinal jerk limit [m/s^3]")
    sp.add_argument("--turn-in", type=float, default=3.0, help="curvature smoothing length [m]")
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
    sp.set_defaults(func=cmd_ggv)

    def objective_args(sp):
        sp.add_argument("--objective", default="lap_time", choices=["lap_time", "skidpad_time", "ay_max"])
        sp.add_argument("--track", help="track CSV (lap_time objective)")
        sp.add_argument("--start", choices=["flying", "standing"])
        sp.add_argument("--speeds-kmh", default="40,60,80,100", help="ay_max objective speeds")
        sp.add_argument("--min-clearance-mm", type=float)
        sp.add_argument("--max-roll-deg", type=float)
        sp.add_argument("--understeer", help="allowed understeer index range lo,hi")

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
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
