"""Automatic optimisation of the whole aero package: every parameter and option that is not kept.

``aero-split`` moves the wing angles. ``auto-aero`` frees *everything* the aero devices are made
of (:mod:`aerolap.analysis.design_space`: angles, slots, chords, span, position, radiator areas and
the on / off options of devices and flaps) except the parameters the user keeps, and minimises the
objective - by default the energy-limited endurance time - under the setup constraints.

Stages (all with the search accuracy = coarse envelope; ``max_evals`` is the total budget):

1. **options** - each free on / off option (a device, a flap) is flipped in turn; the wing angles
   are re-trimmed for it (taking a wing off shifts the balance, so an untrimmed comparison would
   only ever show the constraint violation) and the best flip is taken if it beats the trimmed
   current car. Repeated while a flip helps.
2. **screen** - one step of every free variable: what each is worth, and which ones matter. The
   variables below ``screen_tol`` of the largest effect are left alone (``max_vars`` caps the count).
3. **search** - Hooke-Jeeves pattern search over the remaining variables, most important first
   (coordinate steps with a parabolic fit, a pattern move after each improving pass, step halving).
   The penalised objective is used, as in ``aerolap optimize``.
4. **check** - the start car and the best candidates again with the fine envelope; the best
   feasible one is the result. If nothing beats the start car, the start car is returned.

The devices are the models of :mod:`aerolap.aero.devices`: with the plain element model the slot
optimum is the model's built-in best gap / overlap, so treat gaps and overlaps from it as "leave
them near the usual values" - with ``section_fit: true`` they come from the AERO_AUTOMATION data.
"""

from __future__ import annotations

import copy
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..aero.devices import geometry_from_cfg
from ..aero.maps import component_table, parse_components
from ..config import get_path, with_overrides
from ..solvers.ggv import SolverOptions
from .design_space import DesignSpace, DesignVar, design_space
from .objectives import Evaluation, Objective


@dataclass
class OptionTrial:
    name: str
    to: bool
    evaluation: Evaluation
    trim: dict[str, float]
    accepted: bool = False


@dataclass
class Screened:
    var: DesignVar
    step: float
    delta: float  # change of the penalised objective for the step
    used: bool = False


@dataclass
class AutoAeroResult:
    label: str
    unit: str
    space: DesignSpace  # at the start
    final_space: DesignSpace  # after the option stage (the variables that were searched)
    initial_eval: Evaluation  # fine envelope
    best_eval: Evaluation  # fine envelope
    initial_search: Evaluation
    best_search: Evaluation
    best_cfg: dict
    changes: dict[str, tuple[float, float]] = field(default_factory=dict)  # path -> (before, after)
    option_changes: dict[str, tuple[bool, bool]] = field(default_factory=dict)
    trials: list[OptionTrial] = field(default_factory=list)
    trimmed: Evaluation | None = None  # current options, angles re-trimmed (the options' reference)
    screen: list[Screened] = field(default_factory=list)
    history: list[tuple[int, str, float, float, bool]] = field(default_factory=list)  # n, stage, value, total, ok
    evals: int = 0
    elapsed_s: float = 0.0
    improved: bool = True

    def report(self, cfg: dict) -> str:
        i, b = self.initial_eval, self.best_eval
        lines = [f"objective: {self.label} [{self.unit}]   evaluations: {self.evals}   time: {self.elapsed_s:.0f} s", "",
                 "design space (kept = not changed):", self.space.table(), ""]
        if self.trials:
            ref = self.trimmed.value if self.trimmed else self.initial_search.value
            lines.append(f"options - each flipped with the wing angles re-trimmed (search accuracy; current options, "
                         f"trimmed: {ref:.3f}):")
            for t in self.trials:
                lines.append(f"  {t.name:40s} -> {'on ' if t.to else 'off'} {t.evaluation.value:10.3f} "
                             f"({t.evaluation.value - ref:+.3f})"
                             f"{'' if t.evaluation.feasible else '  violates a constraint'}"
                             f"{'   <- taken' if t.accepted else ''}")
            lines.append("")
        if self.screen:
            lines.append("what each free variable is worth (one step from the start, search accuracy; * = searched):")
            for s in sorted(self.screen, key=lambda s: -abs(s.delta)):
                lines.append(f"  {'*' if s.used else ' '} {s.var.name:44s} {s.step:+9.3g} {s.var.unit:3s} -> "
                             f"{s.delta:+9.4f} {self.unit}")
            lines.append("")
        lines.append(f"{'parameter':44s} {'before':>10s} {'after':>10s} {'range':>22s}")
        lines.append("-" * 90)
        for name, (a, bb) in self.option_changes.items():
            lines.append(f"{name:44s} {'on' if a else 'off':>10s} {'on' if bb else 'off':>10s}")
        by_path = {v.path: v for v in self.final_space.variables}
        for path, (a, bb) in self.changes.items():
            v = by_path.get(path)
            rng = f"[{v.lo:.4g}, {v.hi:.4g}]" if v else ""
            edge = "  at the range limit" if v and (abs(bb - v.lo) < 1e-6 * (v.hi - v.lo) + 1e-12
                                                    or abs(bb - v.hi) < 1e-6 * (v.hi - v.lo) + 1e-12) else ""
            lines.append(f"{path[len('aero.components.'):]:44s} {a:10.4g} {bb:10.4g} {rng:>22s}{edge}")
        if not self.changes and not self.option_changes:
            lines.append("(nothing changed)")
        lines += ["", f"{'':28s} {'before':>12s} {'after':>12s} {'change':>10s}   (fine envelope)",
                  f"{'objective':28s} {i.value:12.4f} {b.value:12.4f} {b.value - i.value:+10.4f}"]
        for k in sorted(set(i.metrics) | set(b.metrics)):
            vi, vb = i.metrics.get(k, math.nan), b.metrics.get(k, math.nan)
            lines.append(f"{k:28s} {vi:12.4f} {vb:12.4f} {vb - vi:+10.4f}")
        lines.append("constraints (normalised violation, <= 0 is satisfied):")
        for k in sorted(b.violations):
            lines.append(f"  {k:30s} before {i.violations.get(k, math.nan):+8.3f}   after {b.violations[k]:+8.3f}")
        lines.append("feasible: " + ("yes" if b.feasible else "NO - no setting of the free parameters meets the "
                                                              "constraints; free more parameters or relax them"))
        if not self.improved:
            lines.append("no better feasible package was found: the start car is returned")
        lines += ["", "before:", _table(cfg), "", "after:", _table(self.best_cfg)]
        return "\n".join(lines)


def _table(cfg: dict) -> str:
    return component_table(parse_components(cfg["aero"]["components"], cfg.get("_base_dir", ".")),
                           geometry_from_cfg(cfg))


def _cost(ev: Evaluation) -> float:
    return ev.total if math.isfinite(ev.total) else 1e9


class _Search:
    """Evaluations of the search objective for one option set, cached, counted against the budget."""

    def __init__(self, parent: "_Run", cfg: dict, variables: list[DesignVar], stage: str):
        self.p, self.cfg, self.vars, self.stage = parent, cfg, variables, stage
        self.cache: dict[tuple, Evaluation] = {}
        self.points: list[tuple[dict, Evaluation]] = []

    def values(self, u) -> dict[str, float]:
        return {v.path: v.lo + float(np.clip(ui, 0.0, 1.0)) * (v.hi - v.lo) for v, ui in zip(self.vars, u)}

    def unit(self, cfg: dict | None = None) -> np.ndarray:
        cfg = cfg or self.cfg
        return np.array([np.clip((float(get_path(cfg, v.path)) - v.lo) / (v.hi - v.lo), 0.0, 1.0) for v in self.vars])

    def __call__(self, u) -> float:
        over = self.values(u)
        key = tuple(round(x, 9) for x in over.values())
        if key not in self.cache:
            self.cache[key] = self.p.evaluate(with_overrides(self.cfg, over, "공력 전체 자동 최적화"), self.stage)
            self.points.append((over, self.cache[key]))
        return _cost(self.cache[key])

    def best(self) -> tuple[dict, Evaluation]:
        return min(self.points, key=lambda c: (not c[1].feasible, _cost(c[1])))


def hooke_jeeves(f, u0, done, step: float = 0.15, min_step: float = 0.012, order=None, tol: float = 1e-6):
    """Pattern search on the unit cube. ``f(u)`` is the cost, ``done()`` says the budget is spent.

    Per variable: a step up, if that is not better a step down; one more step in a better direction;
    then the vertex of the parabola through the three points around the best one. After a pass that
    improved, the pattern move ``x + (x - x_before_the_pass)``; after one that did not, half the
    step. Stops when ``done()`` or below ``min_step``. Returns (u, f(u))."""
    x = np.asarray(u0, dtype=float).copy()
    n = len(x)
    order = list(range(n)) if order is None else list(order)
    fx = f(x)
    steps = np.full(n, float(step))
    while n and not done() and steps.max() > min_step:
        x0, f0 = x.copy(), fx
        for i in order:
            if done():
                break
            xi, cand = float(x[i]), {}

            def ev(val, i=i, cand=cand):
                trial = x.copy()
                trial[i] = val
                cand[val] = f(trial)

            up, dn = min(xi + steps[i], 1.0), max(xi - steps[i], 0.0)
            if up - xi > 1e-9:
                ev(up)
            if (up not in cand or cand[up] >= fx - tol) and xi - dn > 1e-9 and not done():
                ev(dn)
            better = [v for v in cand if cand[v] < fx - tol]
            if better and not done():
                b = min(better, key=cand.get)
                far = float(np.clip(2.0 * b - xi, 0.0, 1.0))
                if abs(far - b) > 1e-9:
                    ev(far)
            pts = sorted({**cand, xi: fx}.items())
            k = min(range(len(pts)), key=lambda j: pts[j][1])
            if 0 < k < len(pts) - 1 and not done():
                (a, fa), (b, fb), (c, fc) = pts[k - 1], pts[k], pts[k + 1]
                den = (b - a) * (fb - fc) - (b - c) * (fb - fa)
                if abs(den) > 1e-15:
                    v = b - 0.5 * ((b - a) ** 2 * (fb - fc) - (b - c) ** 2 * (fb - fa)) / den
                    if a < v < c and min(abs(v - q) for q in (a, b, c)) > 0.15 * steps[i]:
                        ev(float(v))
            if cand:
                xb = min(cand, key=cand.get)
                if cand[xb] < fx - tol:
                    x[i], fx = xb, cand[xb]
        if fx < f0 - tol:
            xp = np.clip(x + (x - x0), 0.0, 1.0)
            if np.abs(xp - x).max() > 1e-9 and not done():
                fp = f(xp)
                if fp < fx - tol:
                    x, fx = xp, fp
        else:
            steps *= 0.5
    return x, fx


class _Run:
    def __init__(self, obj: Objective, max_evals: int, progress):
        self.obj, self.max_evals, self.progress = obj, max_evals, progress
        self.n = 0
        self.history: list[tuple[int, str, float, float, bool]] = []
        self.cache: dict[str, Evaluation] = {}

    def evaluate(self, cfg: dict, stage: str) -> Evaluation:
        """Search objective of ``cfg``; only the aero section differs between the points of a run."""
        key = json.dumps(cfg["aero"], sort_keys=True, default=str)
        if key not in self.cache:
            try:
                ev = self.obj(cfg)
            except ValueError as exc:  # e.g. a roll-unstable or impossible combination
                ev = Evaluation(math.inf, {}, {"invalid": 1.0}, 1e6)
                self.say(f"{stage}: invalid point ({str(exc).splitlines()[0]})")
            self.cache[key] = ev
            self.count(stage, ev)
        return self.cache[key]

    @property
    def left(self) -> int:
        return max(self.max_evals - self.n, 0)

    def say(self, msg: str) -> None:
        if self.progress:
            self.progress(msg)

    def count(self, stage: str, ev: Evaluation) -> None:
        self.n += 1
        self.history.append((self.n, stage, ev.value, _cost(ev), ev.feasible))
        if math.isfinite(ev.value):
            self.say(f"{stage} {self.n}/{self.max_evals}: {ev.value:.3f}{'' if ev.feasible else ' (violates)'}")


def auto_aero(cfg: dict, objective_spec: dict, base_dir: str | Path = ".", constraints: dict | None = None,
              fixed: list[str] | None = None, bounds: dict | None = None, options: bool = True,
              max_evals: int = 300, max_vars: int = 24, option_evals: int = 10, screen_tol: float = 0.02,
              opts: SolverOptions | None = None, n_refine: int = 3, progress=None) -> AutoAeroResult:
    """Optimise every aero parameter / option of ``cfg`` that is not kept (``fixed`` patterns,
    ``aero.design.fixed``). See the module text for the stages; ``max_evals`` is the search budget."""
    if not ((cfg.get("aero") or {}).get("components")):
        raise ValueError("the vehicle has no aero.components (front / side / rear devices) to optimise")
    t0 = time.time()
    constraints = dict(constraints or {})
    obj = Objective(objective_spec, base_dir, constraints, opts or SolverOptions.coarse())
    obj_fine = Objective(objective_spec, base_dir, constraints)
    run = _Run(obj, max(int(max_evals), 1), progress)
    space0 = design_space(cfg, fixed, bounds, options)
    if not space0.free and not space0.free_options:
        raise ValueError("every aero design parameter is kept - nothing to optimise (see --fix / aero.design.fixed)")
    run.say(f"design space: {len(space0.free)} free variables, {len(space0.free_options)} free options, "
            f"{len(space0.variables) - len(space0.free)} kept")

    start = _Search(run, cfg, [], "start")
    start(np.zeros(0))
    initial_search = start.points[0][1]
    work, trials, trimmed = copy.deepcopy(cfg), [], None

    # ---- 1. options
    if space0.free_options:
        budget = min(int(0.4 * run.max_evals), run.left)
        flipped: set[str] = set()
        for _round in range(3):
            space = design_space(work, fixed, bounds, options)
            todo = [o for o in space.free_options if o.name not in flipped]
            per = max(1, min(option_evals, budget // (len(todo) + 1))) if todo else 0
            if not todo or budget < len(todo) + 1:
                break

            def trim(c):
                vs = [v for v in design_space(c, fixed, bounds, options).free if v.name.endswith(".aoa_deg")]
                s, n0 = _Search(run, c, vs, "options"), run.n
                hooke_jeeves(s, s.unit(), lambda: run.n - n0 >= per or run.left <= 0, step=0.2, min_step=0.04)
                return s.best()

            before = run.n
            cur_x, cur_ev = trim(work)
            trimmed = trimmed or cur_ev
            work = with_overrides(work, cur_x, "옵션 탐색 기준 (받음각 재조정)")
            best = None
            for o in todo:
                try:
                    alt = with_overrides(work, {o.path: not o.value}, f"옵션 {o.name} 전환")
                    x, ev = trim(alt)
                except ValueError as exc:
                    run.say(f"options: {o.name} -> {'off' if o.value else 'on'} is not possible "
                            f"({str(exc).splitlines()[0]})")
                    continue
                t = OptionTrial(o.name, not o.value, ev, x)
                trials.append(t)
                run.say(f"option {o.name} -> {'on' if t.to else 'off'}: {ev.value:.3f} "
                        f"({ev.value - cur_ev.value:+.3f}){'' if ev.feasible else ' (violates)'}")
                if (not ev.feasible, _cost(ev)) < (not cur_ev.feasible, _cost(cur_ev) - 1e-6) and (
                        best is None or (not ev.feasible, _cost(ev)) < (not best.evaluation.feasible,
                                                                        _cost(best.evaluation))):
                    best = t
            budget -= run.n - before
            if best is None:
                break
            best.accepted = True
            flipped.add(best.name)
            work = with_overrides(work, {"aero.components." + best.name: best.to, **best.trim}, "옵션 적용")
            run.say(f"option taken: {best.name} -> {'on' if best.to else 'off'}")

    # ---- 2. screen (every free variable once, also when that takes more than the budget left)
    space = design_space(work, fixed, bounds, options)
    free = space.free
    base_s = _Search(run, work, free, "screen")
    u0 = base_s.unit()
    f0 = base_s(u0)
    screened: list[Screened] = []
    if run.left < len(free):
        run.say(f"budget: screening the {len(free)} free variables takes more than the {run.left} evaluations left "
                "- raise max_evals to also search them")
    for i, v in enumerate(free):
        u = u0.copy()
        u[i] = u0[i] + 0.1 if u0[i] + 0.1 <= 1.0 else u0[i] - 0.1
        screened.append(Screened(v, (u[i] - u0[i]) * (v.hi - v.lo), base_s(u) - f0))
    top = max((abs(s.delta) for s in screened), default=0.0)
    ranked = sorted(screened, key=lambda s: -abs(s.delta))
    for s in ranked[:max_vars]:
        s.used = top > 0 and abs(s.delta) >= screen_tol * top
    used = [s for s in ranked if s.used]

    # ---- 3. search
    main = _Search(run, work, [s.var for s in used], "search")
    if used and run.left > 0:
        hooke_jeeves(main, main.unit(), lambda: run.left <= 0)

    # ---- 4. fine check
    cands: list[dict] = []
    pool = sorted(main.points + base_s.points, key=lambda c: (not c[1].feasible, _cost(c[1])))
    for over, ev in pool:
        if math.isfinite(_cost(ev)) and over not in cands:
            cands.append(over)
        if len(cands) >= max(n_refine, 1):
            break
    best_search = pool[0][1] if pool else initial_search
    initial_eval = obj_fine(copy.deepcopy(cfg))
    finals = [(copy.deepcopy(cfg), initial_eval, None)]
    for over in cands:
        c = with_overrides(work, over, "공력 전체 자동 최적화 - 정밀 재평가")
        try:
            finals.append((c, obj_fine(c), over))
        except ValueError:
            continue
        run.say(f"fine check: {finals[-1][1].value:.3f}{'' if finals[-1][1].feasible else ' (violates)'}")
    best_cfg, best_eval, _ = min(finals, key=lambda c: (not c[1].feasible, c[1].value if c[1].feasible else c[1].total))
    improved = best_cfg is not finals[0][0]

    res = AutoAeroResult(obj_fine.label, obj_fine.unit, space0, space, initial_eval, best_eval, initial_search,
                         best_search, best_cfg, trials=trials, trimmed=trimmed, screen=screened, history=run.history,
                         evals=run.n, elapsed_s=time.time() - t0, improved=improved)
    if improved:
        for o in space0.options:
            now = get_path(best_cfg, o.path, True)
            now = bool(now) if not isinstance(now, str) else now.strip().lower() not in ("false", "0", "off", "no")
            if now != o.value:
                res.option_changes[o.name] = (o.value, now)
        for v in space.variables:
            a, b = get_path(cfg, v.path, None), float(get_path(best_cfg, v.path))
            if isinstance(a, (int, float)) and abs(b - float(a)) > 1e-9 * max(1.0, abs(b)):
                res.changes[v.path] = (float(a), b)
    return res
