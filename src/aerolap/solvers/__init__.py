"""Lap-time solvers. Only the quasi-steady-state solver is implemented; see docs/architecture.md
for how a transient solver plugs into the same :class:`~aerolap.solvers.base.Solver` interface."""

from .base import LapResult, Solver  # noqa: F401
from .ggv import GGV, SolverOptions, ay_max_curve, compute_ggv  # noqa: F401
from .qss import QSSSolver  # noqa: F401
