"""Lap-time solvers: quasi-steady-state (:class:`QSSSolver`) and time-domain
(:class:`TransientSolver`), both returning :class:`~aerolap.solvers.base.LapResult`."""

from .base import LapResult, Solver  # noqa: F401
from .ggv import GGV, SolverOptions, ay_max_curve, compute_ggv  # noqa: F401
from .qss import QSSSolver  # noqa: F401
from .transient import RideModel, TransientOptions, TransientSolver  # noqa: F401
