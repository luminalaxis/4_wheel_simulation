"""aerolap: quasi-steady-state four-wheel lap simulator with suspension-coupled aerodynamics.

Typical use::

    from aerolap import load_vehicle_config, VehicleModel, Track, QSSSolver

    cfg = load_vehicle_config("data/vehicles/fsae_ev_aero.yaml")
    car = VehicleModel(cfg)
    track = Track.from_file("data/tracks/fsae_endurance_example.csv")
    result = QSSSolver().solve(car, track, start="flying")
    print(result.lap_time)
"""

__version__ = "0.1.0"

from .config import load_vehicle_config, get_path, set_path, with_overrides  # noqa: F401
from .features import ModelFeatures, features_for_level, LEVELS  # noqa: F401
from .vehicle import VehicleModel  # noqa: F401
from .track import Track  # noqa: F401
from .solvers.ggv import SolverOptions  # noqa: F401
from .solvers.qss import QSSSolver  # noqa: F401
from .solvers.transient import TransientOptions, TransientSolver  # noqa: F401
