import copy
from pathlib import Path

import pytest

from aerolap.config import load_vehicle_config, vehicle_config_from_dict

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"

SIMPLE = {
    "name": "simple test car",
    "environment": {"temperature_C": 15.0, "pressure_kPa": 101.325, "relative_humidity": 0.0},
    "mass": {"total_kg": 250.0, "weight_dist_front": 0.45, "cg_height_mm": 300.0},
    "geometry": {"wheelbase_mm": 1600.0, "track_front_mm": 1200.0, "track_rear_mm": 1200.0},
    "tires": {"radius_mm": 200.0, "mu_x": 1.4, "mu_y": 1.5, "ref_load_N": 600.0, "load_sensitivity": 0.0,
              "camber_sensitivity_per_deg2": 0.0, "rolling_resistance": 0.0},
    "suspension": {
        "front": {"spring_rate_N_per_mm": 30.0, "static_ride_height_mm": 30.0, "roll_center_mm": 30.0,
                  "arb_rate_Nm_per_deg": 100.0},
        "rear": {"spring_rate_N_per_mm": 30.0, "static_ride_height_mm": 40.0, "roll_center_mm": 50.0,
                 "arb_rate_Nm_per_deg": 50.0},
    },
    "aero": {"type": "constant", "cla": 3.0, "cda": 1.2, "cop_front": 0.45, "drag_height_mm": 0.0},
    "powertrain": {"type": "ev", "drive": "rwd", "differential": "lsd", "efficiency": 1.0,
                   "ev": {"motor_torque_curve": [[0, 200], [20000, 200]], "max_rpm": 20000,
                          "reduction_ratio": 4.0, "power_limit_kW": 60}},
}


@pytest.fixture
def simple_raw():
    return copy.deepcopy(SIMPLE)


@pytest.fixture
def simple_cfg(simple_raw):
    return vehicle_config_from_dict(simple_raw)


@pytest.fixture
def ev_cfg():
    return load_vehicle_config(DATA / "vehicles" / "fsae_ev_aero.yaml")
