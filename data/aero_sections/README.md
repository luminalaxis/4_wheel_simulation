# 2-D wing sections (AERO_AUTOMATION format)

| file | content |
|---|---|
| `synthetic_front_summary.csv` | **SYNTHETIC** (made-up numbers, not CFD) front-wing sections in ground effect: 24 two-element geometries x clearance 40-100 mm x aoa -4..16 x 40-100 km/h |
| `synthetic_rear_summary.csv` | **SYNTHETIC** rear-wing sections in free stream (no `ground_clearance`): 2- and 3-element geometries |
| `aero_automation_rear_wing.toml` | proposal for an AERO_AUTOMATION free-stream project for the rear wing (sized for the 2026 car) |

The CSV columns and signs are those of AERO_AUTOMATION `runs/summary.csv` (`cl` = downforce coefficient, positive
down, main-chord based). Regenerate the synthetic files with `python scripts/generate_synthetic_sections.py`.
When Fluent has run, point `section_data:` of a wing (or `aerolap rank-sections`) to the real `runs/summary.csv`.
