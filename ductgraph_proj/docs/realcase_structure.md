# Real-Case Structure (Current)

## Main execution path
- `tools/run_realcase_commission_scale.py`: operator-facing runner
- `ductgraph/commissioning_scale.py`: full-load commissioning + partial-load scaling logic
- `cases/real_case.py`: compatibility facade for real-case setup

## Real-case case-definition split
- `cases/real_case_constants.py`: IDs, fixed pressures, setpoints, physical constants, local-loss tables
- `cases/real_case_defaults.py`: operation defaults (damper model/gamma, fan curve defaults, scaling cases, tolerances)
- `cases/real_case_network.py`: resistance build-up, damper calibration, and `make_net()`
- `cases/real_case.py`: stable API/re-exports (`Q_DESIGN`, `CAV_EDGES`, `make_net`, etc.)

## Reporting utilities
- `ductgraph/report_commission_scale.py`: summary/terminal table formatting and reason classification

## Archived comparison work
- `archive/_cmp_d34/`
- `archive/_cmp_d4/`

These are kept for historical comparison and are not part of the runtime path.


## Archived legacy modules
- `archive/legacy_unused/ductgraph/cli_cav.py`
- `archive/legacy_unused/ductgraph/cli_commission_scale.py`
- `archive/legacy_unused/ductgraph/physics/`

These modules are not imported by `tools/run_realcase_commission_scale.py` and were moved out of the main runtime path.
