# Test Layout

## Active tests (current runtime)
- Path: `tests/nodehead/`
- Scope: solver + commissioning_scale + realcase runtime path
- Command (from repo root): `python -m pytest -q ductgraph_proj/tests/nodehead`
- Command (from `ductgraph_proj/`): `python -m pytest -q tests/nodehead`

## Legacy tests (reference/compat)
- Path: `tests/legacy_nodehead/`
- Scope: legacy two-stage commissioning flow and seed(maxload)-based tuning path
- Command: `python -m pytest -q tests/legacy_nodehead`

## Quick verify
- Command: `make verify`
- Location: run inside `ductgraph_proj/`
