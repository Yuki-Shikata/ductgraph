# Test Layout

## 1. Active tests
- Path: tests/nodehead/
- Scope: current runtime path (commissioning_scale, real_case, solver)
- Command (repo root): python -m pytest -q ductgraph_proj/tests/nodehead
- Command (ductgraph_proj/): python -m pytest -q tests/nodehead

## 2. Legacy tests
- Path: tests/legacy_nodehead/
- Scope: historical reference for old two-stage / maxload-seed flow
- Command (ductgraph_proj/): python -m pytest -q tests/legacy_nodehead

## 3. Quick verify
- ductgraph_proj/Makefile に verify ターゲットあり
- 実行場所: ductgraph_proj/
