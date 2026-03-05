# Project scope (ductsim)

## Allowed scope
- Work ONLY inside `ductgraph_proj/`.
- Do NOT read or modify `ductnet_proj/` or `_quarantine/` or `_backup/`.

## Workflow
- Before proposing edits: summarize the plan in bullets.
- Make changes in small commits.
- After edits: run `python -m pytest -q ductgraph_proj/tests/nodehead`.

## Safety
- Never run destructive commands (rm -rf, git reset --hard, etc.)
- If unsure, ask for approval.
