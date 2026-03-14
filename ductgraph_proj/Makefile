.PHONY: verify

PYTHON ?= ../.venv/bin/python
ifeq ("$(wildcard $(PYTHON))","")
PYTHON := python3
endif

verify:
	$(PYTHON) -m compileall -q ductgraph cases tools tests/nodehead
	$(PYTHON) -m pytest -q tests/nodehead
