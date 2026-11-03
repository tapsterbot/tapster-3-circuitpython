# Convenience targets. Everything runs inside .venv, created on first use.
#
#   make test                     unit and integration tests, no hardware
#   make test-hw                  end-to-end test on a connected robot (moves it)
#   make test-hw PORT=/dev/cu.usbmodem2101
#   make test ARGS="-k pbm -v"    pass extra arguments to pytest

VENV ?= .venv
PYTHON := $(VENV)/bin/python
STAMP := $(VENV)/.installed-dev
ARGS ?=
PORT ?=

.DEFAULT_GOAL := help

.PHONY: help install test test-unit test-integration test-hw deploy clean

help: ## Show available targets
	@grep -E '^[a-zA-Z_-]+:.*## ' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*## "}; {printf "  %-17s %s\n", $$1, $$2}'

$(STAMP): requirements-dev.txt
	test -x $(PYTHON) || python3 -m venv $(VENV)
	$(PYTHON) -m pip install -q -r requirements-dev.txt
	touch $@

install: $(STAMP) ## Create .venv and install pyserial, pytest and ampy

test: $(STAMP) ## Run unit and integration tests (no hardware needed)
	$(PYTHON) -m pytest tests/unit tests/integration $(ARGS)

test-unit: $(STAMP) ## Run unit tests only
	$(PYTHON) -m pytest tests/unit $(ARGS)

test-integration: $(STAMP) ## Run firmware + client tests over fake serial
	$(PYTHON) -m pytest tests/integration $(ARGS)

test-hw: $(STAMP) ## Run the end-to-end test on a connected robot (moves it)
	TAPSTER_PORT=$(PORT) $(PYTHON) -m pytest --hardware tests/hardware $(ARGS)

deploy: ## Copy firmware/ to the board (PORT= for a serial port or mount)
	./deploy.sh $(PORT)

clean: ## Remove test and Python caches
	rm -rf .pytest_cache
	find . -path ./$(VENV) -prune -o -name __pycache__ -type d -exec rm -rf {} +
