# cvtrack project Makefile
#
# Common developer-facing tasks.  Designed for direct use on Linux/macOS.
# On Windows, use the equivalent `pwsh` commands or `poetry run <task>`.

PYTHON ?= python3
SRC := $(CURDIR)/src
PYTHONPATH := $(CURDIR)/.local_lib:$(SRC):$(PYTHONPATH)

export PYTHONPATH

.PHONY: install lint typecheck test test-fast test-slow run-drone \
        docker-build clean help

help:  ## Show this help (use: make help)
	@awk 'BEGIN {FS = ":.*## "} /^[a-zA-Z_-]+:.*## / {printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

install:  ## Install cvtrack (editable) with deps
	$(PYTHON) -m pip install --upgrade pip setuptools wheel
	$(PYTHON) -m pip install -e .

install-fast:  ## Install cvtrack (editable) WITHOUT deps (for restricted environments)
	$(PYTHON) -m pip install -e . --no-deps
	@echo "deps NOT installed.  Manually install requirements.txt before running."

lint:  ## Run ruff
	$(PYTHON) -m ruff check src tests scripts eval

typecheck:  ## Run mypy (best effort; project is partially typed)
	$(PYTHON) -m mypy src || true

test: test-fast ## Alias for fast tests (no network / no model downloads)

test-fast:  ## Run pytest, skipping slow tests
	$(PYTHON) -m pytest -p no:anyio -m "not slow" tests/ -W ignore::DeprecationWarning

test-slow:  ## Run pytest including the slow suite
	$(PYTHON) -m pytest -p no:anyio tests/ -W ignore::DeprecationWarning

run-drone:  ## Reproduce the v4 drone preset
	$(PYTHON) -m cvtrack --config drone --source pexels_aerial_2034115.mp4 \
	    --out-dir /tmp/cvtrack_drone

run-synthetic-eval:  ## Self-test the eval harness (no MOT17 needed)
	$(PYTHON) eval/mot17_mini/run_eval.py --synthetic

docker-build:  ## Build the cvtrack Docker image (tag=latest)
	docker build -t cvtrack:latest .

clean:  ## Remove caches + build artifacts
	rm -rf build dist .pytest_cache .mypy_cache .ruff_cache __pycache__
	find . -name '__pycache__' -type d -exec rm -rf {} +
	find . -name '*.pyc' -delete
