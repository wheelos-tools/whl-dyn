# Conventions

## When

Read before adding plans, signals, analysis outputs, or CLI options.

## Facts

- Python packaging and the `whl-dyn` command are configured in `pyproject.toml`.
- Plans use semantic signal names; vehicle protobuf paths belong in YAML signal
  mappings consumed by `whl_dyn/collection/lateral.py`.
- Active open-loop steering requires explicit execution arming and safety
  checks.
- Processing results are JSON-friendly dictionaries and persisted reports live
  beside collected runs.
- Tests use `pytest` and live under `tests/`.

## Sources

- `pyproject.toml`
- `whl_dyn/cli.py`
- `whl_dyn/collection/lateral.py`
- `whl_dyn/processing/`
- `tests/`
