# Testing

## When

Read before validating a code change.

## Rules

- Run the narrowest test file covering the changed behavior first.
- Use `pytest -q` before completing cross-module changes.
- Exercise new CLI argument parsing with `python3 -m whl_dyn.cli <command> --help`.
- Do not install dependencies unless a selected validation command requires them.
- Treat CyberRT/Apollo collection as an integration environment; unit tests must
  not require a live vehicle runtime.

## Sources

- `pyproject.toml`
- `tests/`
- `whl_dyn/cli.py`
