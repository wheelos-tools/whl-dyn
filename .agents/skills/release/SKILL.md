# Release

## When

Read before changing a version or publishing a release artifact.

## Rules

- Update the project version only in `pyproject.toml`.
- Run the full test suite before packaging.
- Verify the `whl-dyn` entry point still resolves to `whl_dyn.cli:main`.
- Document user-visible CLI, plan, or output-schema changes.
- Do not publish vehicle-specific configurations or collected run data.

## Sources

- `pyproject.toml`
- `whl_dyn/cli.py`
- `README.md`
- `docs/`
