# Troubleshooting

## When

Read when local commands, vehicle integration, or collected data are failing.

## Facts

- Install package dependencies from `pyproject.toml` with editable installation.
- CyberRT and Apollo protobuf imports occur only in collection/runtime paths.
- Active lateral tests require a valid signal mapping and actual steering
  feedback; record-only collection is the first integration check.
- Frequency and steady-state reports require the expected timestamp and signal
  columns in a run's `samples.csv`.

## Sources

- `pyproject.toml`
- `whl_dyn/collection/lateral.py`
- `whl_dyn/collection/closed_loop.py`
- `whl_dyn/processing/lateral_dynamics.py`
- `whl_dyn/processing/handling.py`
- `docs/lateral_vehicle_dynamics.md`
