# Architecture

## When

Read when changing data flow, module boundaries, or vehicle-dynamics behavior.

## Facts

- Plans are generated in `whl_dyn/planning/` and executed by collectors in
  `whl_dyn/collection/`.
- Processing modules operate on persisted CSV data rather than live CyberRT
  messages.
- Lateral open-loop collection uses vehicle-local signal mappings; generic code
  uses semantic signal names.
- Closed-loop reference geometry and Apollo publication are separated between
  `whl_dyn/trajectory/` and `whl_dyn/collection/closed_loop.py`.
- UI and CLI are entry surfaces, not the source of planning or analysis logic.

## Sources

- `whl_dyn/planning/`
- `whl_dyn/collection/`
- `whl_dyn/processing/`
- `whl_dyn/trajectory/`
- `whl_dyn/ui/`
- `docs/handling_test_phases.md`
- `docs/lateral_vehicle_dynamics.md`
- `.agents/knowledge/vehicle-dynamics.md`
