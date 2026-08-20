# whl-dyn

`whl-dyn` is a Python toolkit for vehicle actuator calibration and lateral
vehicle-dynamics tests. It provides plan generation, CyberRT collection,
offline analysis, and a Streamlit workbench.

## Install and run

```bash
python3 -m pip install -e .

streamlit run whl_dyn/ui/app.py
```

The CLI entry point is `whl-dyn`; use `python3 -m whl_dyn.cli --help` when
developing from a checkout.

## Documentation

| Topic | Document |
| --- | --- |
| Installation and commands | [`docs/user_guide.md`](docs/user_guide.md) |
| Lateral open-loop frequency tests | [`docs/lateral_vehicle_dynamics.md`](docs/lateral_vehicle_dynamics.md) |
| Phase 1--3 handling workflow | [`docs/handling_test_phases.md`](docs/handling_test_phases.md) |
| Implemented test inventory | [`docs/current_test_cases.md`](docs/current_test_cases.md) |
| Capability assessment and gaps | [`docs/open_loop_lateral_identification_assessment.md`](docs/open_loop_lateral_identification_assessment.md) |
| Agent knowledge and task procedures | [`AGENTS.md`](AGENTS.md) |

## Source map

- `whl_dyn/planning/`: test plans and feasibility validation
- `whl_dyn/collection/`: CyberRT collection and run storage
- `whl_dyn/processing/`: offline metrics and reports
- `whl_dyn/trajectory/`: reference geometry and Apollo trajectory publication
- `whl_dyn/ui/`: Streamlit workbench
- `tests/`: executable behavior specifications
