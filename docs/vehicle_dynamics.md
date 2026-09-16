# Longitudinal Actuator Workbench

## Scope

The Streamlit workbench supports the repository's longitudinal calibration and
dynamic actuator workflows. Lateral vehicle-dynamics tests use the dedicated
CLI plans and collectors documented separately.

## Workflow

```text
generate plan -> collect CSV -> analyze -> export
```

The UI entry point is `whl_dyn/ui/app.py`. Dynamic profile generation and
analysis behavior are implemented in:

- `whl_dyn/planning/generator.py`
- `whl_dyn/collection/collector.py`
- `whl_dyn/processing/dynamics.py`
- `whl_dyn/ui/app.py`

## Boundaries

- Dynamic actuator profiles and their available fields are defined by the plan
  and collector, not by this document.
- Lateral steering, steady-state handling, and closed-loop tracking are
  documented in [`handling_test_phases.md`](handling_test_phases.md) and
  [`lateral_vehicle_dynamics.md`](lateral_vehicle_dynamics.md).
- Use the test inventory for available workflows and the assessment document
  for known limitations.
