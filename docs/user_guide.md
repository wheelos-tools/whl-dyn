# User Guide

## Install

The package metadata and dependencies are defined in
[`pyproject.toml`](../pyproject.toml).

```bash
python3 -m pip install -e .
```

For Apollo/CyberRT collection, run the command in the vehicle integration
environment where the required protobuf modules are available.

## Entry points

```bash
# Streamlit workbench
streamlit run whl_dyn/ui/app.py

# CLI help
python3 -m whl_dyn.cli --help

# Unit tests
pytest -q
```

## Workflows

| Workflow | Commands and reference |
| --- | --- |
| Longitudinal calibration and UI | `whl_dyn/ui/app.py`, [`vehicle_dynamics.md`](vehicle_dynamics.md) |
| Lateral open-loop identification | `plan-open-loop`, `plan-lateral`, `collect-lateral`, [`lateral_vehicle_dynamics.md`](lateral_vehicle_dynamics.md) |
| Fixed-steering steady-state tests | `plan-circles`, `analyze-steady-state`, [`handling_test_phases.md`](handling_test_phases.md) |
| Closed-loop curve tracking | `plan-closed-loop`, `run-closed-loop`, [`handling_test_phases.md`](handling_test_phases.md) |

Active vehicle commands require the documented signal mapping, safety procedure,
and explicit execution arming. Start all new mappings in record-only mode.

## Outputs

Lateral collectors create a unique run directory containing `metadata.yaml`,
`samples.csv`, and `status.json`. Analysis commands write reports under that
run's `analysis/` directory.

## Troubleshooting

- Run `pytest -q` before changing an integration environment.
- Verify semantic signal names, units, signs, and source timestamps in a
  record-only run before active collection.
- See `.agents/knowledge/troubleshooting.md` for source references and
  integration constraints.
