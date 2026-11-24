# User Guide

## Install

Python 3.9 or newer is required. The package metadata and dependencies are
defined in [`pyproject.toml`](../pyproject.toml).

```bash
python3 -m pip install -e .
```

Installation includes the published `wheelos-msgs` wheel, which supplies the
generated planning and other WheelOS protobuf modules. It requires
`protobuf>=5.27.1,<6`; do not bypass dependency resolution with `--no-deps`.
Use a separate compatible Python environment if other tools require protobuf
6 or newer. For Apollo/CyberRT collection, CyberRT must also be available in
that environment; it is provided by the vehicle integration environment, not
this package.

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
