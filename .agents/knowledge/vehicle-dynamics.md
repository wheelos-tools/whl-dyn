# Vehicle-Dynamics Tests

## When

Read before changing a lateral test plan, collector, metric, reference
trajectory, or test acceptance rule.

## Facts

- Phase 1 covers open-loop steering step/ramp and steering frequency-response
  experiments.
- Phase 2 is fixed-steering steady-state testing with speed, direction, and
  repeat matrices; it is not a fixed-radius controller.
- Phase 3 publishes an immutable straight-to-clothoid-to-arc reference and
  stops at the arc endpoint.
- Phase 1, Phase 2, and Phase 3 collection rows retain a fixed sample time,
  source timestamps, source ages, alignment skew, and a `time_aligned` flag.
- Vehicle-specific signals, limit thresholds, and unit conversions must remain
  outside the generic plan and processing modules.
- Assessment conclusions and missing acceptance criteria belong in the
  vehicle-dynamics assessment, not in this index.

## Sources

- `plans/open_loop_identification.yaml`
- `plans/lateral_chirp.yaml`
- `plans/lateral_prbs.yaml`
- `plans/closed_loop_curve.yaml`
- `whl_dyn/planning/handling.py`
- `whl_dyn/planning/vehicle_dynamics.py`
- `whl_dyn/collection/lateral.py`
- `whl_dyn/collection/closed_loop.py`
- `whl_dyn/processing/handling.py`
- `whl_dyn/trajectory/continuous.py`
- `whl_dyn/trajectory/apollo.py`
- `tests/test_handling.py`
- `tests/test_lateral_dynamics.py`
- `docs/handling_test_phases.md`
- `docs/lateral_vehicle_dynamics.md`
- `docs/open_loop_lateral_identification_assessment.md`
