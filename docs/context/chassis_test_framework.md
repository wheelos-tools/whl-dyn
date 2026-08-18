# Chassis Test Framework

## Scope

The repository separates test planning, CyberRT collection, offline processing,
reference-trajectory generation, and UI presentation. This document is a
framework index; implemented behavior is defined by the linked sources.

```text
planning -> collection / trajectory execution -> timestamped run -> processing
```

## Test families

| Family | Current reference |
| --- | --- |
| Longitudinal calibration and actuator dynamics | `whl_dyn/planning/generator.py`, `whl_dyn/processing/dynamics.py` |
| Lateral open-loop identification | `whl_dyn/planning/handling.py`, `whl_dyn/planning/vehicle_dynamics.py`, `whl_dyn/collection/lateral.py` |
| Fixed-steering steady state | `whl_dyn/planning/handling.py`, `whl_dyn/processing/handling.py` |
| Closed-loop tracking | `whl_dyn/trajectory/`, `whl_dyn/collection/closed_loop.py` |

## Operating rules

- Vehicle-specific protobuf fields belong in vehicle-local signal mappings.
- Active open-loop collection requires explicit arming and verified feedback.
- Closed-loop trajectory publication must be the only publisher on
  `/apollo/planning`.
- Reports are valid only for the signals, units, timestamps, and safety limits
  verified for that vehicle.

## Related documents

- [`../lateral_vehicle_dynamics.md`](../lateral_vehicle_dynamics.md)
- [`../handling_test_phases.md`](../handling_test_phases.md)
- [`../open_loop_lateral_identification_assessment.md`](../open_loop_lateral_identification_assessment.md)
