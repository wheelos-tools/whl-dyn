# Implemented Test Inventory

This is an index of test workflows that have a plan, collector/runner, and
processing path in the repository. It does not claim vehicle-specific signals
or safety limits are configured; those remain integration responsibilities.

| ID | Test | Plan / execution | Analysis |
| --- | --- | --- | --- |
| LON-CAL | Longitudinal throttle/brake calibration | Streamlit workflow | Calibration exports and diagnostics |
| LON-DYN | Longitudinal actuator Step/frequency profiles | Dynamic YAML cases | Step and frequency-response helpers |
| LAT-OL-STEP | Open-loop steering Step and slow Ramp | `plan-open-loop`, `collect-lateral` | Generic Step metrics |
| LAT-OL-FRF | Steering Chirp/Sweep/PRBS | `plan-lateral`, `collect-lateral` | Yaw-rate and lateral-acceleration Bode reports |
| LAT-SS | Fixed-steering steady state | `plan-circles`, `collect-lateral` | `analyze-steady-state`, understeer-fit helpers |
| LAT-CL | Closed-loop straight--Clothoid--arc tracking | `plan-closed-loop`, `run-closed-loop` | Tracking metrics |

## Boundaries

- `LAT-SS` is fixed steering, not fixed radius.
- `LAT-CL` ends at the arc endpoint and uses a direct safe-stop command.
- Sideslip, wheel slip, ESC, tire utilization, and vehicle-specific steering
  signals require mapped vehicle data; an unavailable input is not a passing
  stability result.
- Capability gaps and acceptance criteria are maintained in
  [`open_loop_lateral_identification_assessment.md`](open_loop_lateral_identification_assessment.md).

## Sources

- `plans/`
- `whl_dyn/planning/`
- `whl_dyn/collection/`
- `whl_dyn/processing/`
- `tests/`
