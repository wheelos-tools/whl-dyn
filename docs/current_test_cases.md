# Responsibility-Based Test Inventory

The core responsibilities are throttle/brake calibration and chassis-dynamics
measurement. Controller implementation/tuning, ESP/ESC validation and simulator
physics are not prerequisites or responsibilities of these core workflows.
Closed-loop tracking is a separate integration extension.

The checks below verify software logic with deterministic data and bounded
runtime doubles. They are not a vehicle acceptance certificate. Vehicle-specific
units, limits, operating conditions and independent repeat runs remain required.
See the [test program assessment](test_program_assessment.md) for the rationale,
industry comparison and proposed priorities; recommendations there are not
claims of implemented functionality.

| ID | Test | Plan / execution | Analysis |
| --- | --- | --- | --- |
| LON-CAL | Constant throttle / constant brake speed sweeps | Validated YAML, speed triggers | Net-acceleration map; single-actuator and coast values preserved |
| LON-CAL-EXPORT | Control calibration export | CSV diagnostics / protobuf text | Finite-domain entries only, warning for gaps, valid zero acceleration retained |
| LON-CAL-QUALITY | Response threshold / repeatability | Recorded coast / independent repetitions | Threshold unavailable without measured coast; in-sample residual is not independent accuracy |
| LON-DYN-STEP | Isolated positive/negative step | Dynamic YAML | Known-model gain, rise/settling time and FOPDT regressions |
| LON-DYN-FRF | Single sine, chirp, sweep, multi-sine, PRBS | Dynamic YAML | Gain/phase/coherence; bandwidth only with coherent plateau and cutoff coverage |
| LON-DYN-RAW | Ramp, pulse, triangle, hysteresis | Profile generation and collection | No dedicated quantitative analyzer; explicitly rejected by `analyze_dynamic`, never treated as a step |
| LAT-OL-STEP | Positive/negative steering step | `plan-open-loop`, `collect-lateral` | Command-to-feedback and command-to-motion step metrics; excitation and response required |
| LAT-OL-RAMP | Slow steering ramp | `plan-open-loop`, `collect-lateral` | Observed feedback range/rate; calibrated deadzone unavailable without noise/units configuration |
| LAT-OL-FRF | Steering chirp/sweep/PRBS | `plan-lateral`, `collect-lateral` | Actual-steering-to-yaw/lateral-acceleration Bode reports |
| LAT-SS | Fixed-steering steady state | `plan-circles`, `collect-lateral` | Steady-window statistics and measured curvature; Ku requires road-wheel radians, measured curvature, positive wheelbase and distinct operating points |
| LAT-CL (extension) | Closed-loop straight--Clothoid--arc tracking | `plan-closed-loop`, `run-closed-loop` | Reference/message/runtime helpers covered; live controller validation is separate |

## Cross-cutting regression checks

- Generated and hand-edited longitudinal plans share preflight: finite positive
  duration/rate/timeouts, valid triggers, Drive gear, mutually exclusive bounded
  throttle/brake and non-clipping excitation.
- Dynamic brake/both tests require initial-speed preparation. Preparation is not
  part of the analysis log; stopping before the profile ends invalidates the run.
- Longitudinal collection aborts on stale/misaligned feedback, mode/gear departure,
  configured overspeed and timeout. Hold triggers must remain continuously true;
  a transition immediately publishes the new command. Final braking is bounded
  and requires stop confirmation.
- Longitudinal runs persist `.status.json`; aborted runs with this sidecar cannot
  enter calibration. Older CSVs without it remain legacy data, not proof of a
  completed execution.
- Phase 1 checks the recorded plan against the independently supplied expected
  plan, verifies sampled excitation and observable feedback/motion, and rejects
  missing cases, absent quality fields and invalid samples.
- Phase 1 `PASS` means execution and observable response checks passed.
  `vehicle_acceptance: NOT_EVALUATED` is explicit; no unconfigured vehicle
  performance requirement is silently considered satisfied.
- Frequency analysis rejects nonuniform timestamps/rate mismatch. Phase delay is
  not transport delay; unavailable bandwidth/phase metrics display as `N/A`.

## Regression sources

- `tests/test_data_core.py`: net acceleration, valid-domain export, aborted logs,
  command-segment isolation, withheld-speed interpolation and coast reference.
- `tests/test_dynamics.py`: all profile execution forms, longitudinal preflight,
  preparation, state transitions, freshness, stopping, signed step and FRF truth.
- `tests/test_lateral_dynamics.py`: complete Phase 1 reports and negative controls.
- `tests/test_handling.py`: geometry, mirrored excitation, steady selection,
  understeer identifiability and closed-loop reference helpers.
- Message/runtime/UI tests cover integration contracts and unavailable outputs.

## Boundaries

- `LAT-SS` is fixed steering, not fixed radius.
- `LAT-CL` ends at the arc endpoint and uses a direct safe-stop command.
- Sideslip, wheel slip, tire utilization, and vehicle-specific steering
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
