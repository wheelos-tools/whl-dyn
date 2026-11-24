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

## LON-CAL plan contract

Generated longitudinal calibration cases contain a forward gear selection,
`gear_location: 1` (`GEAR_DRIVE`), a command-update rate, declared input/output
signals, and timed steps. Each step commands throttle and brake percentages
with at most one actuator nonzero; speed thresholds trigger transitions but
are not closed-loop speed setpoints. The collector defaults older plans that
omit `gear_location` to Drive.

The plan can command:

- `throttle` and `brake` percentages in the range 0–100;
- Drive gear (fixed for generated LON-CAL plans);
- command-update rate, speed-trigger thresholds, hold duration, and step
  timeouts.

It does not command steering angle, steering rate, wheel torque, or a target
acceleration. Logged sample rate is measured from message timestamps and is
not guaranteed by the command-update rate. The collector records commanded
and reported throttle/brake, chassis and localization speed, acceleration,
commanded and actual gear, and driving mode.

The hold interval requires the trigger to remain continuously true. Each case
has a bounded stopping phase. New recordings persist a `.status.json` sidecar;
incomplete/aborted cases are excluded from calibration processing.

## Calibration output semantics

The table represents **net longitudinal acceleration** as a function of speed
and signed command (positive throttle, negative brake). Zero command is not
zero acceleration: drag, rolling resistance and grade still act on the vehicle.
No command row is forced to zero and positive throttle can have negative net
acceleration. Monotonic adjustment does not fill unmeasured grid cells.

Diagnostic CSVs retain missing cells. Control protobuf exports omit non-finite
cells with an explicit warning, retain measured zero-acceleration entries, and
reject exports with no valid coverage. Consumers must honor that coverage;
the exporter does not synthesize an extrapolated inverse control map.

Reported response thresholds are relative to **measured coast** and unavailable
without it. They are not actuator-internal deadzones. Fit residuals are
in-sample diagnostics, not independent repeat-run accuracy.

Braking from the maximum planned speed covers lower speeds during deceleration.
Separate starts at every speed are not mandatory; useful coverage and
repeatability, rather than the number of generated cases, determine sufficiency.

## Dynamic execution and analysis

Dynamic brake/both plans require `initial_speed_mps` when generated, producing a
`preparation` stage with a positive throttle command, target speed and timeout.
The default preparation throttle is 20%; choose it and the default 5 m/s dynamic
hard speed limit for the actual vehicle. Preparation precedes the timed log.
A stop before profile completion is an invalid run, not a zero-gain plant.

`generate_dynamic_plan` and the collector validate durations, actuator bounds,
gear and profile parameters. Symmetric profiles use a positive baseline by
default for single-pedal actuation; explicitly out-of-range excitation is
rejected instead of clipped. File-loaded plans receive the same preflight.

Only an isolated step is eligible for step metrics. Ramp/pulse/triangle/
hysteresis collection is supported, but `analyze_dynamic` explicitly rejects
these modes until dedicated quantitative analysis is available.

Frequency analysis requires uniform sample times and a matching sample rate.
The default Welch segment length is limited to half the record and at least two
overlapping segments are required; segment length and count are reported.
Bandwidth is only estimated when coherent low-frequency plateau and cutoff
coverage exist; otherwise it is unavailable. Phase delay includes plant dynamics
and must not be interpreted as pure transport delay. The step
`steady_state_error` compatibility field denotes a fitted-model residual, not
closed-loop tracking error.

## Boundaries

- Dynamic actuator profiles and their available fields are defined by the plan
  and collector, not by this document.
- Lateral steering, steady-state handling, and closed-loop tracking are
  documented in [`handling_test_phases.md`](handling_test_phases.md) and
  [`lateral_vehicle_dynamics.md`](lateral_vehicle_dynamics.md).
- Use the test inventory for available workflows and the assessment document
  for known limitations.
