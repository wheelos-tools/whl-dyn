# Lateral Vehicle-Dynamics Capability Assessment

## Current conclusion

The repository **partially satisfies** the requested lateral test program.
Phase 1 open-loop steering identification, Phase 2 fixed-steering steady-state
testing, and Phase 3 closed-loop curve tracking all have executable planning
and collection paths. Their vehicle-specific signal mappings, safety limits,
and acceptance thresholds still require integration validation.

The implementation must not be described as a complete vehicle-model
identification or physical-limit measurement system. The missing outputs are
listed below so that unavailable signals are not mistaken for passing results.

## Phase 1: open-loop actuator and vehicle response

### Available

- Positive and negative steering Step and slow-Ramp plans.
- Chirp, Sweep, and PRBS steering excitation.
- Speed stabilization before active excitation.
- Separate steering command and actual steering feedback fields.
- 100 Hz default sampling in the lateral plans.
- `steering_feedback -> yaw_rate_radps` and
  `steering_feedback -> lateral_accel_mps2` Bode reports.
- Gain, phase, coherence, approximate -3 dB bandwidth, resonance peak, and
  phase-based delay estimates.
- Generic Step metrics: dead time, 10--90% rise time, peak, overshoot,
  settling time, steady-state error, gain, and time constant.

### Not yet complete

- Independent standardized `delta_cmd -> delta_sw -> delta_road` reports.
- Return-to-center Step cases and steering acceleration metrics.
- Multi-speed, multi-amplitude, left/right, and repeated Step/FRF matrices.
- Sine-dwell or segmented single-frequency testing.
- Standardized `beta`/`v_y`, steering torque, individual wheel angles, wheel
  speeds, ESC/TCS/ABS status, and longitudinal acceleration fields.
- Bicycle-model parameter fitting, speed scheduling, or controller-ready
  state-space matrices.

The default plans are low-speed templates, not a validated 30/50/70/90 km/h
vehicle test matrix. Low-coherence frequencies and receive-time-only logs
must remain diagnostic rather than calibrated physical delay evidence.

## Phase 2: fixed-steering steady-state and physical limits

The current implementation supports **scheme B only**:

```text
fixed steering command -> multiple target speeds -> measured vehicle response
```

`plan-circles` is a fixed-steering open-loop matrix, not a fixed-radius
controller. It generates left/right cases, multiple speeds, and three repeats
by default. `select_steady_state_samples` removes the ramp, settling period,
and speed-outlier samples.

`analyze-steady-state` reports actual steering statistics, speed, yaw rate,
lateral acceleration, the small-sideslip curvature/radius approximation
(`kappa ~= r / v_x`, `R ~= 1 / abs(kappa)`), steering gains, optional sideslip,
ESC active fraction, wheel-slip peak, and configured limit flags.

The `steering_feedback` mapping must be the actual steering-wheel angle for
the “equal steering-wheel angle” interpretation. A separate effective
road-wheel angle is required for understeer-gradient calculations.

### Still required for physical-limit conclusions

- Reliable `beta` or `v_y`, wheel speeds/slip, ESC/TCS/ABS status, and steering
  torque mappings.
- Automated linear/nonlinear/limit-region classification.
- Persistent threshold logic for sideslip, sideslip growth, yaw-rate deviation,
  lateral-acceleration saturation, wheel slip, and ESC intervention.
- Cross-run aggregation of maximum stable lateral acceleration, repeatability,
  left/right differences, and the ESC intervention point.
- Front/rear axle saturation or tire-utilization estimation. These cannot be
  inferred reliably from `r` and `a_y` alone.

The current `max_abs_lateral_accel_mps2` setting is a safety stop, not an
automatically identified maximum stable lateral acceleration.

## Phase 3: closed-loop constant-curvature tracking

The current route is:

```text
straight entry -> clothoid entry -> constant-radius arc -> direct safe stop
```

The path is anchored once from localization and sampled from one immutable
global geometry. Each published window starts at the next planning cycle,
resets local `path_point.s` to zero, and keeps overlapping geometry identical.
This follows the relevant Apollo `TrajectoryStitcher` time-matching and local
`s` conventions. The publisher reference is:

- `whl_dyn/trajectory/apollo.py`
- Apollo `modules/planning/planning_base/common/trajectory_stitcher.cc`

### Available

- Configurable entry straight, entry Clothoid, radius, arc angle, speed, and
  direction.
- Direct `/apollo/planning` `ADCTrajectory` publication.
- Localization, chassis, control, and Apollo `simple_mpc_debug` recording.
- Lateral-error MAE/RMS/P95/peak and heading-error MAE/RMS helpers.
- Endpoint clamping, zero reference speed, and bounded direct braking stop.

### Still required for full controller evaluation

- Batch matrices for radius, speed, direction, repeats, load, surface, and
  controller version.
- Unified source-time alignment and unit normalization across localization,
  chassis, control, and debug rows.
- Entry overshoot, settling time, steady-state bias, and phase-specific
  metrics.
- Steering command/feedback error, steering rate/acceleration, oscillation
  count, high-frequency control energy, and saturation duty cycle.
- Reference versus measured yaw rate and lateral acceleration, sideslip, jerk,
  tire utilization, ESC state, and online safety aborts.

The runner must be the only publisher on `/apollo/planning`; stop or isolate
the normal planning publisher before active execution.

## Common vehicle-data requirements

For reusable results, preserve source timestamps, signal age, sampling jitter,
actual speed, longitudinal acceleration, steering units/signs, IMU mounting
and compensation, wheelbase, mass, axle loads, tire specification and
pressure, load, temperature, road grade/crossfall, and surface condition.

At minimum, a vehicle mapping should expose effective road-wheel angle, speed,
`r`, center-of-mass-compensated `a_y`, `v_y` or `beta`, roll, wheel speeds,
ESC/TCS/ABS state, and source timestamps. Missing optional signals must be
reported as unavailable.

## Sources

- Plans: `plans/`
- Planning: `whl_dyn/planning/`
- Collection: `whl_dyn/collection/`
- Processing: `whl_dyn/processing/`
- Trajectory publication: `whl_dyn/trajectory/`
- Tests: `tests/`
- Operations: `docs/handling_test_phases.md`,
  `docs/lateral_vehicle_dynamics.md`
