# Handling Tests: Phases 1–3

The first three phases use independent modules:

```text
planning/handling.py       test matrix and feasibility validation
trajectory/continuous.py   immutable circle/clothoid geometry
trajectory/apollo.py       overlapping ADCTrajectory publication
collection/                CyberRT execution and timestamped raw collection
processing/handling.py     delay/handling/tracking metrics
```

## Phase 1: open-loop identification

Phase 1 has **no planning trajectory**. Stop or isolate the ordinary control
publisher before active testing: this test owns `/apollo/control` and sends
only the configured constant `speed` plus its steering profile.

```bash
whl-dyn plan-open-loop --output open_loop_identification.yaml \
  --target-speed-mps 2.0 --amplitude 2.0 --ramp-rate 1.0

# Static check: no trajectory, complete speed gate, valid test types.
whl-dyn validate-open-loop --plan open_loop_identification.yaml

# After creating vehicle_signals.yaml, additionally require real steering
# feedback before an active run.
whl-dyn validate-open-loop --plan open_loop_identification.yaml \
  --signal-config vehicle_signals.yaml --active
```

The resulting step and slow-ramp cases are collected with
`whl-dyn collect-lateral`. Each persisted row is a fixed-time snapshot with
`sample_time_sec`, source timestamps, source ages, `alignment_skew_sec`, and a
`time_aligned` flag. The plan covers positive and negative steering:

```text
step:      command -> front/rear feedback -> yaw rate -> lateral acceleration
slow ramp: deadzone, hysteresis, actual steering-rate limit
chirp/PRBS: gain, phase, bandwidth, coherence and delay
```

Run open-loop profiles only with `--execute --arm`; record-only remains the
first integration step.  Use the actual steering feedback rather than the
command as the primary yaw/acceleration frequency-response input.

### Phase 1 on-vehicle order

1. Start a **record-only** run and confirm that `samples.csv` contains
   `steering_feedback`, `chassis_speed_mps`, `yaw_rate_radps`, and
   `lateral_accel_mps2`; confirm the feedback unit/sign changes as expected.
2. Run the two small positive/negative slow-ramp cases.  These identify
   deadzone, hysteresis and achieved wheel rate without an intentional step.
3. Run the two small positive/negative steering-step cases.  A command step is
   intentional; report measured command-to-feedback and feedback-to-yaw
   timing separately.
4. Run Chirp, then PRBS only after the step responses are bounded and the
   front/rear or effective angle feedback is healthy.

Every active case first holds `ControlCommand.speed=target_mps` until actual
speed remains inside `target_mps +/- tolerance_mps` for three seconds. During
the case it stops on a hard speed-range violation, stale actual steering
feedback, actual feedback beyond the configured steering limit, lateral
acceleration cap, profile limit violation, or user interrupt. It then sends
zero speed and zero steering.

## Phase 2: steady-state circles

Phase 2 is also **open loop** and has the same `/apollo/control` ownership
requirement. It does not use `/apollo/planning`.

```bash
whl-dyn plan-circles --output steady_state_turns.yaml \
  --steering-commands 1,2,3 \
  --speed-targets-mps 1,2,3 \
  --steering-ramp-rate 0.5 \
  --max-lateral-accel-mps2 1.5
```

This is an **open-loop** test: it publishes only `ControlCommand.speed` and a
rate-limited fixed `ControlCommand.steering_target`; it does not publish a
planning trajectory.  The actual radius/curvature and lateral acceleration are
measurements.  Consequently, the fixed steering test determines the
relationship:

```text
fixed steering + speed -> measured kappa, yaw rate, ay
```

The generated matrix includes both directions and repetitions. It ramps from
zero to the requested fixed steering angle, holds it through the steady
window, and aborts on the configured lateral-acceleration cap. Select only a
steady window after the ramp, saturation and speed-transient samples have been
removed. Then fit:

```text
delta - L * kappa = Ku * ay + offset
```

with `steady_state_handling_metrics`.  `Ku` is not valid until the steering
unit is converted to wheel angle radians and the wheelbase is known.

The plan separates `max_abs_steering` (the commanded profile limit) from
`max_abs_feedback_steering` (the actual-angle hard stop).  The generated
Phase-2 plan gives feedback a 20% margin for normal actuator overshoot; change
this only with a vehicle-approved mechanical limit.

### Phase 2 on-vehicle order and stable-window rule

1. Begin with the smallest positive/negative steering command at the lowest
   speed, and complete all repeats before increasing either variable.
2. Run the preflight and one record-only run for each new signal mapping or
   unit conversion:

   ```bash
   whl-dyn validate-open-loop --plan steady_state_turns.yaml \
     --signal-config vehicle_signals.yaml --active
   whl-dyn collect-lateral --plan steady_state_turns.yaml \
     --signal-config vehicle_signals.yaml --output-dir vehicle_dynamics_runs
   ```

3. Start active execution only after reviewing the record-only signal signs:

   ```bash
   whl-dyn collect-lateral --plan steady_state_turns.yaml \
     --signal-config vehicle_signals.yaml \
     --output-dir vehicle_dynamics_runs --execute --arm
   ```

Each raw `samples.csv` has `case_phase=ramp` until the fixed steering target
is reached and `case_phase=steady` afterward.  For the steady-state fit:

```text
discard: all ramp samples
discard: first 3 seconds after the steady phase begins
keep:    chassis_speed_mps within target_mps +/- tolerance_mps
discard: any run terminated by a safety limit
```

Use `select_steady_state_samples` before `steady_state_handling_metrics`.
Only aggregate repeated left/right runs after their individual steady windows
are valid.  The lateral-acceleration cap is a safety stop, not a target to
reach.

### Phase 2 common-metric report

For the fixed-steering (scheme B) scope, analyze each valid run after it has
settled:

```bash
whl-dyn analyze-steady-state \
  --run-dir vehicle_dynamics_runs/<fixed_steering_run> \
  --target-speed-mps 10.0 \
  --steering-wheel-column steering_wheel_angle_rad \
  --road-wheel-column effective_road_wheel_angle_rad \
  --wheelbase-m 2.8 \
  --max-abs-sideslip-rad 0.10 \
  --max-sideslip-rate-radps 0.05 \
  --max-yaw-rate-error-radps 0.15 \
  --max-abs-wheel-slip 0.10
```

It writes `analysis/steady_state_metrics.json`, including the actual speed,
actual steering-wheel and road-wheel angle statistics, mean yaw rate and
lateral acceleration, the small-sideslip curvature/radius estimate
(`kappa ~= r/v_x`, `R ~= 1/abs(kappa)`), yaw-rate and lateral-acceleration
gain, sideslip, ESC active fraction, wheel-slip peak, and available limit
flags.

`sideslip_rad`, `esc_active`, and `wheel_slip_ratio` are optional vehicle
mapping columns. A missing column is written as unavailable; it is never
treated as proof that the vehicle remained stable. Configure
`steering_feedback` as the actual steering-wheel angle for this test, and
map a separate effective road-wheel angle column when calculating vehicle
gains or understeer gradient. The steady-state and tracking processors filter
out rows whose `time_aligned` flag is false. The analyzer does not yet
aggregate multiple runs or infer front/rear axle saturation.

## Phase 3: closed-loop Clothoid-to-circle tracking

```bash
whl-dyn plan-closed-loop --output closed_loop_curve.yaml \
  --radius-m 50 --speed-mps 2 \
  --straight-entry-length-m 20 --entry-length-m 15 \
  --arc-angle-rad 1.57 --direction left

whl-dyn run-closed-loop --plan closed_loop_curve.yaml \
  --output-dir vehicle_dynamics_runs
```

The runner publishes `ADCTrajectory` directly to `/apollo/planning`.  It must
be the **only publisher** on that topic for the vehicle under test; stop or
isolate the ordinary planning publisher first.  It is not a RoutingRequest
and does not invoke `GenerateRefLineFromRawPath`.

The runner records one fixed-rate snapshot per published planning window, not
one row per callback. Each row contains `sample_time_sec`, localization,
chassis, and control source timestamps, source ages, `alignment_skew_sec`, and
`time_aligned`. The default maximum source-time skew is 50 ms and is stored in
the generated case as `max_alignment_skew_sec`. Reports must reject rows whose
flag is false.

The route is:

```text
straight entry -> clothoid entry -> constant-radius arc -> direct safe stop
```

At startup it anchors this path once from localization. Every later frame
samples the same immutable path from the next planning cycle:

```text
global_s = (experiment_elapsed + planning_cycle_time
            + trajectory_relative_time) * speed
```

Thus shared future points in two consecutive frames have identical
`x/y/theta/kappa`. `path_point.s` is local to each published window and begins
at zero; `relative_time` begins at the planning cycle. This follows the
relevant Apollo `TrajectoryStitcher` convention: it time-matches a prior
trajectory, preserves valid points, and resets `s` at the stitch point. The
test publisher deliberately does not re-anchor or transform the reference
from vehicle pose on every frame, because that would alter the controlled
geometry and invalidate the experiment.

Reference: Apollo
[`trajectory_stitcher.cc`](https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planning_base/common/trajectory_stitcher.cc),
especially `ComputeStitchingTrajectory` and `ComputeReinitStitchingTrajectory`.

Raw output records localization, chassis and control/debug samples in a unique
run directory as fixed-rate snapshots rather than one row per asynchronous
callback. Each row retains source timestamps and alignment diagnostics.
Apply `tracking_metrics` only to the selected steady segment:

```text
ey MAE / RMSE / P95 / Peak
epsi MAE / RMSE
```

The configured duration ends at the constant-radius arc endpoint. The final
forward window is clamped at that endpoint with zero reference speed;
the runner then sends a bounded zero-speed, zero-steering brake command when
the case ends, is interrupted, or fails. Validate this direct-stop behavior on
the vehicle integration before active use.
