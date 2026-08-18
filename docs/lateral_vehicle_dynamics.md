# Lateral Vehicle-Dynamics Frequency Tests

`whl-dyn` implements the collection-first workflow:

```text
plan -> collect a timestamped raw run -> analyze -> metrics, Bode CSV and plots
```

It supports the following test cases:

- Frequency Sweep / Chirp;
- PRBS;
- steering-feedback to yaw-rate frequency response;
- steering-feedback to lateral-acceleration frequency response.

The checked-in ready-to-configure plans are:

```text
plans/lateral_chirp.yaml
plans/lateral_prbs.yaml
```

Both use a configurable 120-second excitation after speed settles, with
`ControlCommand.speed=2.0 m/s` by default.  They are templates: validate the
vehicle signal mapping and steering scale in record-only mode before active
execution.

## Generate a plan

```bash
whl-dyn plan-lateral --mode chirp --output chirp.yaml \
  --frequency-start-hz 0.05 --frequency-end-hz 2.0 \
  --duration-sec 120 --speed-min-mps 0 --speed-max-mps 3.0 \
  --target-speed-mps 2.0 --speed-tolerance-mps 0.15

whl-dyn plan-lateral --mode prbs --output prbs.yaml \
  --bit-duration-sec 0.25 --prbs-seed 7
```

Plans use only semantic names such as `front_steering_feedback`,
`yaw_rate_radps` and `lateral_accel_mps2`; they contain no vehicle-specific
protobuf fields.

## Configure signals

Create a vehicle-local YAML mapping.  This is the only layer where protobuf
module names and field paths are configured.

```yaml
topics:
  chassis: /apollo/canbus/chassis
  chassis_detail: /apollo/canbus/chassis_detail
  localization: /apollo/localization/pose
  control: /apollo/control

detail_message:
  module: your_vehicle_proto_module
  class: YourVehicleDetail

detail_fields:
  # Required for active tests. Map this to one normalized effective wheel
  # angle; it is the analysis input, not merely a command echo.
  steering_feedback: feedback.effective_steering_angle
  front_steering_command: command.front_steering_angle
  rear_steering_command: command.rear_steering_angle
  front_steering_feedback: feedback.front_steering_angle
  rear_steering_feedback: feedback.rear_steering_angle

# Converts the generic ControlCommand steering target to the vehicle command
# unit.  Keep 1.0 only when their units already agree.
control_steering_scale: 1.0

# Converts raw actual feedback into the same normalized steering unit used by
# the test profile and safety limit. It may be negative for an inverted sensor.
steering_feedback_scale: 1.0

# Active collection rejects stale actual steering feedback.
max_feedback_age_sec: 0.2
```

Localization supplies `yaw_rate_radps` from vehicle-frame angular velocity Z
and `lateral_accel_mps2` from vehicle-frame linear acceleration X.  file also retains source and collector timestamps, signal ages, sample time,
alignment skew and a `time_aligned` flag, chassis speed, driving mode, raw
steering command/feedback, and roll. Offline analysis should filter on the
alignment flag and configured skew threshold before estimating phase or delay.

## Collect

Start with record-only mode to verify signal fields, signs and time bases:

```bash
whl-dyn collect-lateral --plan chirp.yaml \
  --signal-config vehicle_signals.yaml --output-dir runs
```

The collector creates one non-overwriting directory per case:

```text
runs/20260817T072300Z_lateral_chirp_3.0_a1b2c3d4/
  metadata.yaml
  samples.csv
  status.json
```

Command injection is intentionally explicit:

```bash
whl-dyn collect-lateral --plan chirp.yaml \
  --signal-config vehicle_signals.yaml --output-dir runs --execute --arm
```

The plan constrains absolute steering and steering rate.  The operator must
also validate the speed gate and vehicle safety procedure before executing an
open-loop test.

Before any active Phase 1/2 run, execute:

```bash
whl-dyn validate-open-loop --plan your_open_loop_plan.yaml \
  --signal-config vehicle_signals.yaml --active
```

The command verifies that the plan contains no planning trajectory and that
the configuration exposes a normalized actual `steering_feedback` field. It
does not replace record-only verification of the live signal unit and sign.

For an active test, the collector continuously publishes `ControlCommand.speed`
at `target_mps` while steering remains zero.  It starts the Chirp/PRBS only
after chassis speed stays within `target_mps +/- tolerance_mps` for
`stable_duration_sec`; it aborts if that does not occur by `max_wait_sec`.
During the excitation, every control message contains both the same speed
target and the steering profile.  Leaving the independent
`min_mps..max_mps` hard range ends the case and commands zero speed/steering.
The plan also reserves message freshness, driving mode and fault-state abort
policies; those three checks are recorded as reserved until vehicle-specific
acceptance tests are complete.

## Choosing PRBS timing and amplitude

Amplitude is a steering command angle in the command unit defined by
`control_steering_scale`; it should begin with the smallest value that produces
a measurable yaw-rate response.  The default is 2 command units.

For a bipolar PRBS that changes from `-A` to `+A`, the worst command rate is:

```text
max_rate_required = 2 * A / bit_duration
```

Choose `bit_duration >= 2 * A / max_steering_rate`.  For example, `A=2` and
`max_steering_rate=30` require at least `0.134 s`; the default `0.25 s` is
therefore valid.  For Chirp, the peak sinusoidal rate is
`2*pi*A*frequency_end`; the plan generator rejects either excitation when it
exceeds the configured rate limit.

## Analyze

```bash
whl-dyn analyze-lateral --run-dir runs/20260817T072300Z_lateral_chirp_3.0_a1b2c3d4
```

The analyzer prioritizes `steering_feedback` as the input.  If it is absent,
it can combine `front_steering_feedback` and `rear_steering_feedback`; set
`--steering-column` when the vehicle uses another already-normalized signal.

It writes:

```text
analysis/metrics.json
analysis/bode_yaw_rate.csv
analysis/bode_yaw_rate.png
analysis/bode_lateral_acceleration.csv
analysis/bode_lateral_acceleration.png
```

Each response reports Gain, Phase, coherence, -3 dB bandwidth, resonance peak
and estimated delay.  Treat low-coherence frequencies and receive-time-only
logs as diagnostic evidence rather than calibrated physical delay. When a run
contains `time_aligned`, the analyzer rejects rows whose flag is false and
fails if no aligned rows remain.
