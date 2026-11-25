# Simulation validation: 2026-11-24

This is a bounded integration experiment, not vehicle acceptance or a claim
that the simulator models a particular vehicle accurately. It covers the test
inventory except closed-loop trajectory tracking. Each availability assessment
distinguishes collection, analysis and suitability for calibration.

## Setup and evidence

- An isolated Apollo test container ran the MuJoCo backend, with no other
  simulator or control publisher on its isolated network. Longitudinal tests
  used throttle mode; lateral tests used speed mode.
- `whl-dyn` was tested from the then-current working tree, including
  uncommitted changes, on top of `74f4579`. The simulation source was based on
  `ea9ba11` with uncommitted changes. The loaded simulator backend library
  predates the source's wheel-speed correction; this experiment does **not**
  establish that correction was loaded and cannot validate wheel slip.
- The raw plans, CSVs, status files, logs, signal mapping and analysis outputs
  are retained in the local session artifact directory
  `simulation-validation-20261124/`. This directory is not part of the
  repository and may not be available on other machines.
- The existing software suite had 111 passing tests. That is regression
  evidence, not a substitute for the simulator results below or road tests.

## System timing corrections and repeat experiments

The results in this section supersede the corresponding pre-fix observations
below; the original evidence is retained for comparison. The simulator was
rebuilt in an isolated workspace with the current source, including the
wheel-speed correction. The default component interval and control period are
now both 10 ms; the physics step remains 2 ms. Real-vehicle rates are the
user-provided 100 Hz specification, not independently measured here.

The system changes preserve the existing timing-quality and steering safety
limits:

- Simulator chassis and localization messages share their actual publication
  wall time, rather than a fixed-step simulation clock extrapolated from an
  initial wall-clock offset. The internal physics clock remains fixed-step.
- The collector takes its snapshot time while holding the data lock and records
  source, receive, snapshot and command-publication timing separately. Invalid,
  future and stale source times are rejected.
- Frequency analysis uses independently timestamped source measurements on a
  common time grid, checking for gaps, conflicting or reversed timestamps,
  inadequate source rate and unsupported downsampling. Legacy records without
  independent steering timestamps use their actual snapshot times and must
  satisfy the uniform-sampling requirement.
- Steering preparation and collection check the rate against conservative
  actual publication intervals, rather than a nominal 10 ms interval. Welch
  frequency analysis requires at least two overlapping segments, avoiding
  single-segment coherence as evidence of reliability.

An independent chassis probe received 901 messages over approximately
9.001 s, or 99.994 Hz on average, with no sequence gaps and no source timestamp
ahead of its receive time; receive ages were approximately 0.107–1.163 ms.
This is an observation in this environment, not a real-time guarantee.

| Repeat | Outcome | Remaining limitation |
| --- | --- | --- |
| Original 100 Hz steady-turn plan, rate limit 5 | Both directions completed, each with 2,949 rows and 2,649 steady samples; radius was approximately 46.684 m. | Limit evaluation remains unavailable; a single operating point does not establish understeer or vehicle limits. |
| Original eight-case Phase 1 plan | All eight acquisitions reported completed; offline analysis using the final code reported 8/8 PASS. The four frequency cases each had 3,000 source samples at approximately 100 Hz and 22 Welch segments. | The full collector process did **not** exit normally after completing the cases: `timeout 300` returned 124. Thus this is a data-analysis result, not a fully successful end-to-end CLI run. Bandwidth remained unavailable and vehicle acceptance was not evaluated. |
| Extended sweep, 0.2–5 Hz, 30 s, 100 Hz, ±0.5% steering | Collection and analysis completed with normal process exit, 22 Welch segments and usable frequency points at approximately 0.391–2.734 Hz. | Yaw-rate and lateral-acceleration bandwidth remained unavailable; do not lower coherence or other validity criteria to manufacture a result. |

The full Phase 1 shutdown log repeatedly showed the Cyber scheduler waiting
for the localization reader task. A single-case rerun and the extended sweep
exited normally. A separate bounded live-feedback probe closed 300 chassis
readers normally; this does not reproduce or disprove the intermittent
localization shutdown issue. The Cyber Python reader binding releases the GIL
while draining callbacks but resets the native reader after reacquiring it;
a scheduler wait concurrent with a callback waiting for the GIL is a plausible
deadlock mechanism, not a confirmed root cause. The shared external Cyber
source was not modified.

The updated `whl-dyn` test suite passed 120 tests. Isolated simulator builds
passed `cyber_adapter_test`, `mujoco_backend_test` and
`simulation_engine_test`. Raw repeat evidence is retained in
`chassis_fixed_probe.csv`, `circle_fixed2_runs/`, `phase1_fixed2_runs/`,
`phase1_fixed2_report.json`, `phase1_fixed2.log` and `sweep_extended_runs/`
under the local session artifact directory.

## Availability by test item before system timing corrections

| Inventory ID | Observed result | Availability and limits |
| --- | --- | --- |
| LON-CAL | At 0.6 m/s, 4/4 cases completed, but the default processing window discarded all 20% brake samples. At 1.2 m/s, 5/5 completed; brake 5%, 12%, 20% retained 119, 39, 47 samples respectively, and throttle 10%, 20% retained 135, 234 across all cases, including brake preparation. | Bounded low-speed acquisition and processing work; the shorter brake segment is insufficient at 0.6 m/s. Coverage and independent repetitions are inadequate for a vehicle calibration. |
| LON-CAL-EXPORT | The 0.6 m/s data formed a 3 × 7 grid with 12/21 finite cells. Export produced finite protobuf entries and warned about 9 missing cells. The 1.2 m/s data formed a 9 × 6 grid with 40/54 finite cells. | Partial-table export works; interpolated finite cells are not independent measurements. Neither table is a complete validated calibration. |
| LON-CAL-QUALITY | No zero-command samples entered either calibration dataset. A separate 1.2 m/s, 4 s zero-throttle dynamic run completed: speed fell from 1.2042 to 1.1946 m/s; median longitudinal IMU acceleration was approximately -0.0024 m/s². The calibration loader excluded that dynamic run. | Response thresholds remain unavailable. This coast diagnostic does not establish a coast-reference calibration workflow; no independent repeatability or withheld-run accuracy was assessed. |
| LON-DYN-STEP | Throttle and pre-accelerated brake steps completed. Reported gains were about 0.0403 and 0.0807 m/s² per command percentage point; rise time was 0 and fitted time constant about 0.02 s. | Basic capture and analysis work in this idealized low-speed model. Response timing is at the sampling resolution, not an identified real actuator delay. |
| LON-DYN-FRF | Single sine, chirp, sweep, multi-sine and PRBS profiles completed; frequency-response arrays were produced, but bandwidth was unavailable. Four-second records have at most 256 samples and yield only one Welch segment with default settings. | Acquisition and preliminary spectral output are available; single-segment coherence near 1 is not independent evidence of reliable coherence or bandwidth. Longer, appropriately excited repeat runs are needed. |
| LON-DYN-RAW | Ramp, pulse, triangle and hysteresis profiles completed. | Raw capture works; the generic dynamic analyzer intentionally rejects these profiles rather than mislabeling them as steps. Dedicated quantitative analysis is not implemented. |
| LAT-OL-STEP | Both directions passed Phase 1 at 1 m/s and ±2% steering, 400 samples each. Feedback gain was approximately 0.895 and rise time 0.600 s; yaw-rate rise time was 0.680 s. | Basic execution and observable-response checks work at this operating point; this is not a vehicle performance acceptance. |
| LAT-OL-RAMP | Both directions passed Phase 1 at the same operating point. | Feedback range/rate is observable; steering deadzone, hysteresis and true maximum steering rate are not established. |
| LAT-OL-FRF | Both chirps passed the basic Phase 1 checks, but only 0.391 and 0.781 Hz survived analysis and bandwidth was unavailable. Both PRBS cases failed analysis: one on timestamp nonuniformity, the other on negative localization ages; that second record also exceeded the interval-jitter tolerance. A separate 15 s, 50 Hz sweep collected 750 samples but its frequency analysis rejected nonuniform sampling. | Chirp produces a limited report, not a validated bandwidth. PRBS and sweep were collected but did not pass analysis under the current timing quality rules. |
| LAT-SS | The generated 100 Hz plan failed before sampling with `steering profile exceeds configured rate`; increasing the limit from 5 to 10 still failed. A diagnostic-only plan at 20 Hz and limit 25 completed both directions, 591 samples each, with 531 selected as steady. | Default-plan execution is not established. Modified settings demonstrate a usable analysis path, not a fix or default-plan pass; no Ku or vehicle-limit evaluation was established. |

Phase 1 was **FAIL, 6/8 cases passing**. Its individual PASS results only
indicate execution and observable responses; `vehicle_acceptance` was
`NOT_EVALUATED`.

## Pre-fix diagnostic details

### 100 Hz feedback follow-up

The 50 Hz feedback rate above was the configured 20 ms simulation component
interval, not a demonstrated MuJoCo maximum. A separate diagnostic changed
only the isolated simulation DAG interval to 10 ms and `sim_control_dt` to
0.01 s, retaining the 0.002 s physics step (five substeps). Repository defaults
had not yet been changed at that point. The user's stated real-vehicle command
and feedback rates are 100 Hz; this experiment did not measure a real vehicle.

A separate Cyber chassis subscriber received 1,001 messages in ten seconds:
100.002 Hz mean, with 8.523–11.311 ms arrival intervals and no sequence
gaps. This demonstrates approximately 100 Hz in this environment, not
hard-real-time guarantees or the maximum supported rate. Under 100 Hz source
and 100 Hz sampling, the four 30 s chirp/PRBS runs had approximately 100
observed source-timestamp updates per second and no repeated timestamps in
these particular captures.

The unchanged eight-case Phase 1 plan was then **FAIL, 5/8 PASS**: four
step/ramp cases and one PRBS passed, while both chirps and the other PRBS
failed the existing uniform-sampling check. The failing runs had respectively
8, 2 and 1 interval(s) outside the existing 5% tolerance; this independent
repeat does not establish that the higher source rate causes the difference
from the earlier 6/8 result. Individual PASS still does not imply vehicle
acceptance. The unmodified 100 Hz steady-turn plan again aborted in
`establishing_steering`, before recording any samples.

An independent ten-second chassis subscriber observed 94 source timestamps
ahead of callback wall time, by up to 0.105 ms. Thus source time can be in the
future even without the lateral collector's snapshot race. The simulator
constructs message time from an initial wall-clock offset and its fixed-step
simulation time, which is a plausible contributor; clock and scheduling
behavior require further measurement before assigning the entire cause.

Separately, a deterministic test of the existing steering preparation method
used an 11 ms actual interval at a nominal 100 Hz. A requested 5
percentage-point/s linear ramp was calculated as 5.5 and rejected, although
its actual command slope was 5. This demonstrates a nominal-period estimation
defect, without proving that it was the only contributor to every recorded
abort.

The original 15 s sweep plan, still sampling at 50 Hz, was also repeated
against the 100 Hz source. All 750 rows were collected and the unmodified
frequency analyzer completed; sample intervals were 19.894–20.102 ms.
Both response reports retained five frequency points but had unavailable
bandwidth. The earlier 50 Hz-source sweep failed its timing check, so these
two single runs do not establish which factor caused the difference.

At 0.6 m/s, 20% braking lasted about 0.30 s in the raw data. The default 300 ms
brake stability window and signal alignment left no usable samples. At 1.2 m/s
the same brake command retained 47 samples, showing this observation depends
on the operating condition, not that the whole brake workflow is unusable.
However, neither the low-speed grid nor the coast diagnostic establishes a
production calibration.

For the positive lateral PRBS, 56 of 2,999 sample intervals fell outside the
5% tolerance; observed intervals were about 0.00897–0.01098 s. The negative
PRBS contained six negative localization ages, minimum approximately
-0.0004215 s, and 94 of 2,999 intervals outside tolerance. Acquisition timing
and snapshot timestamp ordering require investigation; the evidence does not
isolate a single root cause. Relaxing time-quality checks would not establish
correctness.

In the modified steady-turn diagnostic, mean speed was 0.99537 m/s, steering
feedback was ±8.9539% (not radians), yaw rate ±0.02132 rad/s, lateral
acceleration ±0.02122 m/s², curvature ±0.02142 1/m, and radius 46.68 m. The
two ideal-model results were symmetric. Limit evaluation was explicitly
unavailable; sideslip, slip and a valid road-wheel angle were not supplied.
One operating point cannot identify understeer gradient.

The steering-preparation code derives commands from actual elapsed time but
checks adjacent commands using nominal sampling frequency. Scheduling and
publication delay can make this check sensitive to jitter. This is a plausible
mechanism, not a measured proof of the specific failures; capture interval and
command traces before changing safety enforcement.

## Next validation priorities

1. Reproduce and resolve the intermittent Cyber localization-reader shutdown
   issue, then rerun the complete Phase 1 CLI and require normal exit.
2. Add a validated coast-reference path and independent repeats; expand speed
   and command coverage and evaluate withheld runs before using a map.
3. Run longer, independently repeated frequency experiments with sufficient
   excitation, spectral averaging and coherent cutoff coverage.
4. Validate wheel-speed and slip against suitable independent reference data.
   Physical tire, actuator and sensor realism and vehicle-specific safety
   limits remain separate validation requirements.
