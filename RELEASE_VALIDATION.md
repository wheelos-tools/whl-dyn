# whl_dyn Industrial Release Validation

Date: 2026-03-27

## Scope
Validate that `whl_dyn` fully replaces legacy `whl-dyn` workflow and is deployable.

## Replacement Checklist
- [x] Plan generation: `whl_dyn.planning.generator` generates YAML plans.
- [x] Collection workflow UI: case table/source, single-case start/stop/retry, batch executor.
- [x] Real collector process launch from UI: `python -m whl_dyn.collection.collector --auto-start`.
- [x] Live log streaming and per-case status tracking in UI.
- [x] Automatic data sanity checks + manual confirmation gating.
- [x] Processing pipeline: filtering, latency alignment, LOF outlier handling, monotonic table construction.
- [x] WYSIWYG analytics: 3D surface, throttle/brake lookup tables, speed-slice response curves, step responses.
- [x] Export outputs: `unified_calibration_table.csv`, `calibration_table.pb.txt`, `evaluation_metrics.json`, `step_responses/*.png`.
- [x] Packaging/deployment: wheel and sdist build successful.

## Verified Runtime Evidence
- Syntax compile passed for core modules.
- Processing/export smoke test on existing logs:
  - RAW_FILES: 15
  - PROCESSED_ROWS: 14380
  - Required columns present: `command_type`, `aligned_speed`, `is_outlier`, `accel_aligned`
  - Export files existence check: all true
- Streamlit startup smoke test passed for `whl_dyn/ui/app.py`.
- `python -m build` succeeded.

## Build Artifacts
- `dist/whl_dyn-0.1.0-py3-none-any.whl`
- `dist/whl_dyn-0.1.0.tar.gz`

## Legacy Cleanup
- `legacy_whl-dyn` has been permanently deleted.

## Residual Risk Notes
- Real vehicle collection execution requires Apollo/Cyber runtime and messages at runtime.
- Non-Apollo host can validate UI process launching and log/error propagation but cannot validate vehicle bus behavior.
