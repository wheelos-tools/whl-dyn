# Architecture

## When

Read when changing data flow, module boundaries, or vehicle-dynamics behavior.

## Facts

- Plans are generated in `whl_dyn/planning/` and executed by collectors in
  `whl_dyn/collection/`.
- Processing modules operate on persisted CSV data rather than live CyberRT
  messages.
- Lateral open-loop collection uses vehicle-local signal mappings; generic code
  uses semantic signal names.
- Closed-loop reference geometry and Apollo publication are separated between
  `whl_dyn/trajectory/` and `whl_dyn/collection/closed_loop.py`.
- UI and CLI are entry surfaces, not the source of planning or analysis logic.
- The Streamlit workbench (`whl_dyn/ui/app.py`) follows a unified two-level hierarchy:
  1. Top-level domain selection (`🚗 油门/刹车`, `🧭 横向动力学`).
  2. A consistent 3-stage paradigm under each domain:
     - `📋 ① 生成计划`: Preset template loading, parameter configuration, custom output path in `plans/`, and table/source YAML preview with direct editing and saving.
     - `🚗 ② 数据采集`: Case/run selector, unified toolbar actions (`▶ 开始`, `⏹ 停止`, `↺ 重试`, `🗑 清除`), and real-time logs.
     - `📊 ③ 分析`: Run directory selection, metrics evaluation, and interactive visual reports.
- Standard artifact locations are fixed and unified:
  - Preset and generated test plans: `plans/`
  - Longitudinal collection logs & results: `calibration_data_logs/`, `calibration_results/`
  - Lateral collection runs & analysis: `vehicle_dynamics_runs/`, `vehicle_dynamics_runs/<run>/analysis/`
  - Temporary per-case execution plans: `.whl_dyn_runtime/` (ignored)
  - Vehicle signal mappings: `vehicle_signals.yaml`

## Sources

- `whl_dyn/planning/`
- `whl_dyn/collection/`
- `whl_dyn/processing/`
- `whl_dyn/trajectory/`
- `whl_dyn/ui/`
- `docs/handling_test_phases.md`
- `docs/lateral_vehicle_dynamics.md`
- `.agents/knowledge/vehicle-dynamics.md`
