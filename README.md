# Vehicle Longitudinal Dynamics Calibration Pipeline (whl_dyn)

This is an industrial-grade, full-lifecycle vehicle longitudinal dynamics calibration toolkit.

## 1. Overview
The `whl_dyn` package modularizes the original scripts into a professional Python architecture with a unified Streamlit dashboard. It enables engineers to generate test matrices, collect vehicle data via CyberRT, process it with advanced filtering (LOF, Butterworth, delay compensation, monotonicity constraints), and visualize the dynamics table in 3D.

### Architecture
* **`whl_dyn.planning`**: Test matrix generator.
* **`whl_dyn.collection`**: CyberRT vehicle listener and command publisher.
* **`whl_dyn.processing`**: Core algorithms (LOF, low-pass filters, monotonicity, interpolation).
* **`whl_dyn.ui`**: Unified Streamlit UI binding planning, collection, processing, and analytics together.

## 2. Installation & Quick Start

```bash
pip install streamlit plotly pandas numpy scipy scikit-learn pyyaml protobuf
# Run the dashboard from the project root
.venv/bin/streamlit run whl_dyn/ui/app.py
```

## 3. Workflow (3-Workbench Dashboard)
1. **📑 1. Test Plan Generation**: Adjust boundary conditions and automatically output `calibration_plan.yaml`.
2. **📡 2. Data Collection Workflow**: View the plan in table form, start a real collector subprocess from the UI, stream logs live, run automatic sanity checks, retry failed cases, and manually confirm passed cases.
3. **🧠 3. Processing & Analytics**: Tune filtering and delay parameters and immediately observe processed samples, 3D calibration surfaces, step response curves, and table quality metrics in one combined WYSIWYG workbench.

## 4. Batch Execution
Use `Start batch executor` in the collection workbench to run the plan case-by-case.

For each case:
1. The UI writes a temporary one-case YAML plan.
2. The UI launches `python -m whl_dyn.collection.collector --auto-start`.
3. Live stdout is streamed into the dashboard.
4. After completion, the newest CSV log is checked automatically.
5. If the check passes, the UI waits for manual approval before launching the next case.
6. If the check fails, retry the current case.

## 5. Calibration Outputs
The integrated `whl_dyn` pipeline exports the same core artifacts required for deployment:

- `unified_calibration_table.csv`
- `calibration_table.pb.txt`
- `evaluation_metrics.json`
- `step_responses/*.png`

These can be exported directly from the `Processing & Analytics` workbench.

## 6. Replacement Status
`whl_dyn` now covers the full legacy flow:

- Plan generation (`generate_plan.py` equivalent)
- Collection execution (`collect_data.py` equivalent, with real subprocess and live logs)
- Processing and table export (`process.py` equivalent)
- Visualization and response analysis (`plot.py` equivalent)

The old legacy folder has been permanently removed after replacement verification.

## 7. Deployment Build
Create distributable artifacts:

```bash
.venv/bin/python -m build
```

Generated artifacts:

- `dist/whl_dyn-0.1.0-py3-none-any.whl`
- `dist/whl_dyn-0.1.0.tar.gz`
