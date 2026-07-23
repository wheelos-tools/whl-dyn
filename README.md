# whl_dyn — Vehicle Longitudinal Dynamics Calibration (Quick Start)

Overview

whl_dyn is a Python toolkit and Streamlit workbench for generating calibration plans, collecting vehicle actuator data, processing signals, and exporting calibration tables and diagnostics. The detailed design notes were preserved in DESIGN.md.

Quick start (local / in-container)

1. Install dependencies:

    pip install -r requirements.txt

2. Run the Streamlit workbench (from repo root):

    streamlit run whl_dyn/ui/app.py

3. Open the UI in a browser at http://localhost:8501

Primary artifacts

- Generate calibration plans (YAML)
- Collect CSV logs per test case
- Produce calibration tables (CSV, protobuf text)
- Visualize 3D response surfaces and step responses

Repository layout (high level)

- whl_dyn/planning — test matrix and plan generator
- whl_dyn/collection — data collector & CyberRT integration
- whl_dyn/processing — filtering, outlier detection, interpolation
- whl_dyn/ui — Streamlit dashboard binding all modules

How this README helps an agent learn

- Clear entrypoint (whl_dyn/ui/app.py) and expected runtime (Streamlit)
- Module boundaries and responsibilities listed above
- Key file locations for plans, logs, and results

If you need the user manual, see docs/USER_GUIDE.md. For design rationale and architecture details, see DESIGN.md.
