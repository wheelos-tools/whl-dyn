# whl_dyn User Guide

Contents

1. Environment
2. Quick start
3. Modules and workflows
4. Troubleshooting

1. Environment

- Linux (Ubuntu 20.04+ recommended)
- Python 3.7+
- Optional: Docker / Apollo dev container when integrating into Apollo

Install

    pip install -r requirements.txt
    # or
    pip install -e .

2. Quick start

Start the Streamlit UI (from project root):

    streamlit run whl_dyn/ui/app.py

Visit: http://localhost:8501

Command-line options passed to Streamlit are supported, e.g.:

    streamlit run whl_dyn/ui/app.py --server.port 8502 --server.address 0.0.0.0

Basic workflow

1. Generate a calibration plan (YAML) with test cases and parameters.
2. Execute collection for one or more test cases; logs are saved as CSV files.
3. Run analysis: tune filters/delay compensation, inspect 2D/3D visualizations.
4. Export calibration tables and diagnostics.

Files and locations

- calibration_plan.yaml — generated plan (default location used by UI)
- calibration_data_logs/ — collected CSV logs
- calibration_results/ — processed outputs and plots

3. Modules and UI

- Planning: create the test matrix (throttle/brake ranges, steps, hold times).
- Collection: run the collector (supports auto mode) and stream logs to the UI.
- Processing: LOF outlier detection, Butterworth filtering, delay compensation, monotonicity checks, interpolation, and table export.
- UI: workbench to run the above steps and evaluate quality metrics (dead zone, R², smoothness, monotonicity).

Quality indicators (examples)

- Dead zone: < 3% is excellent
- Linearity (R²): > 0.98 is excellent
- Smoothness: > 85 is excellent

4. Troubleshooting

- If pip is slow inside a container, use a mirror:

    pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt

- To enter the Apollo dev container (if used):

    ./docker/scripts/dev_start.sh
    ./docker/scripts/dev_into.sh

For more detailed architecture and rationale, consult DESIGN.md.
