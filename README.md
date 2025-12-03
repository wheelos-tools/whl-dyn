# whl-dyn

Vehicle Dynamics Calibration Tools

## Overview

This repository contains tools for vehicle dynamics calibration and data processing. The `whl-dyn/tools/` directory provides scripts for:
- Generating calibration plans with configurable test scenarios
- Collecting vehicle dynamics data through automated test plans
- Processing and analyzing calibration data to extract acceleration characteristics
- Generating calibration tables for vehicle control systems
- Visualizing results through interactive plots
- Converting results to Protocol Buffer format for integration

## Documentation

- [English Documentation](whl-dyn/tools/README.md)
- [中文文档](whl-dyn/tools/README_CN.md)

## Workflow

```
Generate Calibration Plan → Automatic Data Collection → Data Processing → Result Visualization
(whl-dyn/tools/generate_plan.py) (whl-dyn/tools/collect_data.py) (whl-dyn/tools/process.py) (whl-dyn/tools/plot.py)
```

## Tools Directory Scripts

### 1. **generate_plan.py** - Calibration Plan Generation
- Creates YAML test plans for vehicle dynamics calibration
- Configurable throttle and brake test scenarios
- Generates acceleration tests at different speeds

### 2. **collect_data.py** - Automated Data Collection
- Implements automated data collection using predefined calibration plans
- Uses CyberRT framework for vehicle communication
- Executes test cases defined in YAML format
- Generates CSV logs with vehicle state data
- Supports custom output directories via command line arguments

### 3. **process.py** - Data Processing and Analysis
- Preprocesses raw vehicle data from CSV logs
- Extracts acceleration characteristics from test data
- Segments data based on control command changes
- Generates calibration tables mapping speed/command to acceleration
- Supports optional protobuf calibration table output

### 4. **plot.py** - Data Visualization
- Plots vehicle dynamics calibration data
- Generates both 3D surface plots and 2D contour plots
- Provides interactive plotting with keyboard controls
- Visualizes throttle and brake dynamics separately

## Quick Start

### 1. Generate Calibration Plan
```bash
python whl-dyn/tools/generate_plan.py
```

### 2. Collect Data
```bash
# Use default plan file
python whl-dyn/tools/collect_data.py

# Use custom plan and output directory
python whl-dyn/tools/collect_data.py -p calibration_plan.yaml -o output_directory/
```

### 3. Process Data
```bash
# Process CSV files and generate calibration table
python whl-dyn/tools/process.py -i calibration_data_logs/ -o results/

# Process with protobuf output
python whl-dyn/tools/process.py -i calibration_data_logs/ -o results/ --output-calibration-table
```

### 4. Visualize Results
```bash
python whl-dyn/tools/plot.py -i results/calibration_table.pb.txt
```

## Dependencies

- Python 3.7+
- matplotlib
- numpy
- scipy
- pandas
- PyYAML
- CyberRT (for data collection)

## Detailed Tool Usage

### Data Processing with process.py

Use `whl-dyn/tools/process.py` to process the collected data and generate visualization:

```bash
# Process all CSV files in a directory and generate visualization
python whl-dyn/tools/process.py -i data_directory/ -o results/

# Process all CSV files and also generate calibration table
python whl-dyn/tools/process.py -i data_directory/ -o results/ --output-calibration-table
```

**Parameter Description:**
- `-i`, `--input_dir`: Directory containing the raw CSV data logs (required)
- `-o`, `--output_dir`: Directory to save the final plots and table (default: `./calibration_results`)
- `--output-calibration-table`: Also output calibration table in protobuf or native format (optional)

The processing result will generate a `unified_calibration_table.csv` file containing the mapping relationship between speed, command, and acceleration, along with 3D surface and 2D contour plots. If the `--output-calibration-table` flag is specified, it will also generate a calibration table named `calibration_table.pb.txt` in protobuf format (if protobuf modules are available) or native format.

### Input/Output Format

**Input CSV Format:**
The script expects CSV files with the following columns:
- `time`: Timestamp (seconds)
- `speed_mps`: Vehicle speed (m/s)
- `imu_accel_y`: IMU Y-axis acceleration (m/s²)
- `ctl_throttle`: Control throttle command (%)
- `ctl_brake`: Control brake command (%)

**Output Format:**
The script generates:
1. `unified_calibration_table.csv`: A CSV file with columns `speed`, `command`, and `acceleration`
2. 3D surface plots and 2D contour plots for visualization
3. `calibration_table.pb.txt` (optional): A protobuf text format file containing calibration data with speed, acceleration, and command values (generated only when `--output-calibration-table` flag is specified)
