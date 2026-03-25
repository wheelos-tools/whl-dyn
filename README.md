# Vehicle Dynamics Calibration Tools

## 1. Overview

This toolset provides a complete solution for vehicle dynamics calibration, including calibration plan generation, automatic data collection, data processing, and result visualization. With this toolset, vehicle longitudinal dynamics characteristics can be efficiently calibrated.

### 1.1 Tool Components

- **whl-dyn/tools/generate_plan.py**: Generate vehicle dynamics calibration plans
- **whl-dyn/tools/collect_data.py**: Automatically collect data according to calibration plans
- **whl-dyn/tools/process.py**: Process collected data and generate calibration tables with visualization, including optional protobuf format output
- **whl-dyn/tools/plot.py**: Visualize calibration results

### 1.2 Workflow

```
Generate Calibration Plan → Automatic Data Collection → Data Processing → Result Visualization
(whl-dyn/tools/generate_plan.py) (whl-dyn/tools/collect_data.py) (whl-dyn/tools/process.py) (whl-dyn/tools/plot.py)
```

## 2. Installation and Setup

### 2.1 Dependencies

Ensure the following dependencies are installed:

```bash
# Python dependencies
pip install numpy scipy matplotlib pandas pyyaml

# Apollo dependencies (if automatic data collection is needed)
# Apollo CyberRT environment needs to be configured
```

### 2.2 Environment Configuration

1. Clone the repository
2. Install Python dependencies
3. Configure Apollo environment (if automatic data collection is needed)

## 3. Usage Workflow

### 3.1 Generate Calibration Plan

Use `generate_plan.py` to generate YAML format calibration plans:

```bash
# Generate default calibration plan
python whl-dyn/tools/generate_plan.py

# Generate custom calibration plan
python whl-dyn/tools/generate_plan.py \
  --throttle-min 10 --throttle-max 80 --throttle-num-steps 8 \
  --brake-min 10 --brake-max 50 --brake-num-steps 5 \
  --speed-targets 1.0 3.0 5.0 7.0 \
  -o my_calibration_plan.yaml
```

**Parameter Description:**
- `--throttle-min`, `--throttle-max`: Throttle test range (%)
- `--throttle-num-steps`: Number of throttle test steps
- `--brake-min`, `--brake-max`: Brake test range (%)
- `--brake-num-steps`: Number of brake test steps
- `--speed-targets`: Target speeds for acceleration tests (m/s)
- `-o`: Output file name

### 3.2 Data Collection

Use `collect_data.py` to automatically collect data according to the calibration plan:

```bash
# Use default plan file and output directory
python whl-dyn/tools/collect_data.py

# Use custom plan file
python whl-dyn/tools/collect_data.py -p my_calibration_plan.yaml

# Use custom output directory
python whl-dyn/tools/collect_data.py -o /path/to/output/directory

# Specify both plan file and output directory
python whl-dyn/tools/collect_data.py -p my_calibration_plan.yaml -o /path/to/output/directory
```

Collected data will be saved in the specified output directory, with each test case generating a CSV file.

### 3.3 Data Processing

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

### 3.4 Result Visualization

Use `whl-dyn/tools/plot.py` to visualize calibration results:

```bash
# Use default input file
python whl-dyn/tools/plot.py

# Use custom input file
python whl-dyn/tools/plot.py -i calibration_data.txt
```

The visualization tool will generate 3D surface plots and 2D contour plots for both throttle and brake dynamics.

## 4. Safety Operation Guidelines

### 4.1 On-site Operation Standards

1. **Pre-test Check**
   - Ensure the test area is safe, with no personnel or obstacles
   - Check vehicle status and ensure sufficient battery power
   - Confirm the vehicle is in good working condition
   - Check that sensors and control systems are working properly

2. **Operator Requirements**
   - Operators must be professionally trained
   - A safety supervisor must be present on site
   - Operators should be familiar with emergency stop procedures

3. **Test Environment Requirements**
   - Choose a flat, spacious test area
   - Ensure the surface friction coefficient is stable
   - Avoid testing in adverse weather conditions

### 4.2 Safety Considerations

1. **Emergency Handling**
   - Press the emergency stop button immediately if abnormalities are detected during testing
   - Maintain a safe distance from the vehicle
   - Have manual control equipment ready as backup

2. **Parameter Setting Safety**
   - Use lower throttle and brake values for initial tests
   - Gradually increase test intensity
   - Avoid setting excessively high target speeds

3. **Equipment Safety**
   - Regularly check sensors and actuators
   - Ensure communication links are stable
   - Backup important data

## 5. Troubleshooting

### 5.1 Common Issues

1. **Unable to Connect to Vehicle**
   - Check network connection
   - Confirm CyberRT service is running
   - Check firewall settings

2. **Data Collection Failure**
   - Check if sensor data is normal
   - Confirm the vehicle is in the correct driving mode
   - Check output directory permissions

3. **Abnormal Processing Results**
   - Check input data quality
   - Confirm parameter settings are reasonable
   - Check log files for detailed information

## 6. Input/Output File Format Specification

### 6.1 Calibration Plan File (YAML Format)

The calibration plan file defines test cases and steps in the following format:

```yaml
- case_name: "throttle_30_to_5mps"
  description: "Accelerate with 30% throttle, target >5m/s, then brake to stop."
  steps:
    - command:
        throttle: 30.0
        brake: 0.0
      trigger:
        type: "speed_greater_than"
        value: 5.0
      timeout_sec: 30.0
    - command:
        throttle: 0.0
        brake: 30.0
      trigger:
        type: "speed_less_than"
        value: 0.1
      timeout_sec: 30.0

- case_name: "brake_20_from_5mps"
  description: "Accelerate to >5m/s, then apply 20% brake."
  steps:
    - command:
        throttle: 80.0
        brake: 0.0
      trigger:
        type: "speed_greater_than"
        value: 5.0
      timeout_sec: 30.0
    - command:
        throttle: 0.0
        brake: 20.0
      trigger:
        type: "speed_less_than"
        value: 0.1
      timeout_sec: 30.0
```

**Field Description:**
- `case_name`: Test case name
- `description`: Test case description
- `steps`: List of test steps
  - `command`: Control command
    - `throttle`: Throttle command (%)
    - `brake`: Brake command (%)
  - `trigger`: Trigger condition
    - `type`: Trigger type (`speed_greater_than` or `speed_less_than`)
    - `value`: Trigger value (m/s)
  - `timeout_sec`: Step timeout (seconds)

### 6.2 Data Log Files (CSV Format)

Data log files record vehicle status information during testing in the following format:

```
time,speed_mps,imu_accel_y,driving_mode,actual_gear,throttle_pct,brake_pct,ctl_throttle,ctl_brake
123456.7890,2.5,0.3,1,1,25.0,0.0,30.0,0.0
123456.7990,2.6,0.4,1,1,25.0,0.0,30.0,0.0
...
```

**Field Description:**
- `time`: Timestamp (seconds)
- `speed_mps`: Vehicle speed (m/s)
- `imu_accel_y`: IMU Y-axis acceleration (m/s²)
- `driving_mode`: Driving mode
- `actual_gear`: Actual gear
- `throttle_pct`: Actual throttle percentage
- `brake_pct`: Actual brake percentage
- `ctl_throttle`: Control throttle command (%)
- `ctl_brake`: Control brake command (%)

### 6.3 Processing Result Files (CSV Format)

Processing result files contain the mapping relationship between speed, command, and acceleration:

```
speed,command,acceleration
0.00,0.00,0.0000
0.00,2.00,0.0000
0.00,4.00,0.0000
...
```

**Field Description:**
- `speed`: Speed value (m/s)
- `command`: Command value (positive for throttle, negative for brake)
- `acceleration`: Acceleration value (m/s²)

Note: The `whl-dyn/tools/process.py` script generates a file named `unified_calibration_table.csv` with a header row and three columns representing speed, command, and acceleration values respectively.

### 6.4 Calibration Table Files (Protocol Buffer Text Format)

Calibration table files contain the final calibration data:

```
calibration_table {
  calibration {
    speed: 0.10000000149011612
    acceleration: -3.085177183151245
    command: -50.599998474121094
  }
  calibration {
    speed: 0.10000000149011612
    acceleration: -3.0587549209594727
    command: -48.042103817588405
  }
  ...
}
```

**Field Description:**
- `speed`: Speed value (m/s)
- `acceleration`: Acceleration value (m/s²)
- `command`: Command value (positive for throttle, negative for brake)

## 7. Appendix

### 7.1 Command Line Parameter Reference

#### generate_plan.py
```
usage: generate_plan.py [-h] [-o OUTPUT] [--throttle-min THROTTLE_MIN]
                        [--throttle-max THROTTLE_MAX]
                        [--throttle-num-steps THROTTLE_NUM_STEPS]
                        [--brake-min BRAKE_MIN] [--brake-max BRAKE_MAX]
                        [--brake-num-steps BRAKE_NUM_STEPS]
                        [--speed-targets SPEED_TARGETS [SPEED_TARGETS ...]]
                        [--default-brake DEFAULT_BRAKE]
                        [--accel-timeout ACCEL_TIMEOUT]
                        [--decel-timeout DECEL_TIMEOUT]

Generate a YAML plan for vehicle longitudinal calibration.

optional arguments:
  -h, --help            show this help message and exit
  -o OUTPUT, --output OUTPUT
                        Output YAML file name.
  --throttle-min THROTTLE_MIN
                        Minimum throttle command (%) to test.
  --throttle-max THROTTLE_MAX
                        Maximum throttle command (%) to test.
  --throttle-num-steps THROTTLE_NUM_STEPS
                        Number of throttle steps to generate.
  --brake-min BRAKE_MIN
                        Minimum brake command (%) to test.
  --brake-max BRAKE_MAX
                        Maximum brake command (%) to test.
  --brake-num-steps BRAKE_NUM_STEPS
                        Number of brake steps to generate.
  --speed-targets SPEED_TARGETS [SPEED_TARGETS ...]
                        List of target speeds (m/s) for acceleration tests.
  --default-brake DEFAULT_BRAKE
                        Default brake command (%) used to stop the vehicle
                        after a test step.
  --accel-timeout ACCEL_TIMEOUT
                        Timeout in seconds for acceleration steps.
  --decel-timeout DECEL_TIMEOUT
                        Timeout in seconds for deceleration steps.
```

#### collect_data.py
```
usage: collect_data.py [-h] [-p PLAN] [-o OUTPUT_DIR]

Production-Ready, Plan-Driven Data Collector for Apollo.

optional arguments:
  -h, --help            show this help message and exit
  -p PLAN, --plan PLAN  Path to the YAML calibration plan file (default:
                        calibration_plan.yaml)
  -o OUTPUT_DIR, --output-dir OUTPUT_DIR
                        Output directory for collected data files (default:
                        ./calibration_data_logs)
```

#### tools/process.py
```
usage: process.py [-h] [-i INPUT_DIR] [-o OUTPUT_DIR] [--output-calibration-table]

Process and visualize raw vehicle data to generate a unified calibration table.

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT_DIR, --input-dir INPUT_DIR
                        Directory containing the raw CSV data logs.
  -o OUTPUT_DIR, --output-dir OUTPUT_DIR
                        Directory to save the final plots and table (default:
                        ./calibration_results)
  --output-calibration-table
                        Also output calibration table in protobuf or native
                        format.
```

#### tools/plot.py
```
usage: plot.py [-h] [-i INPUT]

Plot vehicle dynamics calibration data.

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT, --input INPUT
                        Input calibration data file (default:
                        calibration_data.txt)
```