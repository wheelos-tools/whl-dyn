# Data Collector Commands

## Command Line Arguments

The data collector supports the following command line arguments:

```
usage: data_collector.py [-o OUTPUT_DIR] [-f COMMANDS_FILE]

Data Collector for Vehicle Dynamics Calibration

optional arguments:
  -o OUTPUT_DIR, --output-dir OUTPUT_DIR
                        Output directory for recorded data files (default: current directory)
  -f COMMANDS_FILE, --commands-file COMMANDS_FILE
                        File containing commands to execute in batch mode
```

## Interactive Mode Commands

When running in interactive mode, the following commands are available:

- `q` - Quit the program
- `p` - Plot results from the last run
- `x` - Remove results from the last run
- `x y z` - Execute a test run where:
  - `x` is the throttle value (positive number only)
  - `y` is the speed limit (positive number only)
  - `z` is the brake value (positive number only)

The system will automatically follow an accelerate-then-decelerate pattern:
1. First, it will apply the throttle to accelerate to the specified speed limit
2. Then, it will apply the brake to decelerate to a complete stop

All values must be positive numbers. The system handles the logic internally to ensure proper acceleration and deceleration phases.

## Designing Throttle/Brake Gradients for Better Calibration

To obtain high-quality vehicle dynamics calibration data, it's important to design your throttle and brake test cases strategically:

### Throttle Test Design
1. **Cover a wide range of throttle values**: Collect data across the full range of throttle commands your vehicle supports, from low to high values.
2. **Use multiple speed targets**: For each throttle value, test at different speed targets to capture the relationship between throttle, speed, and acceleration across the vehicle's operating range.
3. **Consider overlapping ranges**: Collect data with overlapping speed ranges to ensure smooth transitions in the calibration table.

### Brake Test Design
1. **Include various brake intensities**: Test with different brake values from light to heavy braking.
2. **Test from different initial speeds**: Apply brakes from various starting speeds to understand how deceleration varies with speed.
3. **Account for brake heating**: If conducting many brake tests in succession, allow time for brakes to cool to avoid thermal effects on the data.

### General Recommendations
1. **Dense sampling near critical points**: Collect more data points where the vehicle dynamics change rapidly (e.g., low speeds where friction effects are significant).
2. **Repeat tests for consistency**: Run the same test multiple times to verify data consistency and identify outliers.
3. **Cover extreme conditions**: Include tests at both ends of the throttle/brake range to ensure proper extrapolation behavior.
4. **Balance data collection**: Collect approximately equal amounts of acceleration and deceleration data for a balanced calibration model.

## Batch Mode

In batch mode, you can execute multiple test runs automatically by providing a commands file with the `-f` option.

The commands file should contain one command per line, with each command consisting of three values:
1. Throttle value (positive number only)
2. Speed limit (positive number only)
3. Brake value (positive number only)

Lines starting with `#` are treated as comments and ignored. Empty lines are also ignored.

### Example Commands File

```
# Throttle and brake tests (all values positive)
5.0 10.0 3.0
10.0 15.0 5.0
15.0 20.0 7.0

# Different combinations
3.0 10.0 5.0
5.0 15.0 7.0
7.0 20.0 10.0
```

### Running in Batch Mode

```bash
python data_collector.py -f commands.txt -o /path/to/output/directory
```

This will execute all commands in `commands.txt` and save the output files to the specified directory.

## File Naming Convention

Output files are now named using a simplified convention with clear separation between command parameters and run index:
- `t{throttle}_s{speed_limit}_b{brake}_run{index}_recorded.csv`

For example, a command `5.0 10.0 3.0` will generate a file named `t5_s10_b3_run0_recorded.csv`.

If the same command is run multiple times, the index will increment:
- First run: `t5_s10_b3_run0_recorded.csv`
- Second run: `t5_s10_b3_run1_recorded.csv`
- Third run: `t5_s10_b3_run2_recorded.csv`

This clear separation eliminates ambiguity between the brake value and the run index.