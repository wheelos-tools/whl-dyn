# process_and_visualize.py
import argparse
import glob
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import numpy as np
import pandas as pd
from scipy.interpolate import griddata
from scipy.signal import butter, filtfilt, correlate, correlation_lags
from sklearn.neighbors import LocalOutlierFactor

# Try to import protobuf modules for calibration table output
try:
    from modules.control.proto import calibration_table_pb2
    PROTOBUF_AVAILABLE = True
except ImportError:
    PROTOBUF_AVAILABLE = False
    print(
        "INFO: Protobuf modules not available. Calibration table output will use native format."
    )


# --- Configuration Block ---
class CalibrationConfig:
    """A single, centralized place for all tunable parameters."""
    # Signal Processing
    ACCEL_FILTER_ORDER = 6
    ACCEL_FILTER_CUTOFF_HZ = 1.0
    SAMPLING_RATE_HZ = 100.0

    # Outlier Detection (LOF)
    LOF_NEIGHBORS = 30
    LOF_CONTAMINATION = 0.02  # Expect ~2% of data to be outliers

    # Grid Generation for Final Table
    SPEED_GRID_RESOLUTION = 0.2  # m/s
    COMMAND_GRID_RESOLUTION = 5.0  # % command (-100 to +100)

    THROTTLE_LATENCY = 100  # throttle latency in ms
    BRAKE_LATENCY = 80  # brake latency in ms


class CalibrationProcessor:
    """A class to encapsulate the entire data processing and visualization workflow."""

    def __init__(self, config: CalibrationConfig):
        self.config = config
        self.unified_df = None
        self.unified_df_processed = None
        self.unified_table = None

    def run(self,
            input_dir: str,
            output_dir: str,
            output_calibration_table: bool = False):
        """Execute the full processing pipeline."""
        print("--- Starting Calibration Data Processing & Visualization ---")
        self._load_and_create_unified_df(input_dir)
        self._process_unified_data()
        self._build_unified_monotonic_table()
        self._visualize_and_save(output_dir, output_calibration_table)
        print("\n--- Processing Finished Successfully ---")

    def _load_and_create_unified_df(self, input_dir: str):
        """Loads all CSVs, merges them, and creates a unified command column."""
        print(f"INFO: Searching for *.csv files in '{input_dir}'...")
        csv_files = glob.glob(str(Path(input_dir) / "*.csv"))
        if not csv_files:
            raise FileNotFoundError(
                f"No CSV files found in directory: {input_dir}")

        all_dfs = []
        for file_path in csv_files:
            try:
                df = pd.read_csv(file_path,
                                 usecols=[
                                     'time', 'speed_mps', 'imu_accel_y',
                                     'ctl_throttle', 'ctl_brake'
                                 ])
                if not df.empty:
                    all_dfs.append(df)
            except Exception as e:
                print(
                    f"WARNING: Could not process file {file_path}. Reason: {e}"
                )

        if not all_dfs:
            raise ValueError(
                "No valid data loaded. Check CSV files for content and format."
            )

        # Create a single DataFrame
        master_df = pd.concat(
            all_dfs,
            ignore_index=True).sort_values(by='time').reset_index(drop=True)

        # --- Create the Unified Command Column ---
        # Throttle is positive, brake is negative
        # Handle cases where both throttle and brake might be applied
        master_df[
            'command'] = master_df['ctl_throttle'] - master_df['ctl_brake']
        self.unified_df = master_df.drop(columns=['ctl_throttle', 'ctl_brake'])
        print(
            f"OK: Loaded {len(csv_files)} files. Created unified dataset with {len(self.unified_df)} points."
        )

    def _process_unified_data(self):
        """Applies filtering, signal sync, and outlier removal to the unified dataset."""
        df = self.unified_df

        # 1. Denoise acceleration signal using low-pass filter
        nyquist = 0.5 * self.config.SAMPLING_RATE_HZ
        b, a = butter(self.config.ACCEL_FILTER_ORDER,
                      self.config.ACCEL_FILTER_CUTOFF_HZ / nyquist,
                      btype='low')
        print('before filtfilt')
        df['accel_filtered'] = filtfilt(b, a, df['imu_accel_y'])
        print('after filtfilt')

        # 2. Synchronize signals (must be done separately for throttle and brake)
        # This accounts for the delay between command input and acceleration response
        df_throttle = df[df['command'] > 0].copy()
        df_brake = df[df['command'] < 0].copy()

        if not df_throttle.empty:
            # TODO(leafyleong): Re-enable automatic synchronization
            # df_throttle = self._synchronize_segment(df_throttle, 'positive')
            df_throttle['accel_aligned'] = df_throttle['accel_filtered'].shift(
                0 -
                int(config.THROTTLE_LATENCY / 1000 * config.SAMPLING_RATE_HZ))
        if not df_brake.empty:
            # TODO(leafyleong): Re-enable automatic synchronization
            # df_brake = self._synchronize_segment(df_brake, 'negative')
            df_brake['accel_aligned'] = df_brake['accel_filtered'].shift(
                0 - int(config.BRAKE_LATENCY / 1000 * config.SAMPLING_RATE_HZ))

        df = pd.concat([df_throttle, df_brake],
                       ignore_index=True).sort_values(by='time')
        print("OK: Latency correction applied to throttle and brake segments.")

        # 3. Remove statistical outliers from the combined, latency-corrected data
        # Using Local Outlier Factor (LOF) algorithm for outlier detection
        features = ['speed_mps', 'command', 'accel_aligned']
        df_clean = df.dropna(subset=features).copy()
        # TODO(leafyleong): splite LOF removal for throttle and brake segments,
        # not combined.
        # lof = LocalOutlierFactor(n_neighbors=self.config.LOF_NEIGHBORS,
        #                          contamination=self.config.LOF_CONTAMINATION)
        # outlier_mask = lof.fit_predict(df_clean[features])
        # num_outliers = (outlier_mask == -1).sum()
        # print(
        #     f"INFO: Identified and removed {num_outliers} outliers using LOF.")
        # self.unified_df_processed = df_clean[outlier_mask == 1]
        self.unified_df_processed = df_clean

    def _synchronize_segment(self, df: pd.DataFrame,
                             command_type: str) -> pd.DataFrame:
        """Helper to synchronize a specific segment (throttle or brake)."""
        command_signal = df['command'].values
        accel_signal = df['accel_filtered'].values

        # For brake commands (negative values), we want to use absolute values for correlation
        if command_type == 'negative':
            command_signal = np.abs(command_signal)

        command_signal = (command_signal -
                          np.mean(command_signal)) / np.std(command_signal)
        accel_signal = (accel_signal -
                        np.mean(accel_signal)) / np.std(accel_signal)

        correlation = correlate(command_signal, accel_signal, mode='full')
        lags = correlation_lags(len(command_signal),
                                len(accel_signal),
                                mode='full')
        optimal_lag = lags[np.argmax(correlation)]
        print(
            f"INFO: Detected optimal lag for {command_type} commands: {optimal_lag} samples."
        )

        df['accel_aligned'] = df['accel_filtered'].shift(-optimal_lag)
        return df

    def _build_unified_monotonic_table(self):
        """Builds a single unified 2D lookup table and enforces monotonicity."""
        df = self.unified_df_processed
        points = df[['speed_mps', 'command']].values
        values = df['accel_aligned'].values

        # Create grid with unified command axis
        cmd_min, cmd_max = df['command'].min(), df['command'].max()
        speed_max = df['speed_mps'].max()
        speed_grid = np.arange(0, speed_max, self.config.SPEED_GRID_RESOLUTION)
        command_grid = np.arange(cmd_min,
                                 cmd_max + self.config.COMMAND_GRID_RESOLUTION,
                                 self.config.COMMAND_GRID_RESOLUTION)
        grid_x, grid_y = np.meshgrid(speed_grid, command_grid)

        grid_z = griddata(points,
                          values, (grid_x, grid_y),
                          method='cubic',
                          fill_value=0)

        # --- Enforce Monotonicity in Two Directions from Zero ---
        print(
            "INFO: Enforcing monotonicity constraint on the unified table...")
        # Find the index closest to zero command
        zero_cmd_idx = np.argmin(np.abs(command_grid))

        # For throttle (command > 0) - ensure acceleration increases or stays the same
        for i in range(zero_cmd_idx + 1, len(command_grid)):
            grid_z[i, :] = np.maximum(grid_z[i, :], grid_z[i - 1, :])

        # For brake (command < 0) - ensure acceleration decreases or stays the same
        for i in range(zero_cmd_idx - 1, -1, -1):
            grid_z[i, :] = np.minimum(grid_z[i, :], grid_z[i + 1, :])

        self.unified_table = (speed_grid, command_grid, grid_z)

    def _visualize_and_save(self,
                            output_dir: str,
                            output_calibration_table: bool = False):
        """Visualizes dynamics using 3D/2D plots and saves the final unified table."""
        Path(output_dir).mkdir(exist_ok=True)

        self.unified_df.to_csv(Path(output_dir) / 'unified_df.csv')
        self.unified_df_processed.to_csv(
            Path(output_dir) / 'unified_df_processed.csv')

        # Split the FINAL, CLEANED data for visualization purposes
        df_viz_throttle = self.unified_df_processed[
            self.unified_df_processed['command'] >= 0].copy()
        df_viz_brake = self.unified_df_processed[
            self.unified_df_processed['command'] < 0].copy()
        df_viz_brake['command'] = df_viz_brake['command'].abs(
        )  # Use positive command for brake plot axis

        # --- Save the Single, Unified Table ---
        filepath = Path(output_dir) / "unified_calibration_table.csv"
        speed_axis, command_axis, accel_values = self.unified_table
        with open(filepath, 'w') as f:
            f.write("speed,command,acceleration\n")
            for i, speed in enumerate(speed_axis):
                for j, command in enumerate(command_axis):
                    f.write(
                        f"{speed:.2f},{command:.2f},{accel_values[j, i]:.4f}\n"
                    )
        print(f"OK: Unified calibration table saved to '{filepath}'")

        # --- Create Plots ---
        throttle_cmap = LinearSegmentedColormap.from_list(
            "throttle_cmap", ["lightblue", "yellow", "red"])
        brake_cmap = LinearSegmentedColormap.from_list(
            "brake_cmap", ["lightgreen", "cyan", "mediumblue"])

        print("INFO: Generating visualization for throttle dynamics...")
        self._create_dynamics_plot(df_viz_throttle,
                                   'Throttle Dynamics',
                                   'Throttle Command (%)',
                                   cmap=throttle_cmap)

        print("INFO: Generating visualization for brake dynamics...")
        self._create_dynamics_plot(df_viz_brake,
                                   'Brake Dynamics',
                                   'Brake Command (%)',
                                   cmap=brake_cmap)

        # --- Generate Calibration Table ---
        if output_calibration_table:
            self._generate_calibration_table(output_dir)

    def _create_dynamics_plot(self, df_data, title, cmd_label, cmap):
        """Creates a combined 3D surface and 2D contour plot."""
        if df_data.empty:
            print(f"WARNING: {title} data is empty, skipping plot.")
            return

        grid_x, grid_y = np.mgrid[
            df_data['speed_mps'].min():df_data['speed_mps'].max():100j,
            df_data['command'].min():df_data['command'].max():100j]
        points = df_data[['speed_mps', 'command']].values
        values = df_data['accel_aligned'].values
        grid_z = griddata(points, values, (grid_x, grid_y), method='cubic')

        fig = plt.figure(figsize=(20, 9))
        fig.suptitle(title, fontsize=20, y=0.98)

        # 3D Surface Plot
        ax1 = fig.add_subplot(1, 2, 1, projection='3d')
        surf = ax1.plot_surface(grid_x,
                                grid_y,
                                grid_z,
                                cmap=cmap,
                                edgecolor='none',
                                alpha=0.8)
        ax1.scatter(df_data['speed_mps'],
                    df_data['command'],
                    df_data['accel_aligned'],
                    c='red',
                    s=40,
                    depthshade=True,
                    label='Cleaned Data Points')
        ax1.set_xlabel('Speed (m/s)', fontsize=12, labelpad=10)
        ax1.set_ylabel(cmd_label, fontsize=12, labelpad=10)
        ax1.set_zlabel('Acceleration (m/s^2)', fontsize=12, labelpad=10)
        ax1.set_title('3D Surface Plot', fontsize=16)
        ax1.view_init(elev=20, azim=-135)
        ax1.legend()
        fig.colorbar(surf, ax=ax1, shrink=0.5, aspect=10, pad=0.1)

        # 2D Contour Map
        ax2 = fig.add_subplot(1, 2, 2)
        contour = ax2.contourf(grid_x,
                               grid_y,
                               grid_z,
                               levels=15,
                               cmap=cmap,
                               alpha=0.9)
        clines = ax2.contour(grid_x,
                             grid_y,
                             grid_z,
                             levels=contour.levels,
                             colors='white',
                             linewidths=0.5)
        ax2.clabel(clines, inline=True, fontsize=8, fmt='%.1f')
        ax2.scatter(df_data['speed_mps'],
                    df_data['command'],
                    c='red',
                    s=40,
                    edgecolor='white',
                    label='Cleaned Data Points')
        ax2.set_xlabel('Speed (m/s)', fontsize=12)
        ax2.set_ylabel(cmd_label, fontsize=12)
        ax2.set_title('2D Contour Map (Top-Down View)', fontsize=16)
        ax2.legend()
        ax2.grid(True, linestyle='--', alpha=0.5)
        fig.colorbar(contour, ax=ax2, label='Acceleration ($m/s^2$)')

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        plt.show()

    def _generate_calibration_table(self, output_dir: str):
        """Generate calibration table in protobuf or native format."""
        print("INFO: Generating calibration table...")

        # Get the unified table data
        speed_axis, command_axis, accel_values = self.unified_table

        # Populate the calibration table
        # We need to convert the 2D grid data to individual calibration points
        calibration_entries = []
        for i, speed in enumerate(speed_axis):
            for j, command in enumerate(command_axis):
                acceleration = accel_values[j, i]

                # Skip zero acceleration values to reduce file size and match typical format
                if abs(acceleration) < 0.001:
                    continue

                # Store calibration entry in the correct format: speed, acceleration, command
                calibration_entries.append({
                    'speed': float(speed),
                    'acceleration': float(acceleration),
                    'command': float(command)
                })

        # Save the calibration table to a file with fixed name
        filepath = Path(output_dir) / "calibration_table.pb.txt"

        # If protobuf is available, use it; otherwise, use native format
        if PROTOBUF_AVAILABLE:
            # Create the calibration table protobuf message
            calibration_table_pb = calibration_table_pb2.ControlCalibrationTable(
            )

            # Populate protobuf message
            count = 0
            for entry in calibration_entries:
                calibration_entry = calibration_table_pb.calibration.add()
                calibration_entry.speed = entry['speed']
                calibration_entry.acceleration = entry['acceleration']
                calibration_entry.command = entry['command']
                count += 1

            # Save using protobuf
            with open(filepath, 'w') as f:
                f.write(str(calibration_table_pb))

            print(
                f"OK: Calibration table (protobuf format) saved to '{filepath}' with {count} entries."
            )
        else:
            # Use native format matching protobuf text format
            with open(filepath, 'w') as f:
                for entry in calibration_entries:
                    f.write("calibration {\n")
                    f.write(f"  speed: {entry['speed']}\n")
                    f.write(f"  acceleration: {entry['acceleration']}\n")
                    f.write(f"  command: {entry['command']}\n")
                    f.write("}\n")

            print(
                f"OK: Calibration table (native format) saved to '{filepath}' with {len(calibration_entries)} entries."
            )


def main():
    parser = argparse.ArgumentParser(
        description=
        "Process and visualize raw vehicle data to generate a unified calibration table."
    )
    parser.add_argument("-i",
                        "--input_dir",
                        type=str,
                        default="./calibration_data_logs",
                        help="Directory containing the raw CSV data logs.")
    parser.add_argument("-o",
                        "--output_dir",
                        type=str,
                        default="./calibration_results",
                        help="Directory to save the final plots and table.")
    parser.add_argument(
        "--output-calibration-table",
        action="store_true",
        help="Also output calibration table in protobuf or native format.")
    args = parser.parse_args()

    config = CalibrationConfig()
    processor = CalibrationProcessor(config)

    try:
        processor.run(args.input_dir, args.output_dir,
                      args.output_calibration_table)
    except Exception as e:
        print(
            f"\nFATAL ERROR: An unexpected error occurred during processing: {e}",
            file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
