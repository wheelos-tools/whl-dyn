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

# --- Configuration Block ---
class CalibrationConfig:
    """A single, centralized place for all tunable parameters."""
    # Signal Processing
    ACCEL_FILTER_ORDER = 4
    ACCEL_FILTER_CUTOFF_HZ = 2.0
    SAMPLING_RATE_HZ = 100.0

    # Outlier Detection (LOF)
    LOF_NEIGHBORS = 30
    LOF_CONTAMINATION = 0.02  # Expect ~2% of data to be outliers

    # Grid Generation for Final Table
    SPEED_GRID_RESOLUTION = 0.5  # m/s
    COMMAND_GRID_RESOLUTION = 2.0  # % command (-100 to +100)

class CalibrationProcessor:
    """A class to encapsulate the entire data processing and visualization workflow."""

    def __init__(self, config: CalibrationConfig):
        self.config = config
        self.unified_df = None
        self.unified_table = None

    def run(self, input_dir: str, output_dir: str):
        """Execute the full processing pipeline."""
        print("--- Starting Calibration Data Processing & Visualization ---")
        self._load_and_create_unified_df(input_dir)
        self._process_unified_data()
        self._build_unified_monotonic_table()
        self._visualize_and_save(output_dir)
        print("\n--- Processing Finished Successfully ---")

    def _load_and_create_unified_df(self, input_dir: str):
        """Loads all CSVs, merges them, and creates a unified command column."""
        print(f"INFO: Searching for *.csv files in '{input_dir}'...")
        csv_files = glob.glob(str(Path(input_dir) / "*.csv"))
        if not csv_files:
            raise FileNotFoundError(f"No CSV files found in directory: {input_dir}")

        all_dfs = []
        for file_path in csv_files:
            try:
                df = pd.read_csv(file_path, usecols=['time', 'speed_mps', 'imu_accel_y', 'ctl_throttle', 'ctl_brake'])
                if not df.empty:
                    all_dfs.append(df)
            except Exception as e:
                print(f"WARNING: Could not process file {file_path}. Reason: {e}")

        if not all_dfs:
            raise ValueError("No valid data loaded. Check CSV files for content and format.")

        # Create a single DataFrame
        master_df = pd.concat(all_dfs, ignore_index=True).sort_values(by='time').reset_index(drop=True)

        # --- Create the Unified Command Column ---
        # Throttle is positive, brake is negative
        master_df['command'] = np.where(master_df['ctl_throttle'] > 0, master_df['ctl_throttle'], -master_df['ctl_brake'])
        self.unified_df = master_df.drop(columns=['ctl_throttle', 'ctl_brake'])
        print(f"OK: Loaded {len(csv_files)} files. Created unified dataset with {len(self.unified_df)} points.")

    def _process_unified_data(self):
        """Applies filtering, signal sync, and outlier removal to the unified dataset."""
        df = self.unified_df

        # 1. Denoise acceleration signal
        nyquist = 0.5 * self.config.SAMPLING_RATE_HZ
        b, a = butter(self.config.ACCEL_FILTER_ORDER, self.config.ACCEL_FILTER_CUTOFF_HZ / nyquist, btype='low')
        df['accel_filtered'] = filtfilt(b, a, df['imu_accel_y'])

        # 2. Synchronize signals (must be done separately for throttle and brake)
        df_throttle = df[df['command'] > 0].copy()
        df_brake = df[df['command'] < 0].copy()

        if not df_throttle.empty:
            df_throttle = self._synchronize_segment(df_throttle, 'positive')
        if not df_brake.empty:
            df_brake = self._synchronize_segment(df_brake, 'negative')

        df = pd.concat([df_throttle, df_brake], ignore_index=True).sort_values(by='time')
        print("OK: Latency correction applied to throttle and brake segments.")

        # 3. Remove statistical outliers from the combined, latency-corrected data
        features = ['speed_mps', 'command', 'accel_aligned']
        df_clean = df.dropna(subset=features).copy()
        lof = LocalOutlierFactor(n_neighbors=self.config.LOF_NEIGHBORS, contamination=self.config.LOF_CONTAMINATION)
        outlier_mask = lof.fit_predict(df_clean[features])
        num_outliers = (outlier_mask == -1).sum()
        print(f"INFO: Identified and removed {num_outliers} outliers using LOF.")
        self.unified_df = df_clean[outlier_mask == 1]

    def _synchronize_segment(self, df: pd.DataFrame, command_type: str) -> pd.DataFrame:
        """Helper to synchronize a specific segment (throttle or brake)."""
        command_signal = df['command'].abs().values
        accel_signal = df['accel_filtered'].values

        command_signal = (command_signal - np.mean(command_signal)) / np.std(command_signal)
        accel_signal = (accel_signal - np.mean(accel_signal)) / np.std(accel_signal)

        correlation = correlate(command_signal, accel_signal, mode='full')
        lags = correlation_lags(len(command_signal), len(accel_signal), mode='full')
        optimal_lag = lags[np.argmax(correlation)]
        print(f"INFO: Detected optimal lag for {command_type} commands: {optimal_lag} samples.")

        df['accel_aligned'] = df['accel_filtered'].shift(-optimal_lag)
        return df

    def _build_unified_monotonic_table(self):
        """Builds a single unified 2D lookup table and enforces monotonicity."""
        df = self.unified_df
        points = df[['speed_mps', 'command']].values
        values = df['accel_aligned'].values

        # Create grid with unified command axis
        cmd_min, cmd_max = df['command'].min(), df['command'].max()
        speed_max = df['speed_mps'].max()
        speed_grid = np.arange(0, speed_max + self.config.SPEED_GRID_RESOLUTION, self.config.SPEED_GRID_RESOLUTION)
        command_grid = np.arange(cmd_min, cmd_max + self.config.COMMAND_GRID_RESOLUTION, self.config.COMMAND_GRID_RESOLUTION)
        grid_x, grid_y = np.meshgrid(speed_grid, command_grid)

        grid_z = griddata(points, values, (grid_x, grid_y), method='cubic', fill_value=0)

        # --- Enforce Monotonicity in Two Directions from Zero ---
        print("INFO: Enforcing monotonicity constraint on the unified table...")
        zero_cmd_idx = np.argmin(np.abs(command_grid))

        # For throttle (command > 0)
        for i in range(zero_cmd_idx + 1, len(command_grid)):
            grid_z[i, :] = np.maximum(grid_z[i, :], grid_z[i-1, :])

        # For brake (command < 0)
        for i in range(zero_cmd_idx - 1, -1, -1):
            grid_z[i, :] = np.minimum(grid_z[i, :], grid_z[i+1, :])

        self.unified_table = (speed_grid, command_grid, grid_z)

    def _visualize_and_save(self, output_dir: str):
        """Visualizes dynamics using 3D/2D plots and saves the final unified table."""
        Path(output_dir).mkdir(exist_ok=True)

        # Split the FINAL, CLEANED data for visualization purposes
        df_viz_throttle = self.unified_df[self.unified_df['command'] >= 0].copy()
        df_viz_brake = self.unified_df[self.unified_df['command'] < 0].copy()
        df_viz_brake['command'] = df_viz_brake['command'].abs() # Use positive command for brake plot axis

        # --- Create Plots ---
        throttle_cmap = LinearSegmentedColormap.from_list("throttle_cmap", ["lightblue", "yellow", "red"])
        brake_cmap = LinearSegmentedColormap.from_list("brake_cmap", ["lightgreen", "cyan", "mediumblue"])

        print("INFO: Generating visualization for throttle dynamics...")
        self._create_dynamics_plot(df_viz_throttle, 'Throttle Dynamics', 'Throttle Command (%)', cmap=throttle_cmap)

        print("INFO: Generating visualization for brake dynamics...")
        self._create_dynamics_plot(df_viz_brake, 'Brake Dynamics', 'Brake Command (%)', cmap=brake_cmap)

        # --- Save the Single, Unified Table ---
        filepath = Path(output_dir) / "unified_calibration_table.csv"
        speed_axis, command_axis, accel_values = self.unified_table
        with open(filepath, 'w') as f:
            f.write("speed,command,acceleration\n")
            for i, speed in enumerate(speed_axis):
                for j, command in enumerate(command_axis):
                    f.write(f"{speed:.2f},{command:.2f},{accel_values[j, i]:.4f}\n")
        print(f"OK: Unified calibration table saved to '{filepath}'")

    def _create_dynamics_plot(self, df_data, title, cmd_label, cmap):
        """Creates a combined 3D surface and 2D contour plot."""
        if df_data.empty:
            print(f"WARNING: {title} data is empty, skipping plot.")
            return

        grid_x, grid_y = np.mgrid[df_data['speed_mps'].min():df_data['speed_mps'].max():100j,
                                  df_data['command'].min():df_data['command'].max():100j]
        points = df_data[['speed_mps', 'command']].values
        values = df_data['accel_aligned'].values
        grid_z = griddata(points, values, (grid_x, grid_y), method='cubic')

        fig = plt.figure(figsize=(20, 9)); fig.suptitle(title, fontsize=20, y=0.98)

        # 3D Surface Plot
        ax1 = fig.add_subplot(1, 2, 1, projection='3d')
        surf = ax1.plot_surface(grid_x, grid_y, grid_z, cmap=cmap, edgecolor='none', alpha=0.8)
        ax1.scatter(df_data['speed_mps'], df_data['command'], df_data['accel_aligned'], c='red', s=40, depthshade=True, label='Cleaned Data Points')
        ax1.set_xlabel('Speed (m/s)', fontsize=12, labelpad=10)
        ax1.set_ylabel(cmd_label, fontsize=12, labelpad=10)
        ax1.set_zlabel('Acceleration (m/s^2)', fontsize=12, labelpad=10)
        ax1.set_title('3D Surface Plot', fontsize=16)
        ax1.view_init(elev=20, azim=-135); ax1.legend()
        fig.colorbar(surf, ax=ax1, shrink=0.5, aspect=10, pad=0.1)

        # 2D Contour Map
        ax2 = fig.add_subplot(1, 2, 2)
        contour = ax2.contourf(grid_x, grid_y, grid_z, levels=15, cmap=cmap, alpha=0.9)
        clines = ax2.contour(grid_x, grid_y, grid_z, levels=contour.levels, colors='white', linewidths=0.5)
        ax2.clabel(clines, inline=True, fontsize=8, fmt='%.1f')
        ax2.scatter(df_data['speed_mps'], df_data['command'], c='red', s=40, edgecolor='white', label='Cleaned Data Points')
        ax2.set_xlabel('Speed (m/s)', fontsize=12)
        ax2.set_ylabel(cmd_label, fontsize=12)
        ax2.set_title('2D Contour Map (Top-Down View)', fontsize=16)
        ax2.legend(); ax2.grid(True, linestyle='--', alpha=0.5)
        fig.colorbar(contour, ax=ax2, label='Acceleration ($m/s^2$)')

        plt.tight_layout(rect=[0, 0, 1, 0.95]); plt.show()

def main():
    parser = argparse.ArgumentParser(description="Process and visualize raw vehicle data to generate a unified calibration table.")
    parser.add_argument("-i", "--input_dir", type=str, required=True, help="Directory containing the raw CSV data logs.")
    parser.add_argument("-o", "--output_dir", type=str, default="./calibration_results", help="Directory to save the final plots and table.")
    args = parser.parse_args()

    config = CalibrationConfig()
    processor = CalibrationProcessor(config)

    try:
        processor.run(args.input_dir, args.output_dir)
    except Exception as e:
        print(f"\nFATAL ERROR: An unexpected error occurred during processing: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
