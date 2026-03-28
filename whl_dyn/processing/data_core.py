import os
import glob
from pathlib import Path
import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt
from scipy.interpolate import griddata
from sklearn.neighbors import LocalOutlierFactor
from .config import CalibrationConfig

class DataCore:
    """Core data processing class handling filtering, derivation, and grid building."""

    def __init__(self, config: CalibrationConfig):
        self.config = config
        self.raw_dfs = []
        self.unified_df = None
        self.processed_df = None

    def load_data(self, input_dir):
        """Load and unify all CSV logs."""
        csv_files = glob.glob(str(Path(input_dir) / "*.csv"))
        for file_path in csv_files:
            try:
                df = pd.read_csv(file_path)
                if not df.empty:
                    # Select appropriate feature columns based on config
                    speed_col = 'speed_mps' if self.config.speed_source == 'chassis' else 'ins_speed_mps'

                    if speed_col not in df.columns:
                        # Fallback for old datasets if ins_speed_mps not found
                        speed_col = 'speed_mps'

                    df['final_speed'] = df[speed_col]
                    df['command'] = df['ctl_throttle'] - df['ctl_brake']
                    df['source_file'] = os.path.basename(file_path)

                    # Acceleration source
                    if self.config.accel_source == 'derivative':
                        dt = df['time'].diff().fillna(1.0 / self.config.sampling_rate)
                        df['raw_accel'] = df['final_speed'].diff() / dt
                        df['raw_accel'].bfill(inplace=True)
                    else:
                        df['raw_accel'] = df['imu_accel_y']

                    self.raw_dfs.append(df)
            except Exception as e:
                print(f"Skipping {file_path}: {e}")

        if self.raw_dfs:
            self.unified_df = pd.concat(self.raw_dfs, ignore_index=True).sort_values(by='time').reset_index(drop=True)

    def process_signals(self):
        """Apply Butterworth filtering, latency sync, and LOF."""
        if self.unified_df is None or self.unified_df.empty: return
        df = self.unified_df.copy()

        df['command_type'] = np.where(df['command'] > 0, 'throttle', np.where(df['command'] < 0, 'brake', 'coast'))

        # 1. Low-pass filter
        nyquist = 0.5 * self.config.sampling_rate
        norm_cutoff = max(0.01, min(self.config.lowpass_cutoff / nyquist, 0.99))
        b, a = butter(self.config.lowpass_order, norm_cutoff, btype='low')
        df['accel_filtered'] = filtfilt(b, a, df['raw_accel'])

        # Store filtered to raw too for step plot later
        for i in range(len(self.raw_dfs)):
            self.raw_dfs[i]['accel_filtered'] = filtfilt(b, a, self.raw_dfs[i]['raw_accel'])

        # 2. Sync latency
        df_throttle = df[df['command'] > 0].copy()
        df_brake = df[df['command'] < 0].copy()
        df_zero = df[df['command'] == 0].copy()

        t_shift = int(self.config.throttle_latency_ms / 1000.0 * self.config.sampling_rate)
        b_shift = int(self.config.brake_latency_ms / 1000.0 * self.config.sampling_rate)

        if not df_throttle.empty: df_throttle['accel_aligned'] = df_throttle['accel_filtered'].shift(-t_shift).fillna(0)
        if not df_brake.empty: df_brake['accel_aligned'] = df_brake['accel_filtered'].shift(-b_shift).fillna(0)
        if not df_zero.empty: df_zero['accel_aligned'] = df_zero['accel_filtered']

        df = pd.concat([df_throttle, df_brake, df_zero], ignore_index=True)
        df['aligned_speed'] = df['final_speed']
        df['is_outlier'] = False
        features = ['final_speed', 'command', 'accel_aligned']
        df_clean = df.dropna(subset=features).copy()

        # 2.5 Stability window filtering (remove transient state after command switch)
        df_clean = self._apply_stability_filter(df_clean)

        # 3. LOF
        if self.config.enable_lof:
            df_t = df_clean[df_clean['command'] > 0].copy()
            df_b = df_clean[df_clean['command'] < 0].copy()
            df_z = df_clean[df_clean['command'] == 0].copy()

            if len(df_t) >= self.config.lof_neighbors:
                lof_t = LocalOutlierFactor(
                    n_neighbors=self.config.lof_neighbors,
                    contamination=self.config.lof_contamination)
                mask_t = lof_t.fit_predict(df_t[features])
                df_t['is_outlier'] = (mask_t == -1)
                df_t = df_t[df_t['is_outlier'] == False]
            if len(df_b) >= self.config.lof_neighbors:
                lof_b = LocalOutlierFactor(
                    n_neighbors=self.config.lof_neighbors,
                    contamination=self.config.lof_contamination)
                mask_b = lof_b.fit_predict(df_b[features])
                df_b['is_outlier'] = (mask_b == -1)
                df_b = df_b[df_b['is_outlier'] == False]

            self.processed_df = pd.concat([df_t, df_b, df_z], ignore_index=True)
        else:
            self.processed_df = df_clean

        if self.processed_df is not None and not self.processed_df.empty:
            if 'aligned_speed' not in self.processed_df.columns:
                self.processed_df['aligned_speed'] = self.processed_df['final_speed']
            if 'is_outlier' not in self.processed_df.columns:
                self.processed_df['is_outlier'] = False
            if 'command_type' not in self.processed_df.columns:
                self.processed_df['command_type'] = np.where(
                    self.processed_df['command'] > 0,
                    'throttle',
                    np.where(self.processed_df['command'] < 0, 'brake', 'coast'))

    def build_calibration_table(self):
        """Generate monotonically enforced lookup table grid."""
        if self.processed_df is None or self.processed_df.empty: return None, None, None

        points = self.processed_df[['final_speed', 'command']].values
        values = self.processed_df['accel_aligned'].values

        cmd_min, cmd_max = self.processed_df['command'].min(), self.processed_df['command'].max()
        speed_max = self.processed_df['final_speed'].max()

        speed_grid = np.arange(0, speed_max, self.config.speed_resolution)
        command_grid = np.arange(cmd_min, cmd_max + self.config.command_resolution, self.config.command_resolution)
        grid_x, grid_y = np.meshgrid(speed_grid, command_grid)

        grid_z = griddata(points, values, (grid_x, grid_y), method='linear', fill_value=0)

        # In case some edges are still nan because interpolation convex hull missing
        grid_z = np.nan_to_num(grid_z)

        # Enforce physical constraints / monotonicity
        zero_cmd_idx = np.argmin(np.abs(command_grid))
        grid_z[zero_cmd_idx, :] = 0.0

        for i in range(zero_cmd_idx + 1, len(command_grid)):
            grid_z[i, :] = np.maximum(grid_z[i, :], grid_z[i - 1, :])
            grid_z[i, :] = np.maximum(grid_z[i, :], 0.0)

        for i in range(zero_cmd_idx - 1, -1, -1):
            grid_z[i, :] = np.minimum(grid_z[i, :], grid_z[i + 1, :])
            grid_z[i, :] = np.minimum(grid_z[i, :], 0.0)

        # Optional extra 1D smoothing along command axis could be added here

        return speed_grid, command_grid, grid_z

    def _apply_stability_filter(self, df):
        """Filter out transient state data after command switches."""
        if df is None or df.empty:
            return df

        # Calculate stability window in samples
        t_window_samples = int(self.config.throttle_stability_window_ms / 1000.0 * self.config.sampling_rate)
        b_window_samples = int(self.config.brake_stability_window_ms / 1000.0 * self.config.sampling_rate)

        # Group by source file to detect command switches within each file
        filtered_dfs = []
        for source_file in df['source_file'].unique():
            file_df = df[df['source_file'] == source_file].copy()

            if len(file_df) == 0:
                continue

            # Detect command switch points
            file_df = file_df.reset_index(drop=True)
            file_df['command_changed'] = file_df['command'].diff().abs() > 1e-6

            # Mark samples within stability window after command switch
            mask_keep = pd.Series([True] * len(file_df), index=file_df.index)

            for idx in file_df[file_df['command_changed']].index:
                # Get the command type at this switch point
                cmd = file_df.loc[idx, 'command']
                window = b_window_samples if cmd < 0 else t_window_samples

                # Mark samples within the window to be discarded
                end_idx = min(len(file_df), idx + window + 1)
                mask_keep.iloc[idx:end_idx] = False

            # Apply the mask
            file_df_filtered = file_df[mask_keep].copy()

            # For throttle: also filter out very low speeds
            throttle_data = file_df_filtered[file_df_filtered['command'] > 0].copy()
            if len(throttle_data) > 0:
                throttle_data = throttle_data[throttle_data['final_speed'] >= self.config.min_throttle_speed_mps]

            # Keep non-throttle data as is
            non_throttle_data = file_df_filtered[file_df_filtered['command'] <= 0].copy()

            filtered_dfs.append(pd.concat([throttle_data, non_throttle_data], ignore_index=True))

        if filtered_dfs:
            result = pd.concat(filtered_dfs, ignore_index=True)
            # Drop the temporary column
            if 'command_changed' in result.columns:
                result = result.drop(columns=['command_changed'])
            return result
        return df