import os
import glob
from pathlib import Path
import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt
from scipy.interpolate import griddata
from sklearn.neighbors import LocalOutlierFactor
from .config import CalibrationConfig
from .dynamics import is_dynamic_log

class DataCore:
    """Core data processing class handling filtering, derivation, and grid building."""

    def __init__(self, config: CalibrationConfig):
        self.config = config
        self.raw_dfs = []
        self.unified_df = None
        self.processed_df = None

    def _sampling_rate(self, frame):
        """Return the measured sampling rate for one recording frame."""
        if 'sample_rate_hz' in frame:
            measured = pd.to_numeric(
                frame['sample_rate_hz'], errors='coerce').dropna()
            if not measured.empty and measured.iloc[0] > 0.0:
                return float(measured.iloc[0])
        if 'time' in frame:
            timestamps = pd.to_numeric(frame['time'], errors='coerce')
            deltas = timestamps.diff()
            deltas = deltas[np.isfinite(deltas) & (deltas > 0.0)]
            if not deltas.empty:
                return float(1.0 / deltas.median())
        return float(self.config.sampling_rate)

    def validate_sampling_rate(self):
        """Compare the configured rate with every log's measured rate."""
        expected = float(self.config.sampling_rate)
        tolerance = float(self.config.sampling_rate_tolerance_pct) / 100.0
        files = []
        for frame in self.raw_dfs:
            measured = self._sampling_rate(frame)
            delta = abs(measured - expected)
            limit = tolerance * max(abs(expected), abs(measured), 1.0)
            files.append({
                'source_file': str(frame['source_file'].iloc[0]),
                'expected_hz': expected,
                'measured_hz': measured,
                'delta_hz': delta,
                'passed': bool(delta <= limit),
            })
        return {
            'expected_hz': expected,
            'tolerance_pct': float(self.config.sampling_rate_tolerance_pct),
            'files': files,
            'passed': all(item['passed'] for item in files),
        }

    def load_data(self, input_dir):
        """Load and unify all CSV logs."""
        csv_files = glob.glob(str(Path(input_dir) / "*.csv"))
        for file_path in csv_files:
            try:
                df = pd.read_csv(file_path)
                if not df.empty:
                    # Dynamic logs are handled by processing.dynamics and
                    # should not contaminate the calibration response grid.
                    if is_dynamic_log(df):
                        continue
                    # Select appropriate feature columns based on config
                    speed_col = 'speed_mps' if self.config.speed_source == 'chassis' else 'ins_speed_mps'

                    if speed_col not in df.columns:
                        # Fallback for old datasets if ins_speed_mps not found
                        speed_col = 'speed_mps'

                    df['final_speed'] = df[speed_col]
                    df['command'] = df['ctl_throttle'] - df['ctl_brake']
                    df['source_file'] = os.path.basename(file_path)
                    sample_rate = self._sampling_rate(df)
                    df['sample_rate_hz'] = sample_rate

                    # Acceleration source
                    if self.config.accel_source == 'derivative':
                        dt = df['time'].diff().fillna(1.0 / sample_rate)
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

        # 1. Low-pass filter.
        # Applied per source_file: the unified frame concatenates independent
        # recording sessions sorted by absolute time, so a single filtfilt()
        # over the whole frame would smear/ring across unrelated files at
        # their concatenation boundary.
        def _filter_command_segments(frame):
            filtered = pd.Series(index=frame.index, dtype=float)
            for _, file_frame in frame.groupby('source_file', sort=False):
                sample_rate = self._sampling_rate(file_frame)
                nyquist = 0.5 * sample_rate
                norm_cutoff = max(
                    0.01, min(self.config.lowpass_cutoff / nyquist, 0.99))
                b, a = butter(
                    self.config.lowpass_order, norm_cutoff, btype='low')
                minimum_samples = max(
                    len(a), len(b), int(2 * sample_rate))
                edge_guard = max(
                    len(a), len(b),
                    int(self.config.filter_edge_guard_ms / 1000.0 *
                        sample_rate))
                indices = file_frame.index.to_numpy()
                commands = file_frame['command'].to_numpy()
                boundaries = np.r_[0, np.flatnonzero(np.diff(commands) != 0) + 1,
                                   len(indices)]
                for start, end in zip(boundaries[:-1], boundaries[1:]):
                    segment_indices = indices[start:end]
                    values = frame.loc[segment_indices, 'raw_accel'].to_numpy()
                    if len(values) >= minimum_samples:
                        result = filtfilt(b, a, values)
                        # filtfilt has no causal state, so a command step at
                        # either segment edge is interpreted as a signal
                        # transition and rings into the segment.  Keep the
                        # measured samples during that edge guard; the
                        # stability filter removes them from calibration.
                        result[:edge_guard] = values[:edge_guard]
                        filtered.loc[segment_indices] = result
                    else:
                        filtered.loc[segment_indices] = values
            return filtered

        # Filter each contiguous command segment independently.  A zero-phase
        # filter over a whole case would leak the following brake segment into
        # the tail of the preceding throttle segment.
        df['accel_filtered'] = _filter_command_segments(df)

        # Store filtered to raw too for step plot later
        for i in range(len(self.raw_dfs)):
            self.raw_dfs[i]['accel_filtered'] = _filter_command_segments(
                self.raw_dfs[i])

        # 2. Sync latency. The shift must stay within each contiguous command
        # segment; otherwise a run's throttle tail can align with its brake
        # segment.
        segment_start = (
            df['source_file'].ne(df['source_file'].shift()) |
            df['command'].ne(df['command'].shift())
        )
        df['command_segment'] = segment_start.cumsum()

        df_throttle = df[df['command'] > 0].copy()
        df_brake = df[df['command'] < 0].copy()
        df_zero = df[df['command'] == 0].copy()

        def _align_segments(frame, latency_ms):
            aligned = pd.Series(np.nan, index=frame.index, dtype=float)
            for _, segment in frame.groupby('command_segment', sort=False):
                shift = int(
                    latency_ms / 1000.0 *
                    float(segment['sample_rate_hz'].iloc[0]))
                aligned.loc[segment.index] = (
                    segment['accel_filtered'].shift(-shift).to_numpy())
            return aligned

        if not df_throttle.empty:
            df_throttle['accel_aligned'] = _align_segments(
                df_throttle, self.config.throttle_latency_ms)
        if not df_brake.empty:
            df_brake['accel_aligned'] = _align_segments(
                df_brake, self.config.brake_latency_ms)
        if not df_zero.empty: df_zero['accel_aligned'] = df_zero['accel_filtered']

        df = pd.concat([df_throttle, df_brake, df_zero], ignore_index=True)
        df = df.drop(columns=['command_segment'])
        df['aligned_speed'] = df['final_speed']
        df['is_outlier'] = False
        features = ['final_speed', 'command', 'accel_aligned']
        df_clean = df.dropna(subset=features).copy()

        # 2.5 Stability window filtering (remove transient state after command switch)
        df_clean = self._apply_stability_filter(df_clean)

        # 3. LOF - per file and per command segment
        if self.config.enable_lof:
            # Group by source file, then split by command changes within each file
            filtered_segments = []
            features = ['final_speed', 'command', 'accel_aligned']

            for source_file in df_clean['source_file'].unique():
                file_df = df_clean[df_clean['source_file'] == source_file].copy()

                if len(file_df) == 0:
                    continue

                # Reset index to detect command changes
                file_df = file_df.reset_index(drop=True)
                file_df['command_changed'] = file_df['command'].diff().abs() > 1e-6

                # Split into segments at command change points
                segment_start = 0
                for idx in file_df[file_df['command_changed']].index:
                    segment = file_df.loc[segment_start:idx-1].copy()
                    if len(segment) >= self.config.lof_neighbors:
                        # Apply LOF to this segment
                        segment_data = segment[features].values
                        lof = LocalOutlierFactor(
                            n_neighbors=self.config.lof_neighbors,
                            contamination=self.config.lof_contamination)
                        mask = lof.fit_predict(segment_data)
                        segment['is_outlier'] = (mask == -1)
                        # Keep only inliers
                        segment = segment[segment['is_outlier'] == False]

                    if len(segment) > 0:
                        filtered_segments.append(segment)
                    segment_start = idx

                # Don't forget the last segment
                last_segment = file_df.loc[segment_start:].copy()
                if len(last_segment) >= self.config.lof_neighbors:
                    segment_data = last_segment[features].values
                    lof = LocalOutlierFactor(
                        n_neighbors=self.config.lof_neighbors,
                        contamination=self.config.lof_contamination)
                    mask = lof.fit_predict(segment_data)
                    last_segment['is_outlier'] = (mask == -1)
                    last_segment = last_segment[last_segment['is_outlier'] == False]

                if len(last_segment) > 0:
                    filtered_segments.append(last_segment)

            if filtered_segments:
                df_clean = pd.concat(filtered_segments, ignore_index=True)
                # Drop temporary columns
                if 'command_changed' in df_clean.columns:
                    df_clean = df_clean.drop(columns=['command_changed'])

        self.processed_df = df_clean

    def build_calibration_table(self):
        """Generate monotonically enforced lookup table grid."""
        if self.processed_df is None or self.processed_df.empty: return None, None, None

        points = self.processed_df[['final_speed', 'command']].values
        values = self.processed_df['accel_aligned'].values

        cmd_min, cmd_max = self.processed_df['command'].min(), self.processed_df['command'].max()
        speed_max = self.processed_df['final_speed'].max()

        # Generate speed grid - don't exceed data range
        speed_grid = np.arange(0, speed_max, self.config.speed_resolution)
        command_grid = np.arange(cmd_min, cmd_max + self.config.command_resolution, self.config.command_resolution)
        grid_x, grid_y = np.meshgrid(speed_grid, command_grid)

        # Keep cells outside the measured convex hull as NaN. Filling them
        # with zero or the last row value fabricates unmeasured responses.
        grid_z = griddata(
            points, values, (grid_x, grid_y), method='linear',
            fill_value=np.nan)

        zero_cmd_idx = np.argmin(np.abs(command_grid))

        # Enforce physical constraints / monotonicity
        grid_z[zero_cmd_idx, :] = 0.0

        for i in range(zero_cmd_idx + 1, len(command_grid)):
            valid = np.isfinite(grid_z[i, :])
            previous_valid = np.isfinite(grid_z[i - 1, :])
            both_valid = valid & previous_valid
            grid_z[i, both_valid] = np.maximum(
                grid_z[i, both_valid], grid_z[i - 1, both_valid])
            grid_z[i, valid] = np.maximum(grid_z[i, valid], 0.0)

        for i in range(zero_cmd_idx - 1, -1, -1):
            valid = np.isfinite(grid_z[i, :])
            next_valid = np.isfinite(grid_z[i + 1, :])
            both_valid = valid & next_valid
            grid_z[i, both_valid] = np.minimum(
                grid_z[i, both_valid], grid_z[i + 1, both_valid])
            grid_z[i, valid] = np.minimum(grid_z[i, valid], 0.0)

        return speed_grid, command_grid, grid_z

    def _apply_stability_filter(self, df):
        """Filter out transient state data after command switches."""
        if df is None or df.empty:
            return df

        # Calculate stability window in samples
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

            transition_indices = [0]
            transition_indices.extend(
                file_df[file_df['command_changed']].index.tolist())
            for idx in transition_indices:
                # Get the command type at this switch point
                cmd = file_df.loc[idx, 'command']
                sample_rate = float(file_df.loc[idx, 'sample_rate_hz'])
                window_ms = (
                    self.config.brake_stability_window_ms if cmd < 0
                    else self.config.throttle_stability_window_ms)
                window = int(window_ms / 1000.0 * sample_rate)

                # Mark samples within the window to be discarded
                end_idx = min(len(file_df), idx + window + 1)
                mask_keep.iloc[idx:end_idx] = False

            # Apply the mask
            file_df_filtered = file_df[mask_keep].copy()

            # For throttle: filter by speed range (min and max)
            throttle_data = file_df_filtered[file_df_filtered['command'] > 0].copy()
            if len(throttle_data) > 0:
                # Apply speed range filter
                min_speed = self.config.min_throttle_speed_mps
                max_speed = self.config.max_throttle_speed_mps
                throttle_data = throttle_data[
                    (throttle_data['final_speed'] >= min_speed) &
                    (throttle_data['final_speed'] <= max_speed)
                ]

            # For brake: filter by speed range (min and max)
            brake_data = file_df_filtered[file_df_filtered['command'] < 0].copy()
            if len(brake_data) > 0:
                # Apply speed range filter
                min_speed = self.config.min_brake_speed_mps
                max_speed = self.config.max_brake_speed_mps
                brake_data = brake_data[
                    (brake_data['final_speed'] >= min_speed) &
                    (brake_data['final_speed'] <= max_speed)
                ]

            # Keep zero command data as is
            zero_data = file_df_filtered[file_df_filtered['command'] == 0].copy()

            filtered_dfs.append(pd.concat([throttle_data, brake_data, zero_data], ignore_index=True))

        if filtered_dfs:
            result = pd.concat(filtered_dfs, ignore_index=True)
            # Drop the temporary column
            if 'command_changed' in result.columns:
                result = result.drop(columns=['command_changed'])
            return result
        return df