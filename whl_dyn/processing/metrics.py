import numpy as np
from scipy.stats import linregress
from scipy.interpolate import griddata

class MetricsEvaluator:
    @staticmethod
    def evaluate(speed_grid, command_grid, grid_z, processed_df=None):
        metrics = {}
        if len(speed_grid) == 0: return metrics

        # ========================================================================
        # 1. Deadzone (improved: compute across all speed points)
        # ========================================================================
        accel_threshold = 0.05  # m/s²

        # Collect deadzone values across all speed points
        t_deadzone_values = []
        b_deadzone_values = []

        for j in range(len(speed_grid)):
            # Throttle deadzone: find first positive command with accel > threshold
            for i, cmd in enumerate(command_grid):
                if cmd > 0 and grid_z[i, j] > accel_threshold:
                    t_deadzone_values.append(cmd)
                    break

            # Brake deadzone: find first negative command with accel < -threshold
            for i, cmd in reversed(list(enumerate(command_grid))):
                if cmd < 0 and grid_z[i, j] < -accel_threshold:
                    b_deadzone_values.append(abs(cmd))
                    break

        # Use median for robustness
        metrics['throttle_deadzone_pct'] = float(np.median(t_deadzone_values)) if t_deadzone_values else 0.0
        metrics['brake_deadzone_pct'] = float(np.median(b_deadzone_values)) if b_deadzone_values else 0.0
        metrics['throttle_deadzone_std'] = float(np.std(t_deadzone_values)) if len(t_deadzone_values) > 1 else 0.0
        metrics['brake_deadzone_std'] = float(np.std(b_deadzone_values)) if len(b_deadzone_values) > 1 else 0.0

        # ========================================================================
        # 2. Linearity (R^2 over slices)
        # ========================================================================
        r2_t = []
        r2_t_low_speed = []  # Low speed (0.5-3 m/s) - critical for control, exclude stationary
        for j in range(len(speed_grid)):
            mask = command_grid > metrics['throttle_deadzone_pct']
            if np.sum(mask) > 2:
                _, _, r, _, _ = linregress(command_grid[mask], grid_z[mask, j])
                r2_t.append(r**2)
                if 0.5 <= speed_grid[j] < 3.0:
                    r2_t_low_speed.append(r**2)
        metrics['throttle_linearity_R2'] = float(np.mean(r2_t)) if r2_t else 0.0
        metrics['throttle_linearity_R2_low_speed'] = float(np.mean(r2_t_low_speed)) if r2_t_low_speed else 0.0

        r2_b = []
        r2_b_low_speed = []
        for j in range(len(speed_grid)):
            mask = command_grid < -metrics['brake_deadzone_pct']
            if np.sum(mask) > 2:
                _, _, r, _, _ = linregress(command_grid[mask], grid_z[mask, j])
                r2_b.append(r**2)
                if 0.5 <= speed_grid[j] < 3.0:
                    r2_b_low_speed.append(r**2)
        metrics['brake_linearity_R2'] = float(np.mean(r2_b)) if r2_b else 0.0
        metrics['brake_linearity_R2_low_speed'] = float(np.mean(r2_b_low_speed)) if r2_b_low_speed else 0.0

        # ========================================================================
        # 3. Smoothness (Laplacian magnitude) - separately for throttle and brake
        # ========================================================================
        throttle_mask = command_grid > 0
        brake_mask = command_grid < 0

        if np.any(throttle_mask) and grid_z.shape[1] > 2:
            throttle_grid = grid_z[throttle_mask, :]
            dx2_t = np.gradient(np.gradient(throttle_grid, axis=0), axis=0)
            dy2_t = np.gradient(np.gradient(throttle_grid, axis=1), axis=1)
            metrics['throttle_smoothness_laplacian_mean'] = float(np.mean(np.abs(dx2_t) + np.abs(dy2_t)))
            metrics['throttle_smoothness_score_100'] = float(max(0.0, 100.0 - metrics['throttle_smoothness_laplacian_mean'] * 500))
        else:
            metrics['throttle_smoothness_score_100'] = 0.0
            metrics['throttle_smoothness_laplacian_mean'] = 0.0

        if np.any(brake_mask) and grid_z.shape[1] > 2:
            brake_grid = grid_z[brake_mask, :]
            dx2_b = np.gradient(np.gradient(brake_grid, axis=0), axis=0)
            dy2_b = np.gradient(np.gradient(brake_grid, axis=1), axis=1)
            metrics['brake_smoothness_laplacian_mean'] = float(np.mean(np.abs(dx2_b) + np.abs(dy2_b)))
            metrics['brake_smoothness_score_100'] = float(max(0.0, 100.0 - metrics['brake_smoothness_laplacian_mean'] * 500))
        else:
            metrics['brake_smoothness_score_100'] = 0.0
            metrics['brake_smoothness_laplacian_mean'] = 0.0

        # Overall smoothness (for backward compatibility)
        if grid_z.shape[0] > 2 and grid_z.shape[1] > 2:
            dx2 = np.gradient(np.gradient(grid_z, axis=0), axis=0)
            dy2 = np.gradient(np.gradient(grid_z, axis=1), axis=1)
            metrics['smoothness_laplacian_mean'] = float(np.mean(np.abs(dx2) + np.abs(dy2)))
            metrics['smoothness_score_100'] = float(max(0.0, 100.0 - metrics['smoothness_laplacian_mean'] * 500))
        else:
            metrics['smoothness_score_100'] = 0.0
            metrics['smoothness_laplacian_mean'] = 0.0

        # ========================================================================
        # 4. Response Range Check
        # ========================================================================
        throttle_mask = command_grid > metrics['throttle_deadzone_pct']
        brake_mask = command_grid < -metrics['brake_deadzone_pct']

        if np.any(throttle_mask):
            metrics['max_throttle_accel'] = float(np.max(grid_z[throttle_mask, :]))
            metrics['min_throttle_accel'] = float(np.min(grid_z[throttle_mask, :]))
        else:
            metrics['max_throttle_accel'] = 0.0
            metrics['min_throttle_accel'] = 0.0

        if np.any(brake_mask):
            metrics['max_brake_decel'] = float(np.min(grid_z[brake_mask, :]))  # Most negative
            metrics['min_brake_decel'] = float(np.max(grid_z[brake_mask, :]))  # Least negative
        else:
            metrics['max_brake_decel'] = 0.0
            metrics['min_brake_decel'] = 0.0

        # ========================================================================
        # 5. Monotonicity Violation Count
        # ========================================================================
        throttle_violations = 0
        brake_violations = 0

        for j in range(len(speed_grid)):
            # Check throttle monotonicity (command increases -> accel increases)
            if np.any(throttle_mask):
                throttle_accel = grid_z[throttle_mask, j]
                if not np.all(np.diff(throttle_accel) >= -0.01):  # Allow small tolerance
                    throttle_violations += 1

            # Check brake monotonicity (command increases -> accel increases, i.e., becomes less negative)
            if np.any(brake_mask):
                brake_accel = grid_z[brake_mask, j]
                # For brake: as command goes from -100 to 0, accel goes from most negative to 0
                # So diff should be positive (accel increases) or close to zero
                if not np.all(np.diff(brake_accel) >= -0.01):  # Allow small tolerance
                    brake_violations += 1

        metrics['throttle_monotonic_violations'] = throttle_violations
        metrics['brake_monotonic_violations'] = brake_violations
        metrics['monotonicity_pass'] = (throttle_violations == 0 and brake_violations == 0)
        metrics['monotonicity_total_violations'] = throttle_violations + brake_violations

        # ========================================================================
        # 6. Residual Analysis (if raw data provided) - separately for throttle and brake
        # ========================================================================
        if processed_df is not None and not processed_df.empty:
            try:
                # Create meshgrid for interpolation points
                speed_mesh, cmd_mesh = np.meshgrid(speed_grid, command_grid)

                # Process throttle data
                throttle_df = processed_df[processed_df['command'] > 0].copy()
                if len(throttle_df) >= 10:  # Need minimum data points
                    throttle_points = np.column_stack([
                        throttle_df['final_speed'].values,
                        throttle_df['command'].values
                    ])

                    throttle_predicted = griddata(
                        np.column_stack([speed_mesh.ravel(), cmd_mesh.ravel()]),
                        grid_z.ravel(),
                        throttle_points,
                        method='linear',
                        fill_value=np.nan
                    )

                    throttle_actual = throttle_df['accel_aligned'].values
                    valid_mask = ~np.isnan(throttle_predicted)

                    if np.sum(valid_mask) > 0:
                        residuals = throttle_actual[valid_mask] - throttle_predicted[valid_mask]
                        metrics['throttle_residual_mae'] = float(np.mean(np.abs(residuals)))
                        metrics['throttle_residual_rmse'] = float(np.sqrt(np.mean(residuals**2)))
                        metrics['throttle_residual_max'] = float(np.max(np.abs(residuals)))
                        metrics['throttle_residual_std'] = float(np.std(residuals))

                        acceptable_error = 0.2  # m/s²
                        metrics['throttle_within_tolerance_pct'] = float(
                            100.0 * np.sum(np.abs(residuals) < acceptable_error) / len(residuals)
                        )
                        metrics['throttle_residual_valid_pct'] = float(100.0 * np.sum(valid_mask) / len(throttle_predicted))
                    else:
                        metrics['throttle_residual_mae'] = None
                        metrics['throttle_residual_rmse'] = None
                else:
                    metrics['throttle_residual_mae'] = None
                    metrics['throttle_residual_rmse'] = None

                # Process brake data
                brake_df = processed_df[processed_df['command'] < 0].copy()
                if len(brake_df) >= 10:  # Need minimum data points
                    brake_points = np.column_stack([
                        brake_df['final_speed'].values,
                        brake_df['command'].values
                    ])

                    brake_predicted = griddata(
                        np.column_stack([speed_mesh.ravel(), cmd_mesh.ravel()]),
                        grid_z.ravel(),
                        brake_points,
                        method='linear',
                        fill_value=np.nan
                    )

                    brake_actual = brake_df['accel_aligned'].values
                    valid_mask = ~np.isnan(brake_predicted)

                    if np.sum(valid_mask) > 0:
                        residuals = brake_actual[valid_mask] - brake_predicted[valid_mask]
                        metrics['brake_residual_mae'] = float(np.mean(np.abs(residuals)))
                        metrics['brake_residual_rmse'] = float(np.sqrt(np.mean(residuals**2)))
                        metrics['brake_residual_max'] = float(np.max(np.abs(residuals)))
                        metrics['brake_residual_std'] = float(np.std(residuals))

                        acceptable_error = 0.2  # m/s²
                        metrics['brake_within_tolerance_pct'] = float(
                            100.0 * np.sum(np.abs(residuals) < acceptable_error) / len(residuals)
                        )
                        metrics['brake_residual_valid_pct'] = float(100.0 * np.sum(valid_mask) / len(brake_predicted))
                    else:
                        metrics['brake_residual_mae'] = None
                        metrics['brake_residual_rmse'] = None
                else:
                    metrics['brake_residual_mae'] = None
                    metrics['brake_residual_rmse'] = None

                # Overall residual (for backward compatibility, using all data)
                points = np.column_stack([
                    processed_df['final_speed'].values,
                    processed_df['command'].values
                ])

                predicted = griddata(
                    np.column_stack([speed_mesh.ravel(), cmd_mesh.ravel()]),
                    grid_z.ravel(),
                    points,
                    method='linear',
                    fill_value=np.nan
                )

                actual = processed_df['accel_aligned'].values
                valid_mask = ~np.isnan(predicted)

                if np.sum(valid_mask) > 0:
                    residuals = actual[valid_mask] - predicted[valid_mask]
                    metrics['residual_mae'] = float(np.mean(np.abs(residuals)))
                    metrics['residual_rmse'] = float(np.sqrt(np.mean(residuals**2)))
                    metrics['residual_max'] = float(np.max(np.abs(residuals)))
                    metrics['residual_std'] = float(np.std(residuals))

                    acceptable_error = 0.2  # m/s²
                    metrics['within_tolerance_pct'] = float(
                        100.0 * np.sum(np.abs(residuals) < acceptable_error) / len(residuals)
                    )
                    metrics['residual_valid_pct'] = float(100.0 * np.sum(valid_mask) / len(predicted))
                else:
                    metrics['residual_mae'] = None
                    metrics['residual_rmse'] = None
            except Exception:
                metrics['residual_mae'] = None
                metrics['residual_rmse'] = None
                metrics['throttle_residual_mae'] = None
                metrics['throttle_residual_rmse'] = None
                metrics['brake_residual_mae'] = None
                metrics['brake_residual_rmse'] = None
        else:
            metrics['residual_mae'] = None
            metrics['residual_rmse'] = None
            metrics['throttle_residual_mae'] = None
            metrics['throttle_residual_rmse'] = None
            metrics['brake_residual_mae'] = None
            metrics['brake_residual_rmse'] = None

        return metrics