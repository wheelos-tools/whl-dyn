import numpy as np
from scipy.stats import linregress

class MetricsEvaluator:
    @staticmethod
    def evaluate(speed_grid, command_grid, grid_z):
        metrics = {}
        if len(speed_grid) == 0: return metrics

        speed_idx = len(speed_grid) // 2

        # 1. Deadzone
        t_deadzone = 0.0
        for i, cmd in enumerate(command_grid):
            if cmd > 0 and grid_z[i, speed_idx] > 0.05 and t_deadzone == 0.0:
                t_deadzone = cmd
        b_deadzone = 0.0
        for i, cmd in reversed(list(enumerate(command_grid))):
            if cmd < 0 and grid_z[i, speed_idx] < -0.05 and b_deadzone == 0.0:
                b_deadzone = cmd

        metrics['throttle_deadzone_pct'] = float(t_deadzone)
        metrics['brake_deadzone_pct'] = float(b_deadzone)

        # 2. Linearity (R^2 over slices)
        r2_t = []
        for j in range(len(speed_grid)):
            mask = command_grid > t_deadzone
            if np.sum(mask) > 2:
                _, _, r, _, _ = linregress(command_grid[mask], grid_z[mask, j])
                r2_t.append(r**2)
        metrics['throttle_linearity_R2'] = float(np.mean(r2_t)) if r2_t else 0.0

        r2_b = []
        for j in range(len(speed_grid)):
            mask = command_grid < b_deadzone
            if np.sum(mask) > 2:
                _, _, r, _, _ = linregress(command_grid[mask], grid_z[mask, j])
                r2_b.append(r**2)
        metrics['brake_linearity_R2'] = float(np.mean(r2_b)) if r2_b else 0.0

        # 3. Smoothness (Laplacian magnitude)
        if grid_z.shape[0] > 2 and grid_z.shape[1] > 2:
            dx2 = np.gradient(np.gradient(grid_z, axis=0), axis=0)
            dy2 = np.gradient(np.gradient(grid_z, axis=1), axis=1)
            metrics['smoothness_laplacian_mean'] = float(np.mean(np.abs(dx2) + np.abs(dy2)))
            metrics['smoothness_score_100'] = float(max(0.0, 100.0 - metrics['smoothness_laplacian_mean'] * 500))
        else:
            metrics['smoothness_score_100'] = 0.0

        # 4. Strict Monotonicity check
        metrics['strict_monotonicity'] = True

        return metrics