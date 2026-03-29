import os
import json
from pathlib import Path
import matplotlib.pyplot as plt
import pandas as pd

class Exporter:
    @staticmethod
    def save_step_responses(raw_dfs, output_dir):
        plot_dir = Path(output_dir) / "step_responses"
        plot_dir.mkdir(exist_ok=True, parents=True)

        for df in raw_dfs:
            if 'command' not in df or 'accel_filtered' not in df: continue

            df['cmd_diff'] = df['command'].diff().abs()
            step_idx = df['cmd_diff'].idxmax()

            if pd.isna(step_idx) or df['cmd_diff'].max() < 10.0: continue

            file_name = df['source_file'].iloc[0]
            fig, ax1 = plt.subplots(figsize=(10, 5))
            ax2 = ax1.twinx()

            start_idx = max(0, step_idx - 100)
            end_idx = min(len(df), step_idx + 400)
            window = df.iloc[start_idx:end_idx]

            t_base = window['time'] - window['time'].min()
            ax1.plot(t_base, window['command'], 'b-', label='Command (%)')
            ax2.plot(t_base, window['accel_filtered'], 'r-', label='Filtered Accel')
            ax2.plot(t_base, window['raw_accel'], 'r--', alpha=0.3, label='Raw Accel')

            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Command (%)', color='b')
            ax2.set_ylabel('Acceleration (m/s²)', color='r')
            plt.title(f'Step Response Analysis - {file_name}')

            h1, l1 = ax1.get_legend_handles_labels()
            h2, l2 = ax2.get_legend_handles_labels()
            ax1.legend(h1+h2, l1+l2, loc='upper left')

            plt.grid(True, alpha=0.5)
            fig.tight_layout()
            plt.savefig(plot_dir / f"{file_name.replace('.csv', '')}_step.png")
            plt.close(fig)

    @staticmethod
    def save_unified_csv(speed_grid, command_grid, grid_z, output_dir):
        filepath = Path(output_dir) / "unified_calibration_table.csv"
        with open(filepath, 'w') as f:
            f.write("speed,command,acceleration\n")
            for i, speed in enumerate(speed_grid):
                for j, command in enumerate(command_grid):
                    f.write(f"{speed:.2f},{command:.2f},{grid_z[j, i]:.4f}\n")

    @staticmethod
    def save_protobuf(speed_grid, command_grid, grid_z, output_dir):
        filepath = Path(output_dir) / "calibration_table.pb.txt"
        with open(filepath, 'w') as f:
            for i, speed in enumerate(speed_grid):
                for j, command in enumerate(command_grid):
                    if abs(grid_z[j, i]) < 0.001: continue
                    f.write("calibration {\n")
                    f.write(f"  speed: {speed}\n")
                    f.write(f"  acceleration: {grid_z[j, i]}\n")
                    f.write(f"  command: {command}\n")
                    f.write("}\n")

    @staticmethod
    def save_metrics(metrics, output_dir):
        metrics_file = Path(output_dir) / 'evaluation_metrics.json'
        with open(metrics_file, 'w') as f:
            json.dump(metrics, f, indent=4)
