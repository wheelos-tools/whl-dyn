#!/usr/bin/env python3
"""
Vehicle Dynamics Calibration Data Visualization Tool

This script visualizes vehicle dynamics calibration data by generating
3D surface plots and 2D contour plots for both throttle and brake dynamics.
"""

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from scipy.interpolate import griddata
import re
import sys
import os


def load_calibration_data(file_path):
    """
    Load and parse calibration data from file.

    Args:
        file_path (str): Path to the calibration data file

    Returns:
        pd.DataFrame: DataFrame containing speed, acceleration, and command data

    Raises:
        SystemExit: If file cannot be read or no valid data is found
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            calibration_data_string = file.read()
    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found. Please make sure the file exists.")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading file '{file_path}': {e}")
        sys.exit(1)

    records = []
    # Split the data into calibration blocks
    for block in calibration_data_string.strip().split('calibration {'):
        if not block.strip():
            continue
        try:
            # Extract speed, acceleration, and command values using regex
            speed = float(re.search(r'speed:\s*([-\d.]+)', block).group(1))
            acceleration = float(re.search(r'acceleration:\s*([-\d.]+)', block).group(1))
            command = float(re.search(r'command:\s*([-\d.]+)', block).group(1))
            records.append({'speed': speed, 'acceleration': acceleration, 'command': command})
        except (AttributeError, IndexError, ValueError):
            # Silently skip invalid data blocks
            continue

    if not records:
        print("Error: No valid data parsed.")
        sys.exit(1)

    return pd.DataFrame(records)


def create_dynamics_plot(df_data, title, cmd_label, cmap_name='viridis', elev=20, azim=-75):
    """
    Create a combined chart with a 3D surface plot and a 2D contour plot.

    Args:
        df_data (pd.DataFrame): DataFrame containing the data to plot
        title (str): Title for the plot
        cmd_label (str): Label for the command axis
        cmap_name: Color map for the plots
    """
    if df_data.empty:
        print(f"Warning: {title} data is empty, skipping plot.")
        return

    # Create a regular grid for interpolation
    grid_x, grid_y = np.mgrid[
        df_data['speed'].min():df_data['speed'].max():100j,
        df_data['command'].min():df_data['command'].max():100j
    ]

    # Interpolate sparse calibration data onto the regular grid
    points = df_data[['speed', 'command']].values
    values = df_data['acceleration'].values
    grid_z = griddata(points, values, (grid_x, grid_y), method='cubic')

    # Create figure with subplots
    fig = plt.figure(figsize=(20, 9))
    fig.suptitle(title, fontsize=20, y=0.98)

    # 3D Surface Plot
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')

    # Plot the interpolated smooth surface
    surf = ax1.plot_surface(grid_x, grid_y, grid_z, cmap=cmap_name, edgecolor='none', alpha=0.8)

    # Overlay original calibration scatter points
    ax1.scatter(df_data['speed'], df_data['command'], df_data['acceleration'],
                c='red', s=50, depthshade=True, label='Original Calibration Points')

    # Configure 3D plot labels and appearance
    ax1.set_xlabel('Speed (m/s)', fontsize=12, labelpad=10)
    ax1.set_ylabel(cmd_label, fontsize=12, labelpad=10)
    ax1.set_zlabel('Acceleration (m/s²)', fontsize=12, labelpad=10)
    ax1.set_title('3D Surface Plot', fontsize=16)
    ax1.view_init(elev=elev, azim=azim)  # Adjust view angle for better visualization
    fig.colorbar(surf, ax=ax1, shrink=0.5, aspect=10, pad=0.1)
    ax1.legend()

    # 2D Contour Plot
    ax2 = fig.add_subplot(1, 2, 2)

    # Plot filled contour map
    contour = ax2.contourf(grid_x, grid_y, grid_z, levels=15, cmap=cmap_name, alpha=0.9)

    # Add contour lines with labels
    clines = ax2.contour(grid_x, grid_y, grid_z, levels=contour.levels, colors='white', linewidths=0.5)
    ax2.clabel(clines, inline=True, fontsize=8, fmt='%.1f')

    # Overlay original calibration point locations
    ax2.scatter(df_data['speed'], df_data['command'], c='red', s=50, edgecolor='white', label='Original Points')

    # Configure 2D plot labels and appearance
    ax2.set_xlabel('Speed (m/s)', fontsize=12)
    ax2.set_ylabel(cmd_label, fontsize=12)
    ax2.set_title('2D Contour Map (Top-Down View)', fontsize=16)
    fig.colorbar(contour, ax=ax2, label='Acceleration (m/s²)')
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()


def main():
    """Main function to parse arguments and generate plots."""
    parser = argparse.ArgumentParser(
        description="Plot vehicle dynamics calibration data.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                           # Use default input file (calibration_data.txt)
  %(prog)s -i my_calibration_data.txt # Use custom input file
        """
    )

    parser.add_argument(
        "-i", "--input",
        default="calibration_data.txt",
        help="Input calibration data file (default: calibration_data.txt)"
    )

    args = parser.parse_args()

    # Validate input file
    if not os.path.exists(args.input):
        print(f"Error: Input file '{args.input}' does not exist.")
        sys.exit(1)

    print(f"Loading calibration data from '{args.input}'...")
    df = load_calibration_data(args.input)

    # Split data into throttle and brake components
    # command > 0 represents throttle, command < 0 represents brake
    # For brake data, we take the absolute value of command to make it positive for plotting
    df_throttle = df[df['command'] >= 0].copy()
    df_brake = df[df['command'] < 0].copy()
    df_brake['command'] = df_brake['command'].abs()

    # Create custom color maps for better visualization
    # Throttle: light blue -> yellow -> red (representing acceleration)
    throttle_cmap = LinearSegmentedColormap.from_list("throttle_cmap", ["lightblue", "yellow", "red"])
    # Brake: light green -> cyan -> blue (representing deceleration)
    brake_cmap = LinearSegmentedColormap.from_list("brake_cmap", ["blue", "cyan", "lightgreen"])

    # Generate plots for both throttle and brake dynamics
    print("Generating throttle dynamics plot...")
    create_dynamics_plot(df_throttle,
                         'Throttle Dynamics: Acceleration(Speed, Command)',
                         'Throttle Command (%)',
                         cmap_name=throttle_cmap)

    print("Generating brake dynamics plot...")
    create_dynamics_plot(df_brake,
                         'Brake Dynamics: Acceleration(Speed, Command)',
                         'Brake Command (%)',
                         cmap_name=brake_cmap,
                         elev=30, azim=35)

    print("Plots generated successfully.")


if __name__ == '__main__':
    main()
