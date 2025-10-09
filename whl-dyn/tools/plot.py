import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from scipy.interpolate import griddata
import re

# ============ 1. Data Parsing (Basically the same as your code) ============
# Assume calibration_data.txt is in the same directory
file_path = 'calibration_data.txt'

try:
    with open(file_path, 'r', encoding='utf-8') as file:
        calibration_data_string = file.read()
except FileNotFoundError:
    print(f"Error: File '{file_path}' not found. Please make sure the file exists.")
    exit()

records = []
for block in calibration_data_string.strip().split('calibration {'):
    if not block.strip():
        continue
    try:
        speed = float(re.search(r'speed:\s*([-\d.]+)', block).group(1))
        acceleration = float(re.search(r'acceleration:\s*([-\d.]+)', block).group(1))
        command = float(re.search(r'command:\s*([-\d.]+)', block).group(1))
        records.append({'speed': speed, 'acceleration': acceleration, 'command': command})
    except (AttributeError, IndexError):
        continue # Silently skip invalid data blocks

if not records:
    print("Error: No valid data parsed.")
    exit()

df = pd.DataFrame(records)

# --- Core modification: Split throttle and brake data ---
# command > 0 is throttle, command < 0 is brake
# Note: We take the absolute value of brake command to make it positive for plotting and understanding
df_throttle = df[df['command'] >= 0].copy()
df_brake = df[df['command'] < 0].copy()
df_brake['command'] = df_brake['command'].abs()

# ============ 2. Visualization Function (Industry Practice) ============

def create_dynamics_plot(df_data, title, cmd_label, cmap_name='viridis'):
    """
    Create a combined chart with a 3D surface plot and a 2D contour plot.
    """
    if df_data.empty:
        print(f"Warning: {title} data is empty, skipping plot.")
        return

    # --- Data gridding for interpolation and smoothing the surface ---
    # Create a regular grid
    grid_x, grid_y = np.mgrid[
        df_data['speed'].min():df_data['speed'].max():100j,
        df_data['command'].min():df_data['command'].max():100j
    ]

    # Interpolate sparse calibration data onto the regular grid
    points = df_data[['speed', 'command']].values
    values = df_data['acceleration'].values
    grid_z = griddata(points, values, (grid_x, grid_y), method='cubic')

    fig = plt.figure(figsize=(20, 9))
    fig.suptitle(title, fontsize=20, y=0.98)

    # --- Left: 3D Surface Plot ---
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')

    # Plot the interpolated smooth surface
    surf = ax1.plot_surface(grid_x, grid_y, grid_z, cmap=cmap_name, edgecolor='none', alpha=0.8)

    # Plot original calibration scatter points
    ax1.scatter(df_data['speed'], df_data['command'], df_data['acceleration'],
                c='red', s=50, depthshade=True, label='Original Calibration Points')

    ax1.set_xlabel('Speed (m/s)', fontsize=12, labelpad=10)
    ax1.set_ylabel(cmd_label, fontsize=12, labelpad=10)
    ax1.set_zlabel('Acceleration (m/s^2)', fontsize=12, labelpad=10)
    ax1.set_title('3D Surface Plot', fontsize=16)
    ax1.view_init(elev=20, azim=-135) # Adjust view angle
    fig.colorbar(surf, ax=ax1, shrink=0.5, aspect=10, pad=0.1)
    ax1.legend()

    # --- Right: 2D Contour Plot ---
    ax2 = fig.add_subplot(1, 2, 2)

    # Plot filled contour map
    contour = ax2.contourf(grid_x, grid_y, grid_z, levels=15, cmap=cmap_name, alpha=0.9)

    # Plot contour lines and values
    clines = ax2.contour(grid_x, grid_y, grid_z, levels=contour.levels, colors='white', linewidths=0.5)
    ax2.clabel(clines, inline=True, fontsize=8, fmt='%.1f')

    # Plot original calibration point locations
    ax2.scatter(df_data['speed'], df_data['command'], c='red', s=50, edgecolor='white', label='Original Points')

    ax2.set_xlabel('Speed (m/s)', fontsize=12)
    ax2.set_ylabel(cmd_label, fontsize=12)
    ax2.set_title('2D Contour Map (Top-Down View)', fontsize=16)
    fig.colorbar(contour, ax=ax2, label='Acceleration ($m/s^2$)')
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

# ============ 3. Generate Charts ============

# Custom color maps for throttle and brake
# Throttle: from neutral to red (acceleration)
throttle_cmap = LinearSegmentedColormap.from_list("throttle_cmap", ["lightblue", "yellow", "red"])
# Brake: from neutral to blue (deceleration)
brake_cmap = LinearSegmentedColormap.from_list("brake_cmap", ["lightgreen", "cyan", "blue"])


create_dynamics_plot(df_throttle,
                     'Throttle Dynamics: Acceleration(Speed, Command)',
                     'Throttle Command (%)',
                     cmap_name=throttle_cmap)

create_dynamics_plot(df_brake,
                     'Brake Dynamics: Acceleration(Speed, Command)',
                     'Brake Command (%)',
                     cmap_name=brake_cmap)
