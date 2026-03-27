"""Configuration properties for vehicle calibration processing."""
from dataclasses import dataclass

@dataclass
class CalibrationConfig:
    # --- Data Source Configuration ---
    # 'chassis' (speed_mps) or 'localization' (ins_speed_mps)
    speed_source: str = 'chassis'
    # 'imu' (imu_accel_y) or 'derivative' (d(speed)/dt)
    accel_source: str = 'imu'

    # --- Signal Processing ---
    lowpass_order: int = 6
    lowpass_cutoff: float = 1.0
    sampling_rate: float = 100.0

    # --- Outlier Detection (LOF) ---
    enable_lof: bool = True
    lof_neighbors: int = 30
    lof_contamination: float = 0.02

    # --- Grid Generation ---
    speed_resolution: float = 0.2
    command_resolution: float = 5.0

    # --- Step Response / Sync Delay ---
    throttle_latency_ms: int = 60
    brake_latency_ms: int = 60
