"""Metrics for steady-state handling and closed-loop curvature tracking."""

import numpy as np
import pandas as pd


def _finite(frame, columns):
    values = frame.loc[:, columns].apply(pd.to_numeric, errors="coerce").dropna()
    if values.empty:
        raise ValueError("no finite samples for {0}".format(", ".join(columns)))
    return values


def tracking_metrics(frame, lateral_column="lateral_error_m",
                     heading_column="heading_error_rad"):
    """Calculate closed-loop lateral and heading error summary metrics."""

    values = _finite(frame, [lateral_column, heading_column])
    lateral = values[lateral_column].to_numpy(dtype=float)
    heading = values[heading_column].to_numpy(dtype=float)
    return {
        "sample_count": int(len(values)),
        "ey_mae_m": float(np.mean(np.abs(lateral))),
        "ey_rmse_m": float(np.sqrt(np.mean(lateral ** 2))),
        "ey_p95_m": float(np.percentile(np.abs(lateral), 95)),
        "ey_peak_m": float(np.max(np.abs(lateral))),
        "epsi_mae_rad": float(np.mean(np.abs(heading))),
        "epsi_rmse_rad": float(np.sqrt(np.mean(heading ** 2))),
    }


def steady_state_handling_metrics(frame, steering_rad_column,
                                  kappa_column="reference_kappa_1pm",
                                  lateral_accel_column="lateral_accel_mps2",
                                  wheelbase_m=None):
    """Fit ``delta - L*kappa = Ku*ay + offset`` on a selected steady window."""

    columns = [steering_rad_column, kappa_column, lateral_accel_column]
    values = _finite(frame, columns)
    steering = values[steering_rad_column].to_numpy(dtype=float)
    kappa = values[kappa_column].to_numpy(dtype=float)
    lateral_accel = values[lateral_accel_column].to_numpy(dtype=float)
    corrected = steering - (float(wheelbase_m) * kappa if wheelbase_m else 0.0)
    design = np.column_stack((lateral_accel, np.ones(len(lateral_accel))))
    gradient, offset = np.linalg.lstsq(design, corrected, rcond=None)[0]
    predicted = gradient * lateral_accel + offset
    residual = corrected - predicted
    return {
        "sample_count": int(len(values)),
        "understeer_gradient_rad_per_mps2": float(gradient),
        "steering_offset_rad": float(offset),
        "fit_rmse_rad": float(np.sqrt(np.mean(residual ** 2))),
        "lateral_accel_peak_mps2": float(np.max(np.abs(lateral_accel))),
    }


def select_steady_state_samples(frame, speed_target_mps, speed_tolerance_mps,
                                settle_after_ramp_sec=3.0):
    """Return only valid Phase-2 steady samples from collected snapshots.

    A plan-generated fixed-steering case labels the initial ramp explicitly.
    This selector further discards a configurable settling period and samples
    whose chassis speed left the target tolerance.  It intentionally does not
    infer a radius: curvature is a measurement, not a plan input.
    """

    required = ("elapsed_sec", "case_phase", "chassis_speed_mps")
    if any(column not in frame for column in required):
        raise ValueError("steady-state selection requires collected case phase and speed")
    values = frame.copy()
    elapsed = pd.to_numeric(values["elapsed_sec"], errors="coerce")
    speed = pd.to_numeric(values["chassis_speed_mps"], errors="coerce")
    steady = values["case_phase"].astype(str).eq("steady")
    first_steady = elapsed[steady].min()
    if pd.isna(first_steady):
        raise ValueError("case contains no steady phase")
    mask = (steady & (elapsed >= first_steady + float(settle_after_ramp_sec)) &
            (np.abs(speed - float(speed_target_mps)) <=
             float(speed_tolerance_mps)))
    selected = values.loc[mask].copy()
    if selected.empty:
        raise ValueError("no samples remain after steady-state speed filtering")
    return selected
