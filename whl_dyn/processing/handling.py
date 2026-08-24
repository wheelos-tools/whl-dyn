"""Metrics for steady-state handling and closed-loop curvature tracking."""

import numpy as np
import pandas as pd


def _finite(frame, columns):
    values = frame.loc[:, columns].apply(pd.to_numeric, errors="coerce").dropna()
    if values.empty:
        raise ValueError("no finite samples for {0}".format(", ".join(columns)))
    return values


def _aligned_frame(frame):
    """Keep only collection rows that pass the persisted time-alignment gate."""

    if "time_aligned" not in frame:
        return frame
    aligned_flag = frame["time_aligned"]
    if aligned_flag.dtype == bool:
        mask = aligned_flag
    else:
        mask = aligned_flag.astype(str).str.lower().isin(("true", "1", "yes"))
    aligned = frame.loc[mask].copy()
    if aligned.empty:
        raise ValueError("no time-aligned samples remain")
    return aligned


def tracking_metrics(frame, lateral_column="lateral_error_m",
                     heading_column="heading_error_rad"):
    """Calculate closed-loop lateral and heading error summary metrics."""

    values = _finite(_aligned_frame(frame), [lateral_column, heading_column])
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
    values = _finite(_aligned_frame(frame), columns)
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
    values = _aligned_frame(frame)
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


def fixed_steering_steady_state_metrics(
        frame, steering_wheel_column="steering_feedback",
        road_wheel_column=None, speed_column="chassis_speed_mps",
        yaw_rate_column="yaw_rate_radps",
        lateral_accel_column="lateral_accel_mps2",
        sideslip_column="sideslip_rad", esc_column="esc_active",
        wheel_slip_column="wheel_slip_ratio", wheelbase_m=None,
        max_abs_sideslip_rad=None, max_sideslip_rate_radps=None,
        max_yaw_rate_error_radps=None, max_abs_wheel_slip=None):
    """Summarize one selected fixed-steering steady-state window.

    Optional limit signals are only evaluated when their columns and threshold
    are both supplied.  Missing signals therefore remain explicitly
    ``unavailable`` instead of being interpreted as safe.
    """

    required = (
        steering_wheel_column, speed_column, yaw_rate_column,
        lateral_accel_column,
    )
    values = _finite(_aligned_frame(frame), list(required)).copy()
    speed = values[speed_column].to_numpy(dtype=float)
    yaw_rate = values[yaw_rate_column].to_numpy(dtype=float)
    lateral_accel = values[lateral_accel_column].to_numpy(dtype=float)
    steering_wheel = values[steering_wheel_column].to_numpy(dtype=float)
    nonzero_speed = np.abs(speed) > np.finfo(float).eps
    if not np.any(nonzero_speed):
        raise ValueError("steady-state metrics require non-zero speed")
    curvature = yaw_rate[nonzero_speed] / speed[nonzero_speed]
    radius = 1.0 / np.abs(curvature[np.abs(curvature) > np.finfo(float).eps])
    result = {
        "sample_count": int(len(values)),
        "speed_mean_mps": float(np.mean(speed)),
        "speed_std_mps": float(np.std(speed)),
        "steering_wheel_mean": float(np.mean(steering_wheel)),
        "steering_wheel_std": float(np.std(steering_wheel)),
        "yaw_rate_mean_radps": float(np.mean(yaw_rate)),
        "lateral_accel_mean_mps2": float(np.mean(lateral_accel)),
        "lateral_accel_peak_mps2": float(np.max(np.abs(lateral_accel))),
        "curvature_mean_1pm": float(np.mean(curvature)),
        "radius_mean_m": float(np.mean(radius)) if len(radius) else None,
        "yaw_rate_gain_per_steering": _ratio_mean(yaw_rate, steering_wheel),
        "lateral_accel_gain_per_steering": _ratio_mean(
            lateral_accel, steering_wheel),
        "limit_flags": {},
    }
    optional = _aligned_optional(frame, values.index, sideslip_column)
    if optional is not None:
        sideslip = optional.to_numpy(dtype=float)
        result["sideslip_mean_rad"] = float(np.mean(sideslip))
        result["sideslip_peak_rad"] = float(np.max(np.abs(sideslip)))
        if "elapsed_sec" in frame:
            elapsed = pd.to_numeric(frame.loc[values.index, "elapsed_sec"],
                                    errors="coerce").to_numpy(dtype=float)
            valid = np.isfinite(elapsed) & np.isfinite(sideslip)
            if np.count_nonzero(valid) >= 2:
                result["sideslip_rate_peak_radps"] = float(np.max(np.abs(
                    np.gradient(sideslip[valid], elapsed[valid]))))
        _threshold_flag(result, "sideslip_exceeded", result["sideslip_peak_rad"],
                        max_abs_sideslip_rad)
        _threshold_flag(result, "sideslip_rate_exceeded",
                        result.get("sideslip_rate_peak_radps"),
                        max_sideslip_rate_radps)
    else:
        result["sideslip_mean_rad"] = None
        result["sideslip_peak_rad"] = None
        result["sideslip_rate_peak_radps"] = None

    if road_wheel_column and road_wheel_column in frame:
        road_wheel = _aligned_optional(frame, values.index, road_wheel_column)
        if road_wheel is not None:
            road_angle = road_wheel.to_numpy(dtype=float)
            result["road_wheel_mean_rad"] = float(np.mean(road_angle))
            result["yaw_rate_gain_per_road_wheel"] = _ratio_mean(
                yaw_rate, road_angle)
            if wheelbase_m is not None:
                wheelbase = float(wheelbase_m)
                if wheelbase <= 0.0:
                    raise ValueError("wheelbase_m must be positive")
                expected = speed * np.tan(road_angle) / wheelbase
                yaw_error = yaw_rate - expected
                result["kinematic_yaw_rate_mean_radps"] = float(np.mean(expected))
                result["kinematic_yaw_rate_error_peak_radps"] = float(
                    np.max(np.abs(yaw_error)))
                _threshold_flag(result, "yaw_rate_deviation_exceeded",
                                result["kinematic_yaw_rate_error_peak_radps"],
                                max_yaw_rate_error_radps)

    esc = _aligned_optional(frame, values.index, esc_column)
    result["esc_active_fraction"] = (
        float(np.mean(esc.to_numpy(dtype=float) != 0.0)) if esc is not None else None)
    result["limit_flags"]["esc_intervention"] = (
        result["esc_active_fraction"] > 0.0
        if result["esc_active_fraction"] is not None else None)

    wheel_slip = _aligned_optional(frame, values.index, wheel_slip_column)
    result["wheel_slip_peak"] = (
        float(np.max(np.abs(wheel_slip.to_numpy(dtype=float))))
        if wheel_slip is not None else None)
    _threshold_flag(result, "wheel_slip_exceeded", result["wheel_slip_peak"],
                    max_abs_wheel_slip)
    result["limit_evaluation_available"] = any(
        value is not None for value in result["limit_flags"].values())
    result["limit_detected"] = any(
        value is True for value in result["limit_flags"].values())
    return result


def _aligned_optional(frame, index, column):
    if not column or column not in frame:
        return None
    values = pd.to_numeric(frame.loc[index, column], errors="coerce")
    return values if values.notna().all() else None


def _ratio_mean(numerator, denominator):
    valid = np.abs(denominator) > np.finfo(float).eps
    return float(np.mean(numerator[valid] / denominator[valid])) if np.any(valid) else None


def _threshold_flag(result, name, value, threshold):
    result["limit_flags"][name] = (
        abs(float(value)) > float(threshold)
        if value is not None and threshold is not None else None)
