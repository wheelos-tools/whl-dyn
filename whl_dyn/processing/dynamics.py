"""Time and frequency domain helpers for open-loop vehicle tests."""

from __future__ import division

from pathlib import Path
import numpy as np
import pandas as pd
from scipy import signal
from scipy.optimize import curve_fit


def _column(df, requested, candidates):
    if requested and requested in df.columns:
        return requested
    for name in candidates:
        if name in df.columns:
            return name
    raise ValueError("none of the requested signal columns are present")


def _signals(df, input_col=None, output_col=None):
    if not isinstance(df, pd.DataFrame) or df.empty:
        raise ValueError("dynamic data must be a non-empty DataFrame")
    time_col = _column(df, "time", ("timestamp",))
    in_col = _column(df, input_col, ("command", "ctl_throttle", "input"))
    out_col = _column(df, output_col,
                      ("feedback", "output", "imu_accel_y", "acceleration", "speed_mps"))
    values = df[[time_col, in_col, out_col]].apply(pd.to_numeric, errors="coerce").dropna()
    if len(values) < 4:
        raise ValueError("at least four valid samples are required")
    values = values.sort_values(time_col).drop_duplicates(time_col)
    t = values[time_col].to_numpy(dtype=float)
    u = values[in_col].to_numpy(dtype=float)
    y = values[out_col].to_numpy(dtype=float)
    if np.any(np.diff(t) <= 0):
        raise ValueError("time must be strictly increasing")
    return t, u, y, in_col, out_col


def _step_index(u):
    changes = np.abs(np.diff(u))
    if not np.any(changes > 0):
        raise ValueError("step analysis requires a command change")
    return int(np.argmax(changes) + 1)


def is_dynamic_log(df):
    """Return True when a collector CSV carries dynamic-test metadata."""
    if not isinstance(df, pd.DataFrame) or df.empty:
        return False
    if "mode" in df.columns:
        mode_value = str(df["mode"].iloc[0]).strip().lower()
        if mode_value not in ("", "calibration", "nan"):
            return True
    if "domain" in df.columns:
        domain_value = str(df["domain"].iloc[0]).strip().lower()
        if domain_value in ("actuator_characterization", "frequency_response"):
            return True
    return False


def identify_fopdt(df, input_col=None, output_col=None, step_index=None):
    """Identify a first-order-plus-dead-time model from a step response."""
    t, u, y, in_col, out_col = _signals(df, input_col, output_col)
    index = _step_index(u) if step_index is None else int(step_index)
    if index <= 0 or index >= len(t):
        raise ValueError("step_index must lie inside the sample range")
    pre_count = max(1, min(index, len(t) // 5))
    u0 = float(np.median(u[max(0, index - pre_count):index]))
    y0 = float(np.median(y[max(0, index - pre_count):index]))
    post_count = max(1, min(len(y) - index, len(y) // 5))
    u1 = float(np.median(u[-post_count:]))
    y1 = float(np.median(y[-post_count:]))
    delta_u = u1 - u0
    if abs(delta_u) < 1e-12:
        raise ValueError("step amplitude is zero")
    gain = (y1 - y0) / delta_u
    tau_guess = max(float(t[-1] - t[index]) / 3.0, np.finfo(float).eps)
    dead_guess = max(float(t[index] - t[0]) * 0.05, 0.0)

    def model(time, offset, model_gain, tau, dead_time):
        delayed = np.maximum(time - dead_time - t[index], 0.0)
        return offset + model_gain * delta_u * (1.0 - np.exp(-delayed / tau))

    fit_start = max(0, index - 1)
    try:
        params, _ = curve_fit(
            model, t[fit_start:], y[fit_start:],
            p0=[y0, gain, tau_guess, dead_guess],
            bounds=([-np.inf, -np.inf, np.finfo(float).eps, 0.0],
                    [np.inf, np.inf, np.inf, max(t[-1] - t[index], 0.0)]),
            maxfev=20000,
        )
        offset, gain, tau, dead_time = [float(value) for value in params]
    except (RuntimeError, ValueError):
        # A threshold estimate remains useful for noisy or very short logs.
        offset, tau, dead_time = y0, tau_guess, dead_guess

    fitted = model(t, offset, gain, tau, dead_time)
    return {
        "input_signal": in_col,
        "output_signal": out_col,
        "gain": float(gain),
        "time_constant_sec": float(tau),
        "dead_time_sec": float(dead_time),
        "offset": float(offset),
        "step_index": index,
        "fit": {"time": t.tolist(), "value": fitted.tolist()},
    }


def analyze_step_response(df, input_col=None, output_col=None):
    """Calculate dead time, rise/settling time, gain and time constant."""
    t, u, y, in_col, out_col = _signals(df, input_col, output_col)
    index = _step_index(u)
    pre_count = max(1, min(index, len(t) // 5))
    baseline_u = float(np.median(u[max(0, index - pre_count):index]))
    baseline_y = float(np.median(y[max(0, index - pre_count):index]))
    final_count = max(1, min(len(y) - index, len(y) // 5))
    final_u = float(np.median(u[-final_count:]))
    final_y = float(np.median(y[-final_count:]))
    delta_u = final_u - baseline_u
    delta_y = final_y - baseline_y
    if abs(delta_u) < 1e-12:
        raise ValueError("step amplitude is zero")
    response = (y - baseline_y) / delta_y if abs(delta_y) > 1e-12 else np.zeros_like(y)
    direction = 1.0 if delta_y >= 0 else -1.0
    normalized = direction * response
    after_t = t[index:] - t[index]
    after_norm = normalized[index:]

    def first_at(level):
        found = np.flatnonzero(after_norm >= level)
        return float(after_t[found[0]]) if len(found) else None

    t10, t90 = first_at(0.1), first_at(0.9)
    rise = t90 - t10 if t10 is not None and t90 is not None else None
    settling = None
    band = 0.02
    for pos in range(len(after_norm)):
        if np.all(np.abs(after_norm[pos:] - 1.0) <= band):
            settling = float(after_t[pos])
            break
    dead = first_at(0.1)
    tau = first_at(1.0 - np.exp(-1.0))
    model = identify_fopdt(df, input_col, output_col, index)
    return {
        "input_signal": in_col,
        "output_signal": out_col,
        "step_index": index,
        "baseline_input": baseline_u,
        "final_input": final_u,
        "baseline_output": baseline_y,
        "final_output": final_y,
        "gain": float(delta_y / delta_u),
        "dead_time_sec": dead,
        "rise_time_sec": rise,
        "settling_time_sec": settling,
        "time_constant_sec": tau,
        "fopdt": model,
        "data": {
            "time": t.tolist(),
            "input": u.tolist(),
            "output": y.tolist(),
            "normalized_output": normalized.tolist(),
        },
    }


def analyze_step(df, input_col=None, output_col=None):
    """Short alias used by the UI and external scripts."""
    return analyze_step_response(df, input_col, output_col)


def analyze_frequency_response(df, input_col=None, output_col=None,
                               sampling_rate_hz=None, nperseg=None):
    """Estimate frequency response, coherence, magnitude and phase."""
    t, u, y, in_col, out_col = _signals(df, input_col, output_col)
    dt = float(np.median(np.diff(t)))
    fs = float(sampling_rate_hz) if sampling_rate_hz else 1.0 / dt
    segment = int(nperseg or min(256, len(t)))
    if segment < 4:
        raise ValueError("at least four samples are required for frequency analysis")
    frequencies, pxx = signal.welch(u, fs=fs, nperseg=segment)
    _, pxy = signal.csd(u, y, fs=fs, nperseg=segment)
    _, coherence = signal.coherence(u, y, fs=fs, nperseg=segment)
    # Ignore numerical-noise bins where the input has no excitation; dividing
    # by those bins produces implausibly large gains for a pure sine.
    valid = ((frequencies > 0.0) &
             (pxx > max(float(np.max(pxx)) * 1e-2, np.finfo(float).eps)))
    if not np.any(valid):
        raise ValueError("input contains no measurable excitation")
    frequencies = frequencies[valid]
    pxx = pxx[valid]
    pxy = pxy[valid]
    coherence = coherence[valid]
    response = pxy / pxx
    phase_deg = np.degrees(np.unwrap(np.angle(response)))
    return {
        "input_signal": in_col,
        "output_signal": out_col,
        "sampling_rate_hz": fs,
        "frequency_hz": frequencies.tolist(),
        "magnitude": np.abs(response).tolist(),
        "magnitude_db": (20.0 * np.log10(np.maximum(np.abs(response),
                                                     np.finfo(float).eps))).tolist(),
        "phase_deg": phase_deg.astype(float).tolist(),
        "coherence": coherence.tolist(),
        "data": {
            "frequency_hz": frequencies.tolist(),
            "magnitude": np.abs(response).tolist(),
            "phase_deg": phase_deg.tolist(),
            "coherence": coherence.tolist(),
        },
    }


def analyze_frequency(df, input_col=None, output_col=None, sampling_rate_hz=None):
    return analyze_frequency_response(df, input_col, output_col, sampling_rate_hz)


def load_dynamic_csv(path):
    """Read a collector CSV without applying calibration-specific filtering."""
    csv_path = Path(path)
    if not csv_path.exists():
        raise FileNotFoundError(str(csv_path))
    return pd.read_csv(csv_path)


def analyze_dynamic(df, mode=None, input_col=None, output_col=None,
                    sampling_rate_hz=None):
    """Dispatch a CSV to time or frequency analysis based on its case mode."""
    mode_name = str(mode or "").lower().replace("-", "_").replace(" ", "_")
    if not mode_name and "mode" in df.columns and len(df):
        mode_name = str(df["mode"].iloc[0]).lower()
    if mode_name in ("step",):
        return {"kind": "step", "result": analyze_step_response(df, input_col, output_col)}
    if mode_name in ("single_sine", "chirp", "sweep", "multi_sine"):
        return {"kind": "frequency",
                "result": analyze_frequency_response(df, input_col, output_col,
                                                     sampling_rate_hz)}
    # A generic profile is still useful even if mode metadata was omitted.
    return {"kind": "step", "result": analyze_step_response(df, input_col, output_col)}


def dynamic_plot_data(df, mode=None, input_col=None, output_col=None,
                      sampling_rate_hz=None):
    """Return plotting-ready dictionaries without requiring Plotly or matplotlib."""
    return analyze_dynamic(df, mode, input_col, output_col, sampling_rate_hz)
