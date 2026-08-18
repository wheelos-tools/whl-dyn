"""Analysis and report artifacts for lateral vehicle frequency-response runs."""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from whl_dyn.processing.dynamics import analyze_frequency_response


def _actual_steering(frame, steering_column=None):
    if steering_column:
        if steering_column not in frame:
            raise ValueError("configured steering column is absent: {0}".format(
                steering_column))
        return steering_column, frame[steering_column]
    if "steering_feedback" in frame:
        return "steering_feedback", frame["steering_feedback"]
    front, rear = "front_steering_feedback", "rear_steering_feedback"
    if front in frame and rear in frame:
        frame["steering_feedback"] = 0.5 * (
            pd.to_numeric(frame[front], errors="coerce") +
            pd.to_numeric(frame[rear], errors="coerce"))
        return "steering_feedback", frame["steering_feedback"]
    if front in frame:
        return front, frame[front]
    raise ValueError(
        "samples require steering_feedback or front_steering_feedback")


def _bode_frame(result):
    return pd.DataFrame({
        "frequency_hz": result["frequency_hz"],
        "magnitude": result["magnitude"],
        "magnitude_db": result["magnitude_db"],
        "phase_deg": result["phase_deg"],
        "coherence": result["coherence"],
    })


def analyze_lateral_frequency_response(frame, steering_column=None,
                                       time_column="elapsed_sec",
                                       sampling_rate_hz=None):
    """Estimate steering-feedback to yaw-rate and lateral-acceleration FRFs."""

    if time_column not in frame:
        raise ValueError("samples require {0}".format(time_column))
    if "time_aligned" in frame:
        aligned_flag = frame["time_aligned"]
        if aligned_flag.dtype == bool:
            mask = aligned_flag
        else:
            mask = aligned_flag.astype(str).str.lower().isin(("true", "1", "yes"))
        working = frame.loc[mask].copy()
        if working.empty:
            raise ValueError("no time-aligned samples remain")
    else:
        working = frame.copy()
    input_name, steering = _actual_steering(working, steering_column)
    working["steering_feedback"] = pd.to_numeric(steering, errors="coerce")
    outputs = {
        "yaw_rate": "yaw_rate_radps",
        "lateral_acceleration": "lateral_accel_mps2",
    }
    reports = {}
    for name, output_column in outputs.items():
        if output_column not in working:
            raise ValueError("samples require {0}".format(output_column))
        result = analyze_frequency_response(
            working.rename(columns={time_column: "time"}),
            input_col="steering_feedback",
            output_col=output_column,
            sampling_rate_hz=sampling_rate_hz,
        )
        reports[name] = result
    return {
        "input_signal": input_name,
        "responses": reports,
    }


def _save_bode_plot(bode, title, output_path):
    figure, (gain_axis, phase_axis) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    frequency = bode["frequency_hz"]
    gain_axis.semilogx(frequency, bode["magnitude_db"])
    gain_axis.set_ylabel("Gain (dB)")
    gain_axis.grid(True, which="both")
    gain_axis.set_title(title)
    phase_axis.semilogx(frequency, bode["phase_deg"])
    phase_axis.set_xlabel("Frequency (Hz)")
    phase_axis.set_ylabel("Phase (deg)")
    phase_axis.grid(True, which="both")
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def write_lateral_frequency_report(run_directory, steering_column=None,
                                   sampling_rate_hz=None):
    """Generate metrics, Bode tables and PNGs beside a collected run."""

    run_path = Path(run_directory)
    samples_path = run_path / "samples.csv"
    if not samples_path.exists():
        raise FileNotFoundError(str(samples_path))
    report = analyze_lateral_frequency_response(
        pd.read_csv(samples_path),
        steering_column=steering_column,
        sampling_rate_hz=sampling_rate_hz,
    )
    output = run_path / "analysis"
    output.mkdir(exist_ok=True)
    summary = {"input_signal": report["input_signal"], "responses": {}}
    for name, result in report["responses"].items():
        _bode_frame(result).to_csv(output / "bode_{0}.csv".format(name), index=False)
        _save_bode_plot(result, "Steering to {0}".format(name.replace("_", " ")),
                        output / "bode_{0}.png".format(name))
        summary["responses"][name] = {
            key: result[key] for key in (
                "input_signal", "output_signal", "sampling_rate_hz",
                "bandwidth_hz", "resonance_peak_db", "resonance_peak_hz",
                "estimated_delay_sec",
            )
        }
    with (output / "metrics.json").open("w") as metrics_file:
        json.dump(summary, metrics_file, indent=2, sort_keys=True)
    return output, summary
