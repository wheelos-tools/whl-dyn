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
    for quality_column in ("sources_fresh", "localization_signals_valid"):
        if quality_column in working:
            quality = working[quality_column]
            if quality.dtype != bool:
                quality = quality.astype(str).str.lower().isin(
                    ("true", "1", "yes"))
            if not bool(quality.all()):
                raise ValueError(
                    "quality gate failed for {0}".format(quality_column))
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


def analyze_phase1_suite(run_root, expected_plan=None):
    """Return a strict machine-readable result for every Phase 1 run."""

    if not expected_plan:
        raise ValueError("Phase 1 report requires the expected plan")
    root = Path(run_root)
    run_paths = sorted(path for path in root.iterdir()
                       if path.is_dir() and (path / "samples.csv").exists())
    if not run_paths:
        raise ValueError("Phase 1 report requires at least one collected run")
    expected_names = {
        str(case["case_name"]) for case in (expected_plan or [])
        if isinstance(case, dict) and case.get("case_name")
    }
    actual_names = set()
    actual_name_list = []
    results = []
    for run_path in run_paths:
        status_path = run_path / "status.json"
        if not status_path.exists():
            raise ValueError("run is missing status.json: {0}".format(run_path))
        status = json.loads(status_path.read_text())
        if not status.get("completed") or status.get("abort_reason"):
            result = {"run": run_path.name, "status": "FAIL",
                      "reason": status.get("abort_reason") or "incomplete"}
        else:
            frame = pd.read_csv(run_path / "samples.csv")
            try:
                import yaml
                metadata = yaml.safe_load(
                    (run_path / "metadata.yaml").read_text()) or {}
                case = metadata.get("case", {})
                test_type = case.get("test_type", "")
                case_name = str(case.get("case_name", ""))
                actual_names.add(case_name)
                actual_name_list.append(case_name)
                required = ("steering_command", "steering_feedback",
                            "yaw_rate_radps", "lateral_accel_mps2")
                missing = [column for column in required if column not in frame]
                if missing:
                    raise ValueError("missing required signals: {0}".format(
                        ", ".join(missing)))
                if "time_aligned" in frame and not bool(
                        frame["time_aligned"].astype(str).str.lower().isin(
                            ("true", "1", "yes")).all()):
                    raise ValueError("time alignment quality gate failed")
                if ("localization_signals_valid" in frame and
                        not bool(frame["localization_signals_valid"].astype(
                            str).str.lower().isin(("true", "1", "yes")).all())):
                    raise ValueError("localization validity gate failed")
                for age_column in (
                        "localization_age_sec", "chassis_age_sec"):
                    if age_column in frame:
                        ages = pd.to_numeric(frame[age_column], errors="coerce")
                        if not np.isfinite(ages.to_numpy()).all() or (
                                ages > 0.5).any():
                            raise ValueError(
                                "source freshness gate failed: {0}".format(
                                    age_column))
                numeric = frame.loc[:, required].apply(pd.to_numeric, errors="coerce")
                if not np.isfinite(numeric.to_numpy(dtype=float)).all():
                    raise ValueError("required signals contain non-finite samples")
                expected_samples = int(float(case.get("duration_sec", 0.0)) *
                                      float(case.get("sampling_rate_hz", 0.0)) * 0.8)
                if len(frame) < max(4, expected_samples):
                    raise ValueError("insufficient samples: {0} < {1}".format(
                        len(frame), expected_samples))
                time_column = "elapsed_sec" if "elapsed_sec" in frame else "sample_time_sec"
                if time_column not in frame:
                    raise ValueError("samples require a time column")
                elapsed = pd.to_numeric(frame[time_column], errors="coerce")
                if not np.isfinite(elapsed.to_numpy()).all() or (
                        np.diff(elapsed.to_numpy()) < 0.0).any():
                    raise ValueError("sample time is invalid")
                gate = case.get("speed_gate", {})
                if "chassis_speed_mps" in frame and gate:
                    speed = pd.to_numeric(frame["chassis_speed_mps"],
                                          errors="coerce")
                    if not np.isfinite(speed.to_numpy()).all():
                        raise ValueError("speed contains non-finite samples")
                    if ((speed < float(gate["min_mps"])) |
                            (speed > float(gate["max_mps"]))).any():
                        raise ValueError("speed hard gate failed")
                    if (abs(speed - float(gate["target_mps"])) >
                            float(gate["tolerance_mps"])).any():
                        raise ValueError("speed target gate failed")
                max_lateral_accel = case.get("safety_limits", {}).get(
                    "max_abs_lateral_accel_mps2")
                if max_lateral_accel is not None:
                    lateral_accel = numeric["lateral_accel_mps2"]
                    if (abs(lateral_accel) > float(max_lateral_accel)).any():
                        raise ValueError("lateral acceleration safety limit failed")
                if test_type == "lateral_frequency_response":
                    metrics = analyze_lateral_frequency_response(frame)
                    for response in metrics["responses"].values():
                        coherence = np.asarray(response["coherence"], dtype=float)
                        if (not coherence.size or
                                not np.isfinite(coherence).all() or
                                float(np.min(coherence)) < 0.5):
                            raise ValueError("frequency coherence gate failed")
                else:
                    metrics = {"sample_count": len(frame)}
                result = {"run": run_path.name, "case_name": case_name,
                          "status": "PASS",
                          "test_type": test_type, "metrics": metrics}
            except (ValueError, KeyError, OSError) as error:
                result = {"run": run_path.name, "status": "FAIL",
                          "reason": str(error)}
        results.append(result)
    missing_cases = sorted(expected_names - actual_names)
    extra_cases = sorted(actual_names - expected_names)
    duplicate_cases = sorted({
        name for name in actual_name_list if actual_name_list.count(name) > 1
    })
    if missing_cases or extra_cases or duplicate_cases:
        results.append({"status": "FAIL", "reason": "missing cases",
                        "missing_cases": missing_cases,
                        "extra_cases": extra_cases,
                        "duplicate_cases": duplicate_cases})
    overall = "PASS" if (not missing_cases and not extra_cases and
                         not duplicate_cases and
                         all(item["status"] == "PASS" for item in results)) else "FAIL"
    return {"phase": "phase1", "status": overall, "run_count": len(results),
            "results": results}


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
