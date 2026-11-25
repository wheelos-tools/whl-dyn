import numpy as np
import json
import pandas as pd
import pytest

from whl_dyn.processing.config import CalibrationConfig
from whl_dyn.processing.data_core import DataCore
from whl_dyn.processing.exporter import Exporter
from whl_dyn.processing.metrics import MetricsEvaluator


def _write_constant_command_file(path, t0, cmd_val, n=60, speed=1.0,
                                 sampling_rate=10.0):
    """Write a synthetic constant-command calibration run CSV."""
    rows = []
    for i in range(n):
        rows.append(dict(
            time=t0 + i / sampling_rate,
            speed_mps=speed, ins_speed_mps=speed, imu_accel_y=cmd_val * 0.01,
            driving_mode=1, actual_gear=1, throttle_pct=0, brake_pct=0,
            ctl_throttle=max(cmd_val, 0), ctl_brake=max(-cmd_val, 0),
        ))
    pd.DataFrame(rows).to_csv(path, index=False)


def test_process_signals_does_not_leak_across_source_files(tmp_path):
    """Two independent recording sessions must stay independent.

    Regression test for a bug where the low-pass filter and latency-sync
    shift were applied to the whole time-sorted, multi-file frame instead of
    per source_file, letting samples from an unrelated later run bleed into
    the tail of an earlier run (and vice versa) purely because they landed
    next to each other after the global time sort.
    """
    config = CalibrationConfig(
        sampling_rate=10.0, throttle_latency_ms=200, brake_latency_ms=200,
        throttle_stability_window_ms=0, brake_stability_window_ms=0,
        enable_lof=False, lowpass_cutoff=4.0,
    )
    _write_constant_command_file(tmp_path / "a.csv", t0=0.0, cmd_val=20)
    _write_constant_command_file(tmp_path / "b.csv", t0=1000.0, cmd_val=40)

    core = DataCore(config)
    core.load_data(str(tmp_path))
    core.process_signals()
    df = core.processed_df

    for source_file, expected_accel in (("a.csv", 0.2), ("b.csv", 0.4)):
        file_df = df[df["source_file"] == source_file]
        assert (file_df["accel_filtered"].round(6) == expected_accel).all()
        # accel_aligned may legitimately be 0 near the shift edge (fillna),
        # but must never take on the *other* file's constant level.
        other_level = 0.4 if expected_accel == 0.2 else 0.2
        assert not (file_df["accel_aligned"].round(6) == other_level).any()


def test_calibration_table_does_not_extrapolate_unmeasured_speed(tmp_path):
    config = CalibrationConfig(
        speed_resolution=0.2, command_resolution=5.0,
    )
    core = DataCore(config)
    core.processed_df = pd.DataFrame([
        {"final_speed": 0.0, "command": -10.0, "accel_aligned": -1.0},
        {"final_speed": 1.0, "command": -10.0, "accel_aligned": -1.1},
        {"final_speed": 0.0, "command": 0.0, "accel_aligned": 0.0},
        {"final_speed": 2.0, "command": 0.0, "accel_aligned": 0.0},
        {"final_speed": 0.0, "command": 10.0, "accel_aligned": 1.0},
        {"final_speed": 2.0, "command": 10.0, "accel_aligned": 0.5},
    ])

    speed_grid, command_grid, grid_z = core.build_calibration_table()
    brake_row = grid_z[np.where(command_grid == -10.0)[0][0]]

    assert np.isfinite(brake_row[np.searchsorted(speed_grid, 0.8)])
    assert np.isnan(brake_row[np.searchsorted(speed_grid, 1.8)])


@pytest.mark.parametrize("commands", [[20.0, 40.0], [-40.0, -20.0],
                                     [-20.0, 0.0, 20.0]])
def test_calibration_preserves_net_acceleration_and_unmeasured_cells(commands):
    core = DataCore(CalibrationConfig(
        speed_resolution=0.5, command_resolution=20.0))
    core.processed_df = pd.DataFrame([
        {"final_speed": speed, "command": command,
         "accel_aligned": 0.01 * command - 0.4}
        for speed in (1.0, 2.0) for command in commands
    ])
    speeds, commands, acceleration = core.build_calibration_table()
    np.testing.assert_allclose(
        acceleration[:, np.where(speeds == 1.0)[0][0]], 0.01 * commands - 0.4)
    assert np.isnan(acceleration[:, speeds < 1.0]).all()


def test_control_export_preserves_zero_and_omits_unknown_cells(tmp_path):
    with pytest.warns(RuntimeWarning, match="Omitting 2"):
        Exporter.save_protobuf(
            [1.0, 2.0], [-20.0, 20.0],
            np.array([[-0.6, np.nan], [0.0, np.inf]]), tmp_path)
    text = (tmp_path / "calibration_table.pb.txt").read_text()
    assert "nan" not in text and "inf" not in text
    assert text.count("calibration {") == 2
    assert "acceleration: 0.0" in text
    with pytest.raises(ValueError, match="no measured"):
        Exporter.save_protobuf([1.0], [20.0], [[np.nan]], tmp_path)
    assert (tmp_path / "calibration_table.pb.txt").read_text() == text


def test_response_threshold_requires_measured_coast_baseline():
    speeds = np.array([1.0, 2.0, 3.0])
    commands = np.array([-20.0, 0.0, 20.0])
    grid = np.tile([-0.6, -0.4, -0.2], (3, 1)).T
    unknown = MetricsEvaluator.evaluate(speeds, commands, grid)
    assert unknown["throttle_deadzone_pct"] is None
    assert unknown["brake_deadzone_pct"] is None
    coast = pd.DataFrame({
        "final_speed": [1.0, 3.0], "command": [0.0, 0.0],
        "accel_aligned": [-0.4, -0.4],
    })
    measured = MetricsEvaluator.evaluate(speeds, commands, grid, coast)
    assert measured["throttle_deadzone_pct"] == 20.0
    assert measured["brake_deadzone_pct"] == 20.0


def test_aborted_recording_does_not_enter_calibration(tmp_path, capsys):
    path = tmp_path / "aborted.csv"
    _write_constant_command_file(path, 0.0, 20)
    path.with_suffix(".status.json").write_text(json.dumps({
        "completed": False, "abort_reason": "speed limit",
    }))
    core = DataCore(CalibrationConfig())
    core.load_data(tmp_path)
    assert not core.raw_dfs
    assert "did not complete successfully" in capsys.readouterr().out


def test_repeated_command_after_brake_gets_its_own_stability_window(tmp_path):
    command = np.repeat([20.0, -20.0, 20.0], 30)
    frame = pd.DataFrame({
        "time": np.arange(len(command)) / 10.0,
        "speed_mps": 1.0, "imu_accel_y": command / 100.0,
        "ctl_throttle": np.maximum(command, 0),
        "ctl_brake": np.maximum(-command, 0),
    })
    frame.to_csv(tmp_path / "repeated.csv", index=False)
    core = DataCore(CalibrationConfig(enable_lof=False, throttle_latency_ms=0,
                                     brake_latency_ms=0, throttle_stability_window_ms=200))
    core.load_data(tmp_path)
    core.process_signals()
    after_second_switch = core.processed_df[core.processed_df["time"] >= 6.0]
    assert after_second_switch["time"].min() >= 6.3


def test_calibration_interpolation_predicts_withheld_speed_with_drag():
    core = DataCore(CalibrationConfig(speed_resolution=0.5, command_resolution=10))
    core.processed_df = pd.DataFrame([
        {"final_speed": speed, "command": command,
         "accel_aligned": 0.02 * command - 0.1 * speed}
        for speed in (1.0, 3.0) for command in (-20.0, 20.0)
    ])
    speeds, commands, grid = core.build_calibration_table()
    np.testing.assert_allclose(
        grid[:, np.where(speeds == 2.0)[0][0]], 0.02 * commands - 0.2,
        atol=1e-12)


def test_filter_does_not_leak_between_command_segments(tmp_path):
    config = CalibrationConfig(
        sampling_rate=10.0, throttle_stability_window_ms=0,
        brake_stability_window_ms=0, enable_lof=False, lowpass_cutoff=4.0,
    )
    rows = []
    for i in range(60):
        throttle = i < 30
        rows.append({
            "time": i / 10.0, "speed_mps": 1.0, "ins_speed_mps": 1.0,
            "imu_accel_y": 1.0 if throttle else -1.0,
            "driving_mode": 1, "actual_gear": 1, "throttle_pct": 10 if throttle else 0,
            "brake_pct": 0 if throttle else 10,
            "ctl_throttle": 10 if throttle else 0,
            "ctl_brake": 0 if throttle else 10,
        })
    pd.DataFrame(rows).to_csv(tmp_path / "segments.csv", index=False)

    core = DataCore(config)
    core.load_data(str(tmp_path))
    core.process_signals()
    df = core.processed_df

    throttle = df[df["command"] == 10.0]
    brake = df[df["command"] == -10.0]
    assert throttle["accel_filtered"].iloc[-1] > 0.9
    assert brake["accel_filtered"].iloc[0] < -0.9


def test_processing_uses_log_rate_and_discards_startup_transition(tmp_path):
    """A 50 Hz log must not be processed with the 100 Hz fallback rate."""
    rows = []
    for i in range(80):
        command = 0 if i == 0 else 20
        rows.append({
            "time": i / 50.0, "speed_mps": i / 50.0,
            "ins_speed_mps": i / 50.0,
            "imu_accel_y": 0.0 if i == 0 else 0.8,
            "driving_mode": 1, "actual_gear": 1, "throttle_pct": command,
            "brake_pct": 0, "ctl_throttle": command, "ctl_brake": 0,
        })
    pd.DataFrame(rows).to_csv(tmp_path / "50hz.csv", index=False)

    core = DataCore(CalibrationConfig(
        sampling_rate=100.0, throttle_latency_ms=60,
        throttle_stability_window_ms=200, enable_lof=False,
    ))
    core.load_data(str(tmp_path))
    core.process_signals()

    raw = core.raw_dfs[0]
    assert raw["sample_rate_hz"].iloc[0] == pytest.approx(50.0)
    rate_check = core.validate_sampling_rate()
    assert rate_check["passed"] is False
    assert rate_check["files"][0]["measured_hz"] == pytest.approx(50.0)
    core.config.sampling_rate = 50.0
    assert core.validate_sampling_rate()["passed"] is True
    throttle = core.processed_df[core.processed_df["command"] == 20.0]
    # The initial command transition is excluded for 200 ms at the measured
    # 50 Hz rate, rather than 20 samples based on the wrong 100 Hz default.
    assert throttle["final_speed"].min() >= 0.17
    # 60 ms at 50 Hz is three samples; the aligned signal remains physical.
    assert throttle["accel_aligned"].median() == pytest.approx(0.8, abs=0.02)
