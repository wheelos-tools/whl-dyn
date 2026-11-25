import sys
import time
from types import ModuleType, SimpleNamespace

import numpy as np
import pandas as pd
import pytest
import yaml

from whl_dyn.collection.lateral import (
    LateralSignalCollector,
    localization_signals,
    nested_value,
)
from whl_dyn.collection.run_storage import RunStorage
from whl_dyn.planning.vehicle_dynamics import (
    LateralFrequencyPlanConfig,
    generate_lateral_frequency_plan,
)
from whl_dyn.planning.handling import generate_phase1_plan
from whl_dyn.collection.collector import evaluate_command_profile
from whl_dyn.processing.lateral_dynamics import (
    analyze_phase1_suite,
    analyze_lateral_frequency_response,
    write_lateral_frequency_report,
)


def test_lateral_plan_is_generic_and_has_required_outputs(tmp_path):
    output = tmp_path / "prbs.yaml"
    plan = generate_lateral_frequency_plan(LateralFrequencyPlanConfig(
        output=str(output), mode="prbs", speed_min_mps=0.0, speed_max_mps=3.0,
    ))
    case = plan[0]
    assert case["domain"] == "vehicle_dynamics"
    assert case["input_signals"] == ["steering_command", "steering_feedback"]
    assert case["output_signals"] == ["yaw_rate_radps", "lateral_accel_mps2"]
    assert case["command_profile"]["type"] == "prbs"
    speed_gate = yaml.safe_load(output.read_text())[0]["speed_gate"]
    assert speed_gate == {
        "min_mps": 0.0,
        "max_mps": 3.0,
        "target_mps": 2.0,
        "tolerance_mps": 0.15,
        "stable_duration_sec": 3.0,
        "max_wait_sec": 30.0,
    }


def test_lateral_plan_rejects_profiles_above_steering_rate_limit():
    try:
        generate_lateral_frequency_plan(
            mode="prbs", steering_amplitude=5.0, bit_duration_sec=0.25,
            max_steering_rate=30.0, output="")
    except ValueError as error:
        assert "requires" in str(error)
    else:
        raise AssertionError("unsafe PRBS plan was accepted")


def test_speed_gate_accepts_only_configured_range():
    collector = LateralSignalCollector(None, {})
    collector._latest["chassis_speed_mps"] = 2.5
    assert collector._speed_in_gate({"min_mps": 0.0, "max_mps": 3.0})
    collector._latest["chassis_speed_mps"] = 3.1
    assert not collector._speed_in_gate({"min_mps": 0.0, "max_mps": 3.0})


def test_chassis_feedback_satisfies_active_source_readiness():
    collector = LateralSignalCollector(
        None,
        {
            "chassis_fields": {"steering_feedback": "steering_percentage"},
            "max_feedback_age_sec": 0.5,
        },
    )
    now = time.time()
    collector._latest.update({
        "localization_source_time_sec": now,
        "chassis_source_time_sec": now,
        "steering_feedback": 2.0,
    })
    assert collector.wait_for_sources(0.01, require_steering_feedback=True)
    assert collector.snapshot()["time_aligned"]


def test_future_source_is_not_ready_for_active_collection():
    collector = LateralSignalCollector(
        None, {"chassis_fields": {"steering_feedback": "steering_percentage"}})
    now = time.time()
    collector._latest.update({
        "localization_source_time_sec": now,
        "chassis_source_time_sec": now + 1.0,
        "steering_feedback": 2.0,
    })
    assert not collector.wait_for_sources(0.01, require_steering_feedback=True)


def test_steering_preparation_uses_actual_interval_and_checks_final_target(monkeypatch):
    collector = LateralSignalCollector(None, {})
    clock = {"seconds": 0.0}
    commands = []
    monkeypatch.setattr(
        "whl_dyn.collection.lateral.time.monotonic", lambda: clock["seconds"])
    monkeypatch.setattr(
        "whl_dyn.collection.lateral.time.sleep",
        lambda seconds: clock.__setitem__("seconds", clock["seconds"] + seconds + 0.001))
    monkeypatch.setattr(
        collector, "publish_control",
        lambda command, *_args, **_kwargs: (
            commands.append((clock["seconds"], command)),
            clock.__setitem__("seconds", clock["seconds"] + 0.003)))
    collector.prepare_steering_before_speed(
        {"target": 10.0, "ramp_end_sec": 2.0}, 100.0, 10.0, 5.0)
    assert commands[-1][1] == 10.0
    for (t1, c1), (t2, c2) in zip(commands, commands[1:]):
        assert abs(c2 - c1) / (t2 - t1) <= 5.0 + 1e-9
    with pytest.raises(RuntimeError, match="configured rate"):
        collector.prepare_steering_before_speed(
            {"target": 10.0, "ramp_end_sec": 1.0}, 100.0, 10.0, 5.0)


def test_snapshot_time_is_taken_while_holding_the_data_lock(monkeypatch):
    class Lock:
        held = False

        def __enter__(self):
            self.held = True

        def __exit__(self, *_args):
            self.held = False

    collector = LateralSignalCollector(None, {})
    collector._lock = Lock()
    collector._latest["chassis_source_time_sec"] = 10.0

    def clock():
        assert collector._lock.held
        return 10.1

    monkeypatch.setattr("whl_dyn.collection.lateral.time.time", clock)
    monkeypatch.setattr("whl_dyn.collection.lateral.time.monotonic", clock)
    assert collector.snapshot()["chassis_age_sec"] == pytest.approx(0.1)


def test_callbacks_retain_per_source_receive_times(monkeypatch):
    wall = iter([10.0, 11.0, 12.0])
    monotonic = iter([20.0, 21.0, 22.0])
    monkeypatch.setattr("whl_dyn.collection.lateral.time.time", lambda: next(wall))
    monkeypatch.setattr("whl_dyn.collection.lateral.time.monotonic", lambda: next(monotonic))
    collector = LateralSignalCollector(None, {
        "chassis_fields": {"steering_feedback": "steering_percentage"}})
    collector._on_chassis(SimpleNamespace(
        header=SimpleNamespace(timestamp_sec=9.9),
        speed_mps=1.0, driving_mode=1, steering_percentage=2.0))
    collector._on_localization(SimpleNamespace(
        measurement_time=10.9,
        pose=SimpleNamespace(
            linear_velocity=SimpleNamespace(x=1.0, y=0.0),
            angular_velocity_vrf=SimpleNamespace(z=0.1),
            linear_acceleration_vrf=SimpleNamespace(x=0.2),
            heading=0.0)))
    sample = collector.snapshot()
    assert sample["chassis_received_time_sec"] == 10.0
    assert sample["chassis_received_monotonic_sec"] == 20.0
    assert sample["localization_received_time_sec"] == 11.0
    assert sample["localization_received_monotonic_sec"] == 21.0
    assert sample["steering_feedback_source_time_sec"] == 9.9
    assert not sample["steering_feedback_source_time_is_receive"]


def test_target_speed_must_be_inside_hard_speed_range():
    try:
        generate_lateral_frequency_plan(
            target_speed_mps=3.1, speed_min_mps=0.0, speed_max_mps=3.0,
            output="")
    except ValueError as error:
        assert "target speed" in str(error)
    else:
        raise AssertionError("out-of-range speed target was accepted")


def test_speed_target_requires_its_own_tolerance_band():
    collector = LateralSignalCollector(None, {})
    collector._latest["chassis_speed_mps"] = 2.14
    assert collector._speed_at_target({"target_mps": 2.0, "tolerance_mps": 0.15})
    collector._latest["chassis_speed_mps"] = 2.16
    assert not collector._speed_at_target({"target_mps": 2.0, "tolerance_mps": 0.15})


def test_speed_stability_checks_window_before_pruning_old_samples(monkeypatch):
    collector = LateralSignalCollector(None, {})
    clock = {"seconds": 0.0}
    monkeypatch.setattr(
        "whl_dyn.collection.lateral.time.monotonic",
        lambda: clock["seconds"],
    )
    monkeypatch.setattr(
        "whl_dyn.collection.lateral.time.sleep",
        lambda seconds: clock.__setitem__("seconds", clock["seconds"] + seconds),
    )
    monkeypatch.setattr(collector, "publish_control", lambda *args, **kwargs: None)
    monkeypatch.setattr(collector, "snapshot", lambda: {"chassis_speed_mps": 2.0})

    stable_speed = collector.wait_for_speed_stable(
        {
            "min_mps": 1.85,
            "max_mps": 2.15,
            "target_mps": 2.0,
            "tolerance_mps": 0.15,
            "stable_duration_sec": 2.0,
        },
        longitudinal={"mode": "speed", "speed_mps": 2.0},
        timeout_sec=5.0,
    )

    assert stable_speed == 2.0


def test_snapshot_reports_source_alignment_and_requires_feedback_source():
    collector = LateralSignalCollector(None, {"max_alignment_skew_sec": 0.02})
    collector._latest.update({
        "localization_source_time_sec": 10.000,
        "chassis_source_time_sec": 10.005,
        "chassis_detail_source_time_sec": 10.010,
        "steering_feedback": 0.1,
    })
    aligned = collector.snapshot()
    assert aligned["time_aligned"]
    assert np.isclose(aligned["alignment_skew_sec"], 0.01)

    collector._latest["chassis_detail_source_time_sec"] = 10.050
    not_aligned = collector.snapshot()
    assert not not_aligned["time_aligned"]
    assert np.isclose(not_aligned["alignment_skew_sec"], 0.05)


def test_lateral_control_defaults_to_drive_gear(monkeypatch):
    class ControlCommand:
        def __init__(self):
            self.header = SimpleNamespace()
            self.pad_msg = SimpleNamespace()

    wheelos_msgs = ModuleType("wheelos_msgs")
    wheelos_msgs.__path__ = []
    control_msgs = ModuleType("wheelos_msgs.control_msgs")
    control_msgs.__path__ = []
    control_cmd_pb2 = ModuleType(
        "wheelos_msgs.control_msgs.control_cmd_pb2")
    control_cmd_pb2.ControlCommand = ControlCommand
    control_msgs.control_cmd_pb2 = control_cmd_pb2
    monkeypatch.setitem(sys.modules, "wheelos_msgs", wheelos_msgs)
    monkeypatch.setitem(sys.modules, "wheelos_msgs.control_msgs", control_msgs)
    monkeypatch.setitem(
        sys.modules, "wheelos_msgs.control_msgs.control_cmd_pb2",
        control_cmd_pb2)

    writer = SimpleNamespace(messages=[])
    writer.write = writer.messages.append
    collector = LateralSignalCollector(None, {})
    collector._writer = writer
    collector.publish_control(0.0, speed_target_mps=1.0)

    assert writer.messages[0].gear_location == 1


def test_timestamped_storage_does_not_overwrite_same_case(tmp_path):
    first = RunStorage(tmp_path, "chirp", {"run": 1})
    second = RunStorage(tmp_path, "chirp", {"run": 2})
    assert first.path != second.path
    assert first.path.exists()
    first.write_samples([{"elapsed_sec": 0.0, "steering_feedback": 0.0}])
    assert first.samples_path.exists()


def test_localization_signal_normalization_uses_vehicle_frame_axes():
    message = SimpleNamespace(
        measurement_time=12.0,
        pose=SimpleNamespace(
            linear_velocity=SimpleNamespace(x=3.0, y=4.0),
            angular_velocity_vrf=SimpleNamespace(z=0.2),
            linear_acceleration_vrf=SimpleNamespace(x=-1.5),
            heading=0.0,
            euler_angles=SimpleNamespace(x=0.1, y=0.2),
        ),
    )
    assert nested_value(message, "pose.angular_velocity_vrf.z") == 0.2
    values = localization_signals(message, 99.0)
    assert values["localization_signals_valid"]
    assert values["speed_mps"] == 5.0
    assert values["longitudinal_velocity_mps"] == 3.0
    assert values["lateral_velocity_mps"] == 4.0
    assert values["yaw_rate_radps"] == 0.2
    assert values["lateral_accel_mps2"] == 1.5
    assert values["roll_rad"] == 0.2


def test_localization_signal_normalization_rejects_missing_dynamics_fields():
    message = SimpleNamespace(
        measurement_time=12.0,
        pose=SimpleNamespace(
            linear_velocity=SimpleNamespace(x=3.0, y=4.0),
            angular_velocity_vrf=SimpleNamespace(),
            linear_acceleration_vrf=SimpleNamespace(x=-1.5),
            heading=0.0,
        ),
    )
    values = localization_signals(message, 99.0)
    assert not values["localization_signals_valid"]
    assert np.isnan(values["yaw_rate_radps"])


def test_lateral_analysis_writes_two_bode_reports(tmp_path):
    sample_rate = 100.0
    elapsed = np.arange(0.0, 30.0, 1.0 / sample_rate)
    random = np.random.RandomState(7)
    steering = random.choice((-1.0, 1.0), size=len(elapsed))
    samples = pd.DataFrame({
        "elapsed_sec": elapsed,
        "steering_feedback": steering,
        "yaw_rate_radps": 0.25 * steering,
        "lateral_accel_mps2": 0.8 * steering,
    })
    report = analyze_lateral_frequency_response(
        samples, sampling_rate_hz=sample_rate)
    assert set(report["responses"]) == {"yaw_rate", "lateral_acceleration"}

    run = tmp_path / "run"
    run.mkdir()
    samples.to_csv(run / "samples.csv", index=False)
    output, metrics = write_lateral_frequency_report(
        run, sampling_rate_hz=sample_rate)
    assert (output / "bode_yaw_rate.csv").exists()
    assert (output / "bode_lateral_acceleration.png").exists()
    assert "bandwidth_hz" in metrics["responses"]["yaw_rate"]


def test_lateral_frequency_uses_independent_source_times_and_bounds_gaps():
    rng = np.random.RandomState(3)
    t = np.arange(2000) * 0.01
    steering_time = t + rng.uniform(-0.0007, 0.0007, len(t))
    motion_time = t + rng.uniform(-0.0007, 0.0007, len(t)) + 0.002
    frame = pd.DataFrame({
        "elapsed_sec": t,
        "sample_elapsed_sec": t + rng.uniform(-0.001, 0.001, len(t)),
        "steering_feedback_source_time_sec": steering_time + 1000.0,
        "localization_source_time_sec": motion_time + 1000.0,
        "steering_feedback": np.sin(2 * np.pi * steering_time),
        "yaw_rate_radps": 0.5 * np.sin(2 * np.pi * motion_time - 0.2),
        "lateral_accel_mps2": np.sin(2 * np.pi * motion_time - 0.2),
    })
    result = analyze_lateral_frequency_response(frame, sampling_rate_hz=100.0)
    yaw = result["responses"]["yaw_rate"]
    peak = np.argmin(np.abs(np.asarray(yaw["frequency_hz"]) - 1.0))
    assert yaw["magnitude"][peak] == pytest.approx(0.5, abs=0.02)
    assert yaw["phase_deg"][peak] == pytest.approx(-np.degrees(0.2), abs=1.0)
    assert yaw["timebase"]["method"] == "linear_interpolation_of_source_timestamps"
    with pytest.raises(ValueError, match="gap exceeding"):
        analyze_lateral_frequency_response(frame.drop(index=100))
    with pytest.raises(ValueError, match="anti-alias"):
        analyze_lateral_frequency_response(frame, sampling_rate_hz=50.0)
    repeated = frame.copy()
    repeated.loc[10, "steering_feedback_source_time_sec"] = repeated.loc[
        9, "steering_feedback_source_time_sec"]
    with pytest.raises(ValueError, match="conflicting"):
        analyze_lateral_frequency_response(repeated)
    fallback = frame.copy()
    fallback["localization_source_time_is_receive"] = True
    with pytest.raises(ValueError, match="receive-time fallbacks"):
        analyze_lateral_frequency_response(fallback)
    custom = frame.assign(other_steering=frame["steering_feedback"])
    with pytest.raises(ValueError, match="do not identify"):
        analyze_lateral_frequency_response(custom, steering_column="other_steering")


def test_lateral_frequency_checks_snapshot_time_for_legacy_logs():
    t = np.arange(100) * 0.01
    snapshot = t.copy()
    snapshot[50] += 0.002
    frame = pd.DataFrame({
        "elapsed_sec": t, "collector_monotonic_sec": snapshot,
        "localization_source_time_sec": 1000.0 + t,
        "steering_feedback": np.sin(t * 10),
        "yaw_rate_radps": np.sin(t * 10),
        "lateral_accel_mps2": np.sin(t * 10),
    })
    with pytest.raises(ValueError, match="uniformly sampled"):
        analyze_lateral_frequency_response(frame)


def test_lateral_frequency_reports_actual_source_rate_for_repeated_samples():
    elapsed = np.arange(1000) * 0.01
    source = np.floor(np.arange(1000) / 2) * 0.02
    steering = np.sin(2 * np.pi * source)
    frame = pd.DataFrame({
        "elapsed_sec": elapsed, "sample_elapsed_sec": elapsed,
        "steering_feedback_source_time_sec": 1000.0 + source,
        "localization_source_time_sec": 1000.0 + source,
        "steering_feedback": steering, "yaw_rate_radps": steering * 0.5,
        "lateral_accel_mps2": steering,
    })
    yaw = analyze_lateral_frequency_response(
        frame, sampling_rate_hz=100.0)["responses"]["yaw_rate"]
    assert yaw["sampling_rate_hz"] == pytest.approx(50.0)
    assert yaw["timebase"]["source_sample_counts"] == [500, 500]


def test_lateral_analysis_rejects_unaligned_samples():
    frame = pd.DataFrame({
        "elapsed_sec": [0.0, 0.01, 0.02, 0.03],
        "steering_feedback": [0.0, 1.0, 0.0, -1.0],
        "yaw_rate_radps": [0.0, 0.2, 0.0, -0.2],
        "lateral_accel_mps2": [0.0, 0.5, 0.0, -0.5],
        "time_aligned": [False, False, False, False],
    })
    try:
        analyze_lateral_frequency_response(frame, sampling_rate_hz=100.0)
    except ValueError as error:
        assert "time-aligned" in str(error)
    else:
        raise AssertionError("unaligned samples were accepted")


def test_phase1_report_requires_expected_cases_and_quality(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    metadata = {
        "case": {
            "case_name": "step",
            "test_type": "steering_step",
            "duration_sec": 1.0,
            "sampling_rate_hz": 10.0,
        },
    }
    (run / "metadata.yaml").write_text(yaml.safe_dump(metadata))
    (run / "status.json").write_text('{"completed": true}')
    pd.DataFrame({
        "steering_command": [0.0] * 10,
        "steering_feedback": [0.0] * 10,
        "yaw_rate_radps": [0.0] * 10,
        "lateral_accel_mps2": [0.0] * 10,
        "time_aligned": [True] * 10,
        "localization_signals_valid": [True] * 10,
    }).to_csv(run / "samples.csv", index=False)
    report = analyze_phase1_suite(
        tmp_path, expected_plan=[metadata["case"],
                                dict(metadata["case"], case_name="missing")])
    assert report["status"] == "FAIL"
    assert report["results"][-1]["missing_cases"] == ["missing"]


def _write_phase1_run(root, case, mutation=None):
    fs = case["sampling_rate_hz"]
    elapsed = np.arange(0, case["duration_sec"], 1.0 / fs)
    command = np.array([evaluate_command_profile(case["command_profile"], t)
                        for t in elapsed])
    if case["test_type"] == "steering_step":
        profile = case["command_profile"]
        response = profile["baseline"] + profile["amplitude"] * (
            1.0 - np.exp(-np.maximum(elapsed - profile["start_time_sec"], 0.0) / 0.2))
    else:
        response = command
    frame = pd.DataFrame({
        "elapsed_sec": elapsed, "steering_command": command,
        "steering_feedback": 0.5 * response,
        "yaw_rate_radps": 0.02 * response, "lateral_accel_mps2": 0.1 * response,
        "chassis_speed_mps": case["speed_gate"]["target_mps"],
        "time_aligned": True, "localization_signals_valid": True,
        "localization_age_sec": 0.01, "chassis_age_sec": 0.01,
    })
    if mutation:
        mutation(frame)
    run = root / case["case_name"]
    run.mkdir()
    frame.to_csv(run / "samples.csv", index=False)
    (run / "metadata.yaml").write_text(yaml.safe_dump({"case": case}))
    (run / "status.json").write_text('{"completed": true}')


def test_phase1_reports_both_directions_step_ramp_and_frequency(tmp_path):
    plan = generate_phase1_plan(output="", amplitude=2.0)
    for case in plan:
        _write_phase1_run(tmp_path, case)
    report = analyze_phase1_suite(tmp_path, expected_plan=plan)
    assert report["status"] == "PASS", report
    assert report["run_count"] == 8
    for result in report["results"]:
        assert result["vehicle_acceptance"] == "NOT_EVALUATED"
        if result["test_type"] == "steering_step":
            assert result["metrics"]["steering_feedback"]["time_constant_sec"] == pytest.approx(0.2, abs=0.02)
        if result["test_type"] == "steering_deadzone_rate":
            assert result["metrics"]["deadzone"] is None
            assert result["metrics"]["peak_observed_feedback_rate"] > 0


def test_phase1_rejects_source_rate_below_plan(tmp_path):
    case = generate_phase1_plan(output="", amplitude=2.0)[-1]

    def use_half_rate_source(frame):
        source = np.floor(np.arange(len(frame)) / 2) * 0.02
        feedback = np.sin(2 * np.pi * source)
        frame["steering_feedback_source_time_sec"] = 1000.0 + source
        frame["localization_source_time_sec"] = 1000.0 + source
        frame["steering_feedback"] = feedback
        frame["yaw_rate_radps"] = 0.1 * feedback
        frame["lateral_accel_mps2"] = 0.2 * feedback

    _write_phase1_run(tmp_path, case, use_half_rate_source)
    report = analyze_phase1_suite(tmp_path, expected_plan=[case])
    assert report["status"] == "FAIL"
    assert "effective frequency sample rate" in report["results"][0]["reason"]


@pytest.mark.parametrize("column", [
    "steering_command", "steering_feedback", "yaw_rate_radps", "lateral_accel_mps2",
])
def test_phase1_rejects_missing_excitation_or_response(tmp_path, column):
    case = generate_phase1_plan(output="", amplitude=2.0)[0]
    _write_phase1_run(tmp_path, case, lambda frame: frame.__setitem__(column, 0.0))
    report = analyze_phase1_suite(tmp_path, expected_plan=[case])
    assert report["status"] == "FAIL"
    assert ("excitation" in report["results"][0]["reason"] or
            "no measurable response" in report["results"][0]["reason"])


def test_phase1_does_not_trust_modified_recorded_plan(tmp_path):
    case = generate_phase1_plan(output="", amplitude=2.0)[0]
    _write_phase1_run(tmp_path, case)
    expected = dict(case, duration_sec=100.0)
    report = analyze_phase1_suite(tmp_path, expected_plan=[expected])
    assert report["status"] == "FAIL"
    assert "expected plan" in report["results"][0]["reason"]
