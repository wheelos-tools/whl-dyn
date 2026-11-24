import csv
import io
import json
from types import SimpleNamespace

import numpy as np
import pandas as pd
import pytest
from scipy import signal
import yaml

from whl_dyn.planning.generator import (
    DynamicPlanConfig,
    generate_calibration_plan,
    generate_dynamic_plan,
)
from whl_dyn.collection.collector import (
    AdvancedDataCollector,
    ControlState,
    VehicleState,
)
from whl_dyn.collection import collector as collector_module
from whl_dyn.planning.preflight import validate_longitudinal_plan
from whl_dyn.processing.dynamics import (
    analyze_dynamic,
    analyze_frequency_response,
    analyze_step,
    is_dynamic_log,
)


def test_dynamic_plan_contains_profile_and_can_be_loaded(tmp_path):
    path = tmp_path / "dynamic.yaml"
    plan = generate_dynamic_plan(DynamicPlanConfig(
        output=str(path), mode="sweep", actuator="brake", duration_sec=4.0,
        initial_speed_mps=2.0,
    ))
    assert plan[0]["domain"] == "frequency_response"
    assert plan[0]["command_profile"]["type"] == "sweep"
    assert yaml.safe_load(path.read_text())[0]["duration_sec"] == 4.0


def test_prbs_plan_contains_prbs_parameters():
    case = generate_dynamic_plan(mode="prbs", output="")[0]
    profile = case["command_profile"]
    assert profile["type"] == "prbs"
    assert "bit_duration_sec" in profile
    assert "prbs_seed" in profile


def test_all_supported_profile_types_are_generatable():
    profile_types = ("step", "ramp", "pulse", "triangle", "hysteresis",
                     "single_sine", "chirp", "sweep", "multi_sine", "prbs")
    for profile_type in profile_types:
        case = generate_dynamic_plan(mode=profile_type, output="")[0]
        assert case["command_profile"]["type"] == profile_type
        assert "speed_greater_than" not in str(case)


def _calibration_plan_args(output):
    return SimpleNamespace(
        output=str(output),
        throttle_min=10,
        throttle_max=20,
        throttle_num_steps=2,
        brake_min=20,
        brake_max=30,
        brake_num_steps=2,
        speed_targets=[0.6, 1.2],
        default_throttle=20.0,
        default_brake=30.0,
        hold_duration_ms=500,
        accel_timeout=20.0,
        decel_timeout=25.0,
        sampling_rate_hz=50.0,
    )


def test_calibration_plan_records_longitudinal_control_contract(tmp_path):
    output = tmp_path / "calibration.yaml"
    plan = generate_calibration_plan(_calibration_plan_args(output))

    assert len(plan) == 6
    assert len({case["case_name"] for case in plan}) == 6
    assert "throttle_10_to_0.6mps" in {case["case_name"] for case in plan}
    assert "brake_30_from_1.2mps" in {case["case_name"] for case in plan}
    assert all(case["domain"] == "longitudinal_calibration" for case in plan)
    assert all(case["gear_location"] == 1 for case in plan)
    assert all(case["sampling_rate_hz"] == 50.0 for case in plan)
    assert all(case["input_signals"] ==
               ["ctl_throttle", "ctl_brake", "ctl_gear"]
               for case in plan)
    assert all(
        all(step["command"]["throttle"] == 0.0 or
            step["command"]["brake"] == 0.0 for step in case["steps"])
        for case in plan
    )
    assert yaml.safe_load(output.read_text()) == plan
    assert ControlState().gear == 1


def test_calibration_plan_rejects_out_of_range_commands():
    args = _calibration_plan_args("")
    args.throttle_max = 101
    try:
        generate_calibration_plan(args)
    except ValueError as error:
        assert "throttle range" in str(error)
    else:
        raise AssertionError("out-of-range throttle was accepted")


def test_collector_logs_commanded_gear():
    collector = AdvancedDataCollector.__new__(AdvancedDataCollector)
    collector.active_case = {}
    collector.output_file = io.StringIO()
    collector.vehicle_state = VehicleState(
        timestamp=1.0, speed_mps=2.0, ins_speed_mps=2.0,
        driving_mode=1, actual_gear=1, throttle_pct=10.0, brake_pct=0.0)
    collector.last_sent_control = ControlState(
        throttle=10.0, brake=0.0, gear=1)

    collector._write_header()
    collector._write_log_entry()
    row = next(csv.DictReader(io.StringIO(collector.output_file.getvalue())))

    assert row["actual_gear"] == "1"
    assert row["ctl_gear"] == "1"
    assert row["ctl_throttle"] == "10.00"


def test_both_actuator_routes_negative_profile_to_brake():
    collector = AdvancedDataCollector.__new__(AdvancedDataCollector)
    collector.active_case = {
        "actuator": "both",
        "command_profile": {
            "type": "step",
            "baseline": 0.0,
            "amplitude": -100.0,
            "start_time_sec": 0.0,
        },
    }

    assert collector._dynamic_command(0.1) == {
        "throttle": 0.0,
        "brake": 100.0,
    }
    collector.active_case["command_profile"]["amplitude"] = -120.0
    with pytest.raises(ValueError, match="within"):
        collector._dynamic_command(0.1)

    collector.active_case["command_profile"]["amplitude"] = 20.0
    assert collector._dynamic_command(0.1) == {
        "throttle": 20.0,
        "brake": 0.0,
    }


def test_step_metrics_are_json_friendly():
    time = np.linspace(0.0, 10.0, 1001)
    command = np.where(time < 1.0, 0.0, 2.0)
    feedback = np.where(time < 1.0, 1.0,
                        1.0 + 3.0 * (1.0 - np.exp(-(time - 1.0) / 0.8)))
    result = analyze_step(pd.DataFrame({
        "time": time, "command": command, "feedback": feedback
    }))
    assert result["gain"] > 1.4
    assert result["rise_time_sec"] is not None
    assert "peak_time_sec" in result
    assert "overshoot_pct" in result
    assert "steady_state_error" in result
    assert isinstance(result["data"]["time"], list)


def test_frequency_analysis_returns_bode_arrays():
    sample_rate = 50.0
    time = np.arange(0.0, 20.0, 1.0 / sample_rate)
    command = np.sin(2.0 * np.pi * 1.0 * time)
    feedback = 0.5 * np.sin(2.0 * np.pi * 1.0 * time - 0.2)
    result = analyze_frequency_response(pd.DataFrame({
        "time": time, "command": command, "feedback": feedback
    }), nperseg=256)
    assert len(result["frequency_hz"]) > 0
    assert len(result["frequency_hz"]) == len(result["coherence"])
    assert abs(result["phase_deg"][0]) <= 180.0
    assert "bandwidth_hz" in result
    assert "resonance_peak_db" in result
    assert "estimated_delay_sec" in result
    peak = int(np.argmax(result["magnitude"]))
    assert result["magnitude"][peak] == pytest.approx(0.5, abs=0.02)
    assert result["phase_deg"][peak] == pytest.approx(-np.degrees(0.2), abs=1)
    assert result["bandwidth_hz"] is None


def test_dynamic_log_detection_uses_metadata():
    assert is_dynamic_log(pd.DataFrame({
        "mode": ["sweep"],
        "domain": ["frequency_response"],
    }))
    assert not is_dynamic_log(pd.DataFrame({"mode": ["calibration"]}))


def test_prbs_mode_dispatches_to_frequency():
    sample_rate = 20.0
    time = np.arange(0.0, 10.0, 1.0 / sample_rate)
    command = np.sign(np.sin(2.0 * np.pi * 0.8 * time))
    feedback = 0.3 * command
    result = analyze_dynamic(pd.DataFrame({
        "time": time, "command": command, "feedback": feedback, "mode": "prbs"
    }), mode="prbs")
    assert result["kind"] == "frequency"


@pytest.mark.parametrize("direction", [-1.0, 1.0])
def test_step_response_matches_known_model_in_both_directions(direction):
    time = np.arange(0.0, 12.0, 0.01)
    result = analyze_step(pd.DataFrame({
        "time": time, "command": (time >= 1.0).astype(float),
        "feedback": direction * (1.0 - np.exp(-np.maximum(time - 1.0, 0.0) / 0.5)),
    }))
    assert result["rise_time_sec"] == pytest.approx(0.5 * np.log(9), abs=0.02)
    assert result["settling_time_sec"] == pytest.approx(-0.5 * np.log(0.02), abs=0.02)
    assert result["time_constant_sec"] == pytest.approx(0.5, abs=0.01)
    assert result["fopdt"]["gain"] == pytest.approx(direction, abs=0.001)


@pytest.mark.parametrize("mode", ["ramp", "pulse", "triangle", "hysteresis", "typo"])
def test_non_step_profiles_never_receive_step_metrics(mode):
    with pytest.raises(ValueError, match="no quantitative analyzer"):
        analyze_dynamic(pd.DataFrame(), mode=mode)


def test_step_analysis_rejects_missing_response_and_multiple_steps():
    frame = pd.DataFrame({
        "time": np.arange(10.0), "command": [0.0] * 3 + [1.0] * 7,
        "feedback": np.zeros(10),
    })
    with pytest.raises(ValueError, match="no measurable"):
        analyze_step(frame)
    frame.loc[8:, "command"] = 0.0
    with pytest.raises(ValueError, match="single isolated"):
        analyze_step(frame)


def test_frequency_bandwidth_matches_known_lowpass_and_rejects_gaps():
    fs, tau = 100.0, 0.2
    time = np.arange(0.0, 300.0, 1.0 / fs)
    command = np.random.RandomState(4).normal(size=len(time))
    pole = np.exp(-1.0 / (fs * tau))
    feedback = signal.lfilter([1.0 - pole], [1.0, -pole], command)
    frame = pd.DataFrame({"time": time, "command": command, "feedback": feedback})
    result = analyze_frequency_response(frame, nperseg=4096)
    assert result["bandwidth_hz"] == pytest.approx(1.0 / (2.0 * np.pi * tau), rel=0.1)
    with pytest.raises(ValueError, match="uniformly sampled"):
        analyze_frequency_response(frame.drop(index=100))
    with pytest.raises(ValueError, match="uniformly sampled"):
        analyze_frequency_response(frame, sampling_rate_hz=50.0)
    with pytest.raises(ValueError, match="configured signal"):
        analyze_frequency_response(frame, output_col="missing_feedback")


@pytest.mark.parametrize("options", [
    {"duration_sec": 0}, {"duration_sec": -1}, {"duration_sec": np.nan},
    {"sampling_rate_hz": 0}, {"amplitude": np.inf},
    {"mode": "single_sine", "baseline": 0}, {"mode": "chirp", "frequency_end_hz": 30},
    {"mode": "prbs", "bit_duration_sec": 0},
    {"actuator": "brake"}, {"actuator": "unknown"},
])
def test_invalid_dynamic_plans_are_rejected(options):
    with pytest.raises(ValueError):
        generate_dynamic_plan(output="", **options)


@pytest.mark.parametrize("invalid", [
    {"command": {"throttle": 101}}, {"command": {"throttle": 20, "brake": 20}},
    {"command": {"throttle": float("nan")}}, {"timeout_sec": 0},
    {"trigger": {"type": "typo", "value": 1}}, {"hold_duration_ms": -1},
])
def test_manual_plans_receive_runtime_preflight(tmp_path, invalid):
    plan = generate_calibration_plan(_calibration_plan_args(""))
    plan[0]["steps"][0].update(invalid)
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(plan))
    collector = AdvancedDataCollector.__new__(AdvancedDataCollector)
    assert not collector._load_plan(str(path))
    assert collector.plan is None


@pytest.fixture
def longitudinal_runtime(monkeypatch, tmp_path):
    clock, published = [100.0], []
    monkeypatch.setattr(collector_module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(collector_module.time, "sleep", lambda duration: None)
    monkeypatch.setattr(collector_module, "cyber", SimpleNamespace(ok=lambda: True))
    monkeypatch.setattr(collector_module, "cyber_time", SimpleNamespace(
        Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: clock[0]))))
    node = SimpleNamespace(create_writer=lambda *args: SimpleNamespace(write=published.append))
    collector = AdvancedDataCollector(node, output_dir=str(tmp_path), auto_start=True)
    collector.vehicle_state = VehicleState(
        timestamp=clock[0], driving_mode=1, actual_gear=1)
    collector.localization_received = collector.chassis_received = True
    collector.localization_timestamp = clock[0]
    collector.localization_received_at = collector.chassis_received_at = clock[0]

    def tick(duration):
        clock[0] += duration
        state = collector.vehicle_state
        command = collector.last_sent_control
        # Bounded test double for lifecycle/ordering, not vehicle identification.
        accel = command.throttle * 0.1 - command.brake * 0.1
        state.speed_mps = max(0.0, state.speed_mps + duration * accel)
        state.ins_speed_mps = state.speed_mps
        state.imu_accel_y = accel
        state.timestamp = collector.localization_timestamp = clock[0]
        collector.localization_received_at = collector.chassis_received_at = clock[0]
        if collector.is_collecting and collector.output_file:
            collector._write_log_entry()

    monkeypatch.setattr(collector_module.time, "sleep", tick)
    return collector, clock, published


def test_calibration_lifecycle_completes_and_stops(longitudinal_runtime):
    collector, _, published = longitudinal_runtime
    args = _calibration_plan_args("")
    args.hold_duration_ms = 0
    plan = generate_calibration_plan(args)
    assert collector._execute_case(plan[0])
    assert published[-1].throttle == 0 and published[-1].brake > 0
    assert not collector.is_collecting
    status = json.loads(collector.output_file_path.with_suffix(".status.json").read_text())
    assert status == {"completed": True, "abort_reason": None}


def test_brake_profile_runs_only_after_preparation(longitudinal_runtime):
    collector, _, published = longitudinal_runtime
    case = generate_dynamic_plan(
        output="", actuator="brake", initial_speed_mps=1.0,
        duration_sec=0.4, start_time_sec=0.1, amplitude=5.0)[0]
    assert collector._execute_case(case)
    data = pd.read_csv(collector.output_file_path)
    assert data["speed_mps"].iloc[0] >= 0.9
    assert (data["ctl_throttle"] == 0).all()
    first_brake = next(i for i, cmd in enumerate(published) if cmd.brake > 0)
    assert any(cmd.throttle > 0 for cmd in published[:first_brake])
    assert collector.vehicle_state.speed_mps <= 0.1


def test_stop_timeout_is_bounded_and_not_reported_as_success(longitudinal_runtime, monkeypatch):
    collector, clock, _ = longitudinal_runtime
    collector.active_case = {"stop_timeout_sec": 0.1}
    collector.vehicle_state.speed_mps = 2.0
    monkeypatch.setattr(collector_module.time, "sleep",
                        lambda duration: clock.__setitem__(0, clock[0] + duration))
    assert not collector._stop_vehicle()
    assert clock[0] <= 100.15


def test_timed_out_case_is_not_successful(longitudinal_runtime):
    collector, _, published = longitudinal_runtime
    case = generate_calibration_plan(_calibration_plan_args(""))[0]
    case["steps"][0]["timeout_sec"] = 0.02
    assert not collector._execute_case(case)
    assert "timed out" in collector.abort_reason
    assert published[-1].brake > 0
    status = json.loads(collector.output_file_path.with_suffix(".status.json").read_text())
    assert not status["completed"]


def test_hold_requires_continuous_trigger_and_switches_command_immediately(longitudinal_runtime):
    collector, clock, published = longitudinal_runtime
    case = generate_calibration_plan(_calibration_plan_args(""))[0]
    case["steps"][0]["hold_duration_ms"] = 100
    collector.active_case = case
    collector.is_collecting = True
    collector.step_start_time = clock[0]
    collector.vehicle_state.speed_mps = 1.0
    collector._state_machine_tick()
    clock[0] += 0.05
    collector.vehicle_state.speed_mps = 0.0
    collector._state_machine_tick()
    assert collector.trigger_met_time is None
    clock[0] += 0.02
    collector.vehicle_state.speed_mps = 1.0
    collector._state_machine_tick()
    clock[0] += 0.11
    collector._state_machine_tick()
    assert collector.active_step_idx == 1
    assert published[-1].throttle == 0 and published[-1].brake > 0


def test_stale_feedback_aborts_before_next_command(longitudinal_runtime):
    collector, clock, published = longitudinal_runtime
    collector.active_case = generate_calibration_plan(_calibration_plan_args(""))[0]
    collector.is_collecting = True
    clock[0] += 1.0
    collector._state_machine_tick()
    assert not collector.is_collecting and not collector.case_completed
    assert "stale" in collector.abort_reason
    assert published[-1].throttle == 0 and published[-1].brake > 0


@pytest.mark.parametrize("mode,t,expected", [
    ("step", 1.5, 20.0), ("ramp", 1.5, 15.0),
    ("pulse", 1.5, 20.0), ("pulse", 2.1, 10.0),
    ("triangle", 1.5, 10.0), ("hysteresis", 2.1, 20.0),
    ("single_sine", 1.25, 20.0), ("chirp", 1.25, 20.0),
    ("sweep", 1.25, 20.0), ("multi_sine", 1.25, 20.0),
])
def test_profile_values_and_baseline_before_start(mode, t, expected):
    profile = {
        "type": mode, "baseline": 10.0, "amplitude": 10.0,
        "start_time_sec": 1.0, "end_time_sec": 2.0,
        "frequency_hz": 1.0, "frequency_start_hz": 1.0,
        "frequency_end_hz": 1.0,
    }
    evaluate = collector_module.evaluate_command_profile
    assert evaluate(profile, 0.5) == 10.0
    assert evaluate(profile, t) == pytest.approx(expected)
