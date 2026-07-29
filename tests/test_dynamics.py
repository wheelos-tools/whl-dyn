import numpy as np
import pandas as pd
import yaml

from whl_dyn.planning.generator import DynamicPlanConfig, generate_dynamic_plan
from whl_dyn.processing.dynamics import (
    analyze_dynamic,
    analyze_frequency_response,
    analyze_step,
    is_dynamic_log,
)


def test_dynamic_plan_contains_profile_and_can_be_loaded(tmp_path):
    path = tmp_path / "dynamic.yaml"
    plan = generate_dynamic_plan(DynamicPlanConfig(
        output=str(path), mode="sweep", actuator="brake", duration_sec=4.0
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
