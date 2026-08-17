import numpy as np
import pandas as pd

from whl_dyn.collection.closed_loop import build_path_from_case
from whl_dyn.planning.handling import (
    generate_closed_loop_curve_plan,
    generate_open_loop_identification_plan,
    generate_steady_state_circle_plan,
)
from whl_dyn.planning.preflight import (
    validate_active_signal_config,
    validate_open_loop_plan,
)
from whl_dyn.processing.handling import (
    select_steady_state_samples,
    steady_state_handling_metrics,
    tracking_metrics,
)
from whl_dyn.trajectory.continuous import (
    CirclePath,
    ClothoidCirclePath,
    build_trajectory_window,
)


def test_circle_windows_share_identical_global_geometry():
    path = CirclePath(10.0, -2.0, 0.3, 0.02)
    first = build_trajectory_window(path, 0.0, 2.0, 1.0, 0.1)
    second = build_trajectory_window(path, 0.1, 2.0, 1.0, 0.1)
    for original, shifted in zip(first[1:], second[:-1]):
        assert np.isclose(original[2].x, shifted[2].x)
        assert np.isclose(original[2].y, shifted[2].y)
        assert np.isclose(original[2].theta, shifted[2].theta)
        assert np.isclose(original[2].kappa, shifted[2].kappa)


def test_clothoid_path_is_continuous_and_requires_long_enough_case():
    path = ClothoidCirclePath(0.0, 0.0, 0.0, 50.0, 15.0, 1.57, 15.0)
    before = path.sample(14.99)
    after = path.sample(15.01)
    assert abs(before.x - after.x) < 0.03
    case = generate_closed_loop_curve_plan(output="")[0]
    assert build_path_from_case(case["trajectory"], 0.0, 0.0, 0.0,
                                case["duration_sec"]) is not None


def test_open_loop_plan_contains_step_and_slow_ramp_both_directions():
    cases = generate_open_loop_identification_plan(output="")
    assert len(cases) == 4
    assert {case["test_type"] for case in cases} == {
        "steering_step", "steering_deadzone_rate"}
    assert all(case["speed_gate"]["target_mps"] == 2.0 for case in cases)
    assert all(case["allow_command_step"] for case in cases
               if case["test_type"] == "steering_step")


def test_steady_turn_matrix_is_open_loop_and_has_both_directions():
    cases = generate_steady_state_circle_plan(
        steering_commands=(1.0, 2.0), speed_targets_mps=(1.0, 2.0),
        repeats=1, steering_ramp_rate=0.5, output="")
    assert cases
    assert all("trajectory" not in case for case in cases)
    assert all(case["command_profile"]["type"] == "ramp" for case in cases)
    assert {case["command_profile"]["target"] > 0.0 for case in cases} == {
        True, False}
    assert validate_open_loop_plan(cases)


def test_open_loop_preflight_rejects_trajectory_and_missing_feedback_config():
    case = generate_steady_state_circle_plan(repeats=1, output="")[0]
    invalid = dict(case, trajectory={"type": "circle"})
    try:
        validate_open_loop_plan([invalid])
    except ValueError as error:
        assert "trajectory" in str(error)
    else:
        raise AssertionError("open-loop trajectory was accepted")
    try:
        validate_active_signal_config({"detail_message": {"module": "m", "class": "C"}})
    except ValueError as error:
        assert "steering_feedback" in str(error)
    else:
        raise AssertionError("active collection accepted missing feedback")
    assert validate_active_signal_config({
        "detail_message": {"module": "m", "class": "C"},
        "detail_fields": {"steering_feedback": "feedback.steering_angle"},
    })


def test_steady_state_selector_discards_ramp_settle_and_speed_outliers():
    frame = pd.DataFrame({
        "elapsed_sec": [0.0, 2.0, 4.0, 5.0, 6.0, 7.0],
        "case_phase": ["ramp", "steady", "steady", "steady", "steady", "steady"],
        "chassis_speed_mps": [2.0, 2.0, 2.0, 2.3, 2.0, 2.0],
    })
    selected = select_steady_state_samples(
        frame, speed_target_mps=2.0, speed_tolerance_mps=0.15,
        settle_after_ramp_sec=2.0)
    assert selected["elapsed_sec"].tolist() == [4.0, 6.0, 7.0]


def test_handling_metrics_fit_known_understeer_gradient_and_tracking():
    lateral_accel = np.array([0.5, 1.0, 1.5, 2.0])
    kappa = np.full(4, 0.02)
    steering = 2.0 * lateral_accel + 2.5 * kappa + 0.1
    frame = pd.DataFrame({
        "steering_rad": steering,
        "reference_kappa_1pm": kappa,
        "lateral_accel_mps2": lateral_accel,
        "lateral_error_m": [0.01, -0.02, 0.03, -0.04],
        "heading_error_rad": [0.01, -0.01, 0.02, -0.02],
    })
    handling = steady_state_handling_metrics(
        frame, "steering_rad", wheelbase_m=2.5)
    tracking = tracking_metrics(frame)
    assert np.isclose(handling["understeer_gradient_rad_per_mps2"], 2.0)
    assert np.isclose(handling["steering_offset_rad"], 0.1)
    assert np.isclose(tracking["ey_peak_m"], 0.04)
