"""Static safety and schema validation for active open-loop test plans."""


OPEN_LOOP_TYPES = {
    "steering_step",
    "steering_deadzone_rate",
    "lateral_frequency_response",
    "fixed_steering_steady_state",
}


def validate_open_loop_plan(cases):
    """Reject plans that could accidentally invoke closed-loop publication."""

    if not isinstance(cases, list) or not cases:
        raise ValueError("open-loop plan must be a non-empty case list")
    for case in cases:
        if case.get("test_type") not in OPEN_LOOP_TYPES:
            raise ValueError("unsupported open-loop test type")
        if case.get("domain") != "vehicle_dynamics":
            raise ValueError("open-loop case must use vehicle_dynamics domain")
        if case.get("actuator") != "steering":
            raise ValueError("open-loop case must command steering")
        if "trajectory" in case:
            raise ValueError("open-loop case must not contain a trajectory")
        if not isinstance(case.get("command_profile"), dict):
            raise ValueError("open-loop case requires a command profile")
        gate = case.get("speed_gate", {})
        required = ("min_mps", "max_mps", "target_mps", "tolerance_mps")
        if any(field not in gate for field in required):
            raise ValueError("open-loop case requires a complete speed gate")
        if not (float(gate["min_mps"]) <= float(gate["target_mps"]) <=
                float(gate["max_mps"])):
            raise ValueError("speed target must lie inside hard speed range")
    return True


def validate_active_signal_config(config):
    """Require a normalized actual steering feedback signal for active tests."""

    fields = config.get("detail_fields", {}) if isinstance(config, dict) else {}
    if "steering_feedback" not in fields:
        raise ValueError(
            "active open-loop collection requires detail_fields.steering_feedback")
    if not config.get("detail_message"):
        raise ValueError("active open-loop collection requires detail_message")
    scale = float(config.get("steering_feedback_scale", 1.0))
    if scale == 0.0:
        raise ValueError("steering_feedback_scale must be non-zero")
    return True
