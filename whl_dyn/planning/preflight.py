"""Static safety and schema validation for active open-loop test plans."""

import math


def finite_number(value, name, minimum=None, positive=False):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("{0} must be a finite number".format(name))
    if not math.isfinite(number):
        raise ValueError("{0} must be finite".format(name))
    if (minimum is not None and number < minimum) or (positive and number <= 0):
        raise ValueError("{0} is outside its allowed range".format(name))
    return number


def validate_longitudinal_command(command, default_gear=1):
    if not isinstance(command, dict):
        raise ValueError("longitudinal command must be a mapping")
    throttle = finite_number(command.get("throttle", 0), "throttle", minimum=0)
    brake = finite_number(command.get("brake", 0), "brake", minimum=0)
    if throttle > 100 or brake > 100 or (throttle > 0 and brake > 0):
        raise ValueError("throttle/brake must be mutually exclusive and within [0, 100]")
    if finite_number(command.get("gear_location", default_gear), "gear") != 1:
        raise ValueError("longitudinal tests currently require forward Drive gear")
    return True


def validate_longitudinal_plan(cases):
    """Validate generated and hand-edited plans before active publication."""
    if not isinstance(cases, list) or not cases:
        raise ValueError("longitudinal plan must be a non-empty case list")
    names = set()
    for case in cases:
        if not isinstance(case, dict):
            raise ValueError("case must be a mapping")
        name = case.get("case_name")
        if (not isinstance(name, str) or not name or name in names or
                "/" in name or "\\" in name or name in (".", "..")):
            raise ValueError("case_name must be a unique filename component")
        names.add(name)
        gear = case.get("gear_location", 1)
        validate_longitudinal_command({}, gear)
        rate = finite_number(case.get("sampling_rate_hz", 100), "sampling_rate_hz",
                             minimum=1)
        finite_number(case.get("max_source_age_sec", 0.5), "max_source_age_sec",
                      positive=True)
        finite_number(case.get("stop_timeout_sec", 10), "stop_timeout_sec",
                      positive=True)
        maximum_speed = case.get("max_speed_mps")
        if maximum_speed is not None:
            maximum_speed = finite_number(maximum_speed, "max_speed_mps", positive=True)
        dynamic = (case.get("dynamic") or "command_profile" in case or
                   "profile" in case or case.get("domain") in
                   ("actuator_characterization", "frequency_response"))
        if dynamic:
            profile = case.get("command_profile", case.get("profile"))
            if not isinstance(profile, dict):
                raise ValueError("dynamic case requires a command profile")
            duration = finite_number(case.get("duration_sec", profile.get("duration_sec")),
                                     "duration_sec", positive=True)
            mode = profile.get("type", profile.get("profile_type", "step"))
            modes = ("step", "ramp", "pulse", "triangle", "hysteresis",
                     "single_sine", "chirp", "sweep", "multi_sine", "prbs")
            if mode not in modes or case.get("mode", mode) != mode:
                raise ValueError("unsupported or inconsistent dynamic mode")
            actuator = case.get("actuator", "throttle")
            if actuator not in ("throttle", "brake", "both"):
                raise ValueError("unsupported longitudinal actuator")
            # Validate all provided numeric fields, including aliases accepted
            # by the evaluator. Do not let NaN hide behind min/max clipping.
            for key, value in profile.items():
                if key in ("type", "profile_type", "mode", "method"):
                    continue
                for item in value if isinstance(value, list) else [value]:
                    finite_number(item, key)
            baseline = float(profile.get("baseline", profile.get("offset", profile.get("start_value", 0))))
            amplitude = float(profile.get("amplitude", profile.get("level", profile.get("height", profile.get("peak", 0)))))
            start = float(profile.get("start_time_sec", profile.get("start_sec", 0)))
            if not 0 <= start < duration:
                raise ValueError("profile start must lie inside the case duration")
            if "duration_sec" in profile and float(profile["duration_sec"]) != duration:
                raise ValueError("profile and case durations must agree")
            low, high = sorted((baseline, baseline + amplitude))
            if mode == "ramp":
                end = float(profile.get("ramp_end_sec", profile.get("end_time_sec", 1)))
                ramp_start = float(profile.get("ramp_start_sec", start))
                if not 0 <= ramp_start < end <= duration:
                    raise ValueError("ramp requires 0 <= start < end <= duration")
                target = float(profile.get("end_value", profile.get("target", profile.get("final_value", baseline + amplitude))))
                low, high = sorted((baseline, target))
            elif mode == "pulse":
                width = finite_number(profile.get("pulse_duration_sec", profile.get("width_sec", 1)),
                                      "pulse_duration_sec", positive=True)
                if float(profile.get("pulse_start_sec", start)) + width > duration:
                    raise ValueError("pulse must fit inside case duration")
            elif mode in ("triangle", "hysteresis"):
                finite_number(profile.get("period_sec", profile.get("period", 2)),
                              "period_sec", positive=True)
                default_low = baseline - abs(amplitude) if mode == "triangle" else baseline
                low = float(profile.get("min_value", profile.get("low", default_low)))
                default_high = baseline + abs(amplitude) if mode == "triangle" else baseline + amplitude
                high = float(profile.get("max_value", profile.get("high", default_high)))
            elif mode == "prbs":
                bit = finite_number(profile.get("bit_duration_sec", profile.get("switch_interval_sec", 0.1)),
                                    "bit_duration_sec", positive=True)
                if bit < 1.0 / rate:
                    raise ValueError("PRBS bit duration is below the command sampling interval")
                low = float(profile.get("prbs_low", profile.get("low", profile.get("min_value", baseline))))
                high = float(profile.get("prbs_high", profile.get("high", profile.get("max_value", baseline + amplitude))))
            elif mode in ("single_sine", "chirp", "sweep", "multi_sine"):
                if mode == "single_sine":
                    frequencies = [profile.get("frequency_hz", profile.get("frequency", 1))]
                elif mode == "multi_sine":
                    frequencies = profile.get("frequencies_hz", profile.get("frequencies", [1]))
                    amplitudes = profile.get("amplitudes", [amplitude] * len(frequencies))
                    if not frequencies or len(amplitudes) != len(frequencies):
                        raise ValueError("multi-sine frequencies and amplitudes must match")
                    amplitude = sum(abs(float(item)) for item in amplitudes)
                else:
                    frequencies = [profile.get("frequency_start_hz", profile.get("f0_hz", profile.get("start_frequency_hz", 0.1))),
                                   profile.get("frequency_end_hz", profile.get("f1_hz", profile.get("end_frequency_hz", 2)))]
                if any(not 0 < float(frequency) < rate / 2 for frequency in frequencies):
                    raise ValueError("excitation frequencies must lie below Nyquist and above zero")
                low, high = baseline - abs(amplitude), baseline + abs(amplitude)
            lower_limit = -100 if actuator == "both" else 0
            if min(low, high, baseline) < lower_limit or max(low, high, baseline) > 100:
                raise ValueError("profile exceeds actuator range; choose a non-clipping baseline/amplitude")
            preparation = case.get("preparation")
            if actuator in ("brake", "both") and preparation is None:
                raise ValueError("brake/both dynamic tests require initial-speed preparation")
            if preparation is not None:
                if not isinstance(preparation, dict):
                    raise ValueError("preparation must be a mapping")
                target = finite_number(preparation.get("target_speed_mps"),
                                       "preparation speed", positive=True)
                finite_number(preparation.get("timeout_sec"), "preparation timeout", positive=True)
                validate_longitudinal_command(preparation.get("command"), gear)
                if float(preparation["command"].get("throttle", 0)) <= 0:
                    raise ValueError("preparation requires positive throttle")
                if maximum_speed is not None and target >= maximum_speed:
                    raise ValueError("preparation speed must be below the hard maximum")
        else:
            steps = case.get("steps")
            if not isinstance(steps, list) or not steps:
                raise ValueError("calibration case requires non-empty steps")
            for step in steps:
                if not isinstance(step, dict):
                    raise ValueError("step must be a mapping")
                validate_longitudinal_command(step.get("command"), gear)
                finite_number(step.get("timeout_sec"), "step timeout", positive=True)
                finite_number(step.get("hold_duration_ms", 0), "hold_duration_ms", minimum=0)
                trigger = step.get("trigger", {})
                if not isinstance(trigger, dict) or trigger.get("type") not in (
                        "speed_greater_than", "speed_less_than"):
                    raise ValueError("unsupported speed trigger")
                finite_number(trigger.get("value"), "trigger speed", minimum=0)
    return True


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
    names = set()
    for case in cases:
        if not isinstance(case, dict):
            raise ValueError("open-loop case must be a mapping")
        name = case.get("case_name")
        if not isinstance(name, str) or not name or name in names:
            raise ValueError("open-loop case names must be non-empty and unique")
        names.add(name)
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
        finite_number(case.get("duration_sec"), "duration_sec", positive=True)
        rate = finite_number(case.get("sampling_rate_hz"), "sampling_rate_hz", positive=True)
        profile = case["command_profile"]
        for key, value in profile.items():
            if key not in ("type", "profile_type", "method", "mode"):
                finite_number(value, key)
        for key in ("frequency_hz", "frequency_start_hz", "frequency_end_hz"):
            if key in profile and not 0 < float(profile[key]) < rate / 2:
                raise ValueError("steering excitation frequency must be below Nyquist")
        if "bit_duration_sec" in profile and float(profile["bit_duration_sec"]) <= 0:
            raise ValueError("bit duration must be positive")
        safety = case.get("safety_limits", {})
        maximum = finite_number(safety.get("max_abs_steering"),
                                "max_abs_steering", positive=True)
        finite_number(safety.get("max_steering_rate"), "max_steering_rate", positive=True)
        for key, value in safety.items():
            finite_number(value, key, positive=True)
        baseline = float(profile.get("baseline", 0))
        amplitude = float(profile.get("amplitude", 0))
        target = float(profile.get("target", baseline + amplitude))
        bound = max(abs(baseline), abs(target))
        if profile.get("type") in ("single_sine", "chirp", "sweep"):
            bound = abs(baseline) + abs(amplitude)
        if profile.get("type") == "prbs":
            bound = max(abs(float(profile.get("prbs_low", baseline - amplitude))),
                        abs(float(profile.get("prbs_high", baseline + amplitude))))
        if bound > maximum:
            raise ValueError("steering profile exceeds configured magnitude limit")
        gate = case.get("speed_gate", {})
        required = ("min_mps", "max_mps", "target_mps", "tolerance_mps")
        if any(field not in gate for field in required):
            raise ValueError("open-loop case requires a complete speed gate")
        for field in required:
            finite_number(gate[field], field, minimum=0)
        if float(gate["max_mps"]) <= float(gate["min_mps"]):
            raise ValueError("speed gate must have positive width")
        if not (float(gate["min_mps"]) <= float(gate["target_mps"]) <=
                float(gate["max_mps"])):
            raise ValueError("speed target must lie inside hard speed range")
        for field in ("stable_duration_sec", "max_wait_sec"):
            if field in gate:
                finite_number(gate[field], field, positive=True)
    return True


def validate_active_signal_config(config):
    """Require a normalized actual steering feedback signal for active tests."""

    fields = config.get("detail_fields", {}) if isinstance(config, dict) else {}
    chassis_fields = (
        config.get("chassis_fields", {}) if isinstance(config, dict) else {})
    if ("steering_feedback" not in fields and
            "steering_feedback" not in chassis_fields):
        raise ValueError(
            "active open-loop collection requires "
            "detail_fields.steering_feedback or chassis_fields.steering_feedback")
    if config.get("detail_message") is not None and not isinstance(
            config.get("detail_message"), dict):
        raise ValueError("detail_message fallback must be a mapping")
    scale = finite_number(config.get("steering_feedback_scale", 1.0),
                          "steering_feedback_scale")
    if scale == 0.0:
        raise ValueError("steering_feedback_scale must be non-zero")
    return True
