"""Plans for open-loop lateral vehicle-dynamics experiments."""

from dataclasses import asdict, dataclass
from typing import Optional

import yaml


SUPPORTED_LATERAL_PROFILES = ("pulse", "single_sine", "chirp", "sweep", "prbs")


@dataclass
class LateralFrequencyPlanConfig:
    """Configuration for one steering-to-vehicle frequency-response case."""

    output: str = "lateral_frequency_plan.yaml"
    mode: str = "chirp"
    case_name: str = ""
    duration_sec: float = 120.0
    sampling_rate_hz: float = 100.0
    baseline_steering: float = 0.0
    steering_amplitude: float = 2.0
    frequency_start_hz: float = 0.05
    frequency_end_hz: float = 2.0
    sweep_method: str = "logarithmic"
    prbs_low: Optional[float] = None
    prbs_high: Optional[float] = None
    bit_duration_sec: float = 0.25
    prbs_seed: int = 7
    pulse_duration_sec: float = 1.0
    sine_frequency_hz: float = 0.5
    speed_min_mps: float = 0.0
    speed_max_mps: float = 3.0
    target_speed_mps: float = 2.0
    speed_tolerance_mps: float = 0.15
    stable_speed_sec: float = 3.0
    max_speed_wait_sec: float = 30.0
    max_steering: float = 20.0
    max_steering_rate: float = 30.0


def _as_values(args, kwargs):
    if isinstance(args, LateralFrequencyPlanConfig):
        values = asdict(args)
    elif isinstance(args, dict):
        values = dict(args)
    elif args is None:
        values = asdict(LateralFrequencyPlanConfig())
    else:
        values = vars(args).copy()
    values.update(kwargs)
    return values


def generate_lateral_frequency_plan(args=None, output=None, **kwargs):
    """Create a YAML-compatible, vehicle-agnostic lateral test case.

    The plan specifies semantic signals only.  Mapping those signals to a
    particular chassis-detail protobuf is a collection-time concern.
    """

    values = _as_values(args, kwargs)
    mode = str(values.get("mode", "chirp")).lower().replace("-", "_")
    if mode not in SUPPORTED_LATERAL_PROFILES:
        raise ValueError("lateral frequency mode must be one of: {0}".format(
            ", ".join(SUPPORTED_LATERAL_PROFILES)))

    duration = float(values.get("duration_sec", 120.0))
    f0 = float(values.get("frequency_start_hz", 0.05))
    f1 = float(values.get("frequency_end_hz", 2.0))
    if duration <= 0.0:
        raise ValueError("duration must be positive and 0 < start frequency < end frequency")
    if mode not in ("pulse", "single_sine") and (f0 <= 0.0 or f1 <= f0):
        raise ValueError("duration must be positive and 0 < start frequency < end frequency")

    baseline = float(values.get("baseline_steering", 0.0))
    amplitude = float(values.get("steering_amplitude", 2.0))
    case_name = values.get("case_name") or "lateral_{0}_{1}".format(
        mode, values.get("speed_max_mps", 3.0))
    profile = {
        "type": mode,
        "baseline": baseline,
        "amplitude": amplitude,
        "start_time_sec": 0.0,
        "end_time_sec": duration,
        "duration_sec": duration,
        "frequency_start_hz": f0,
        "frequency_end_hz": f1,
        "method": str(values.get("sweep_method", "logarithmic")),
        "bit_duration_sec": float(values.get("bit_duration_sec", 0.25)),
        "prbs_seed": int(values.get("prbs_seed", 7)),
        "pulse_duration_sec": float(values.get("pulse_duration_sec", 1.0)),
        "frequency_hz": float(values.get("sine_frequency_hz", 0.5)),
        "prbs_low": float(values.get("prbs_low", baseline - amplitude)
                          if values.get("prbs_low") is not None else baseline - amplitude),
        "prbs_high": float(values.get("prbs_high", baseline + amplitude)
                           if values.get("prbs_high") is not None else baseline + amplitude),
    }
    maximum_rate = float(values.get("max_steering_rate", 30.0))
    if maximum_rate <= 0.0:
        raise ValueError("max steering rate must be positive")
    if mode == "pulse":
        required_rate = float("inf")
    elif mode == "single_sine":
        required_rate = 2.0 * 3.141592653589793 * abs(amplitude) * profile["frequency_hz"]
    elif mode == "prbs":
        required_rate = 2.0 * abs(amplitude) / profile["bit_duration_sec"]
    else:
        required_rate = 2.0 * 3.141592653589793 * abs(amplitude) * f1
    if mode != "pulse" and required_rate > maximum_rate:
        raise ValueError(
            "profile requires {0:.3f} steering units/s, above max_steering_rate "
            "{1:.3f}".format(required_rate, maximum_rate))

    speed_min = float(values.get("speed_min_mps", 0.0))
    speed_max = float(values.get("speed_max_mps", 3.0))
    if speed_min < 0.0 or speed_max <= speed_min:
        raise ValueError("speed gate requires 0 <= min_mps < max_mps")
    target_speed = float(values.get("target_speed_mps", 2.0))
    speed_tolerance = float(values.get("speed_tolerance_mps", 0.15))
    if not speed_min <= target_speed <= speed_max or speed_tolerance < 0.0:
        raise ValueError("target speed must lie within the speed gate")
    case = {
        "case_name": case_name,
        "description": "Steering {0} for lateral vehicle frequency response.".format(mode),
        "domain": "vehicle_dynamics",
        "test_type": "lateral_frequency_response",
        "mode": mode,
        "actuator": "steering",
        "duration_sec": duration,
        "sampling_rate_hz": float(values.get("sampling_rate_hz", 100.0)),
        "input_signals": ["steering_command", "steering_feedback"],
        "output_signals": ["yaw_rate_radps", "lateral_accel_mps2"],
        "speed_gate": {
            "min_mps": speed_min,
            "max_mps": speed_max,
            "target_mps": target_speed,
            "tolerance_mps": speed_tolerance,
            "stable_duration_sec": float(values.get("stable_speed_sec", 3.0)),
            "max_wait_sec": float(values.get("max_speed_wait_sec", 30.0)),
        },
        "safety_limits": {
            "max_abs_steering": float(values.get("max_steering", 20.0)),
            "max_steering_rate": maximum_rate,
        },
        "allow_command_step": mode == "pulse",
        "abort_policy": {
            "speed_range_enforced": True,
            "reserved_checks": ["message_freshness", "driving_mode", "fault_state"],
        },
        "command_profile": profile,
    }
    plan = [case]
    destination = values.get("output") if output is None else output
    if destination:
        with open(destination, "w") as plan_file:
            yaml.safe_dump(plan, plan_file, sort_keys=False, default_flow_style=False)
    return plan
