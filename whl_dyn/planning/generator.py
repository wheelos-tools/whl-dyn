import argparse
import math
from dataclasses import asdict, dataclass
from typing import List, Optional
import yaml
import numpy as np
from whl_dyn.planning.preflight import validate_longitudinal_plan


@dataclass
class DynamicPlanConfig:
    """Arguments used to create one open-loop dynamic test case.

    The class intentionally uses only Python 3.7 compatible typing so it can
    also be used by the Streamlit UI and by small scripts without argparse.
    """

    output: str = "dynamic_plan.yaml"
    mode: str = "step"
    profile_type: str = ""
    actuator: str = "throttle"
    amplitude: float = 20.0
    baseline: Optional[float] = None
    duration_sec: float = 10.0
    sampling_rate_hz: float = 50.0
    start_time_sec: float = 1.0
    end_time_sec: float = 0.0
    frequency_hz: float = 0.5
    frequency_start_hz: float = 0.1
    frequency_end_hz: float = 2.0
    bit_duration_sec: float = 0.1
    prbs_seed: int = 7
    pulse_duration_sec: float = 1.0
    period_sec: float = 2.0
    frequencies: Optional[List[float]] = None
    amplitudes: Optional[List[float]] = None
    case_name: str = ""
    initial_speed_mps: Optional[float] = None
    preparation_throttle: float = 20.0
    preparation_timeout_sec: float = 30.0
    max_speed_mps: float = 5.0


def generate_dynamic_plan(args=None, output=None, **kwargs):
    """Generate a collector-compatible YAML plan for an actuator test.

    A dynamic case deliberately has no speed trigger.  The collector evaluates
    ``command_profile`` against elapsed time and logs the resulting command on
    every chassis callback.  ``args`` may be :class:`DynamicPlanConfig`, an
    argparse Namespace, or a dictionary; the returned list is also useful to
    callers that do not need a YAML file.
    """
    config = args or DynamicPlanConfig()
    if isinstance(config, DynamicPlanConfig):
        values = asdict(config)
    elif isinstance(config, dict):
        values = dict(config)
    else:
        values = vars(config).copy()
    values.update(kwargs)

    mode = str(values.get("profile_type") or values.get("mode", "step")).lower()
    actuator = str(values.get("actuator", "throttle")).lower()
    duration = float(values.get("duration_sec", values.get("duration", 10.0)))
    sample_rate = float(values.get("sampling_rate_hz", values.get("sampling_rate", 50.0)))
    amplitude = float(values.get("amplitude", values.get("level", 20.0)))
    baseline_value = values.get("baseline", values.get("offset"))
    symmetric = mode in ("single_sine", "chirp", "sweep", "triangle", "multi_sine")
    baseline = float(baseline_value) if baseline_value is not None else (
        abs(amplitude) if symmetric and actuator != "both" else 0.0)
    profile = {
        "type": mode,
        "profile_type": mode,
        "baseline": baseline,
        "amplitude": amplitude,
        "duration_sec": duration,
        "start_time_sec": float(values.get("start_time_sec", 1.0)),
        "end_time_sec": float(values.get("end_time_sec") or max(1.0, duration - 1.0)),
        "frequency_hz": float(values.get("frequency_hz", 0.5)),
        "frequency_start_hz": float(values.get("frequency_start_hz", 0.1)),
        "frequency_end_hz": float(values.get("frequency_end_hz", 2.0)),
        "bit_duration_sec": float(values.get("bit_duration_sec", 0.1)),
        "prbs_seed": int(values.get("prbs_seed", 7)),
        "prbs_low": float(values.get("prbs_low", baseline)),
        "prbs_high": float(values.get("prbs_high", baseline + amplitude)),
        "pulse_duration_sec": float(values.get("pulse_duration_sec", 1.0)),
        "period_sec": float(values.get("period_sec", 2.0)),
    }
    frequencies = values.get("frequencies")
    amplitudes = values.get("amplitudes")
    if frequencies is not None:
        profile["frequencies_hz"] = [float(item) for item in frequencies]
    if amplitudes is not None:
        profile["amplitudes"] = [float(item) for item in amplitudes]

    case_name = values.get("case_name") or "dynamic_{0}_{1}".format(actuator, mode)
    case = {
        "case_name": case_name,
        "description": "Open-loop {0} {1} actuator dynamic test.".format(actuator, mode),
        "dynamic": True,
        "domain": "actuator_characterization"
        if mode in ("step", "ramp", "pulse", "triangle", "hysteresis")
        else "frequency_response",
        "mode": mode,
        "actuator": actuator,
        "command_profile": profile,
        "sampling_rate_hz": sample_rate,
        "duration_sec": duration,
        "gear_location": 1,
        "max_speed_mps": values.get("max_speed_mps", 5.0),
        "input_signals": ["command"],
        "output_signals": ["speed_mps", "imu_accel_y"],
    }
    initial_speed = values.get("initial_speed_mps")
    if initial_speed is not None:
        case["preparation"] = {
            "target_speed_mps": initial_speed,
            "timeout_sec": values.get("preparation_timeout_sec", 30.0),
            "command": {"throttle": values.get("preparation_throttle", 20.0),
                        "brake": 0.0},
        }
    plan = [case]
    validate_longitudinal_plan(plan)
    destination = values.get("output") if output is None else output
    if destination:
        with open(destination, "w") as plan_file:
            yaml.safe_dump(plan, plan_file, sort_keys=False, default_flow_style=False)
    return plan


def _finite_number(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("{0} must be a finite number".format(name))
    if not math.isfinite(number):
        raise ValueError("{0} must be a finite number".format(name))
    return number


def _integer_value(value, name):
    number = _finite_number(value, name)
    if not number.is_integer():
        raise ValueError("{0} must be an integer".format(name))
    return int(number)


def _command_levels(minimum, maximum, count, name):
    minimum = _integer_value(minimum, "{0}_min".format(name))
    maximum = _integer_value(maximum, "{0}_max".format(name))
    count = _integer_value(count, "{0}_num_steps".format(name))
    if minimum < 0 or maximum > 100 or minimum > maximum:
        raise ValueError("{0} range must satisfy 0 <= min <= max <= 100".format(name))
    if count < 1:
        raise ValueError("{0}_num_steps must be at least one".format(name))
    if minimum == maximum:
        levels = [minimum]
    else:
        levels = np.linspace(minimum, maximum, count, dtype=int).tolist()
        if len(set(levels)) != len(levels):
            raise ValueError(
                "{0} range and number of steps produce duplicate integer commands"
                .format(name))
    return [int(level) for level in levels if level > 0]


def _speed_token(speed):
    return format(float(speed), "g")


def generate_calibration_plan(args):
    """
    Generates a comprehensive longitudinal calibration plan in YAML format.
    """
    throttle_levels = _command_levels(
        args.throttle_min, args.throttle_max, args.throttle_num_steps, "throttle")
    brake_levels = _command_levels(
        args.brake_min, args.brake_max, args.brake_num_steps, "brake")
    speed_targets = [
        _finite_number(speed, "speed_targets")
        for speed in args.speed_targets
    ]
    if not speed_targets or any(speed <= 0.0 for speed in speed_targets):
        raise ValueError("speed_targets must contain positive speeds")
    if len(set(speed_targets)) != len(speed_targets):
        raise ValueError("speed_targets must not contain duplicates")

    default_throttle = _finite_number(args.default_throttle, "default_throttle")
    default_brake = _finite_number(args.default_brake, "default_brake")
    if not 0.0 <= default_throttle <= 100.0:
        raise ValueError("default_throttle must be in [0, 100]")
    if not 0.0 <= default_brake <= 100.0:
        raise ValueError("default_brake must be in [0, 100]")
    if brake_levels and default_throttle <= 0.0:
        raise ValueError("default_throttle must be positive for brake tests")
    if throttle_levels and default_brake <= 0.0:
        raise ValueError("default_brake must be positive for throttle tests")

    sampling_rate = _finite_number(
        getattr(args, "sampling_rate_hz", 50.0), "sampling_rate_hz")
    if sampling_rate < 1.0:
        raise ValueError("sampling_rate_hz must be at least one")
    accel_timeout = _finite_number(
        getattr(args, "accel_timeout", 30.0), "accel_timeout")
    decel_timeout = _finite_number(
        getattr(args, "decel_timeout", 30.0), "decel_timeout")
    if accel_timeout <= 0.0 or decel_timeout <= 0.0:
        raise ValueError("accel_timeout and decel_timeout must be positive")
    hold_duration_ms = _integer_value(
        getattr(args, "hold_duration_ms", 0), "hold_duration_ms")
    if hold_duration_ms < 0:
        raise ValueError("hold_duration_ms must be non-negative")
    if not throttle_levels and not brake_levels:
        raise ValueError("the plan must contain at least one non-zero command")

    plan = []

    # 1. Generate Throttle Sweep Cases
    for throttle in throttle_levels:
        for speed_target in speed_targets:
            speed_name = _speed_token(speed_target)
            case = {
                'case_name': f"throttle_{throttle}_to_{speed_name}mps",
                'description': f"Accelerate with {throttle}% throttle, target >{speed_target}m/s, then brake to stop.",
                'domain': 'longitudinal_calibration',
                'actuator': 'throttle',
                'gear_location': 1,
                'sampling_rate_hz': sampling_rate,
                'input_signals': [
                    'ctl_throttle', 'ctl_brake', 'ctl_gear'
                ],
                'output_signals': [
                    'speed_mps', 'ins_speed_mps', 'imu_accel_y',
                    'actual_gear', 'throttle_pct', 'brake_pct'
                ],
                'steps': [
                    {
                        'command': {'throttle': float(throttle), 'brake': 0.0},
                        'trigger': {'type': 'speed_greater_than', 'value': speed_target},
                        'timeout_sec': accel_timeout,
                        'hold_duration_ms': hold_duration_ms
                    },
                    {
                        'command': {'throttle': 0.0, 'brake': default_brake},
                        'trigger': {'type': 'speed_less_than', 'value': 0.1},
                        'timeout_sec': decel_timeout
                    }
                ]
            }
            plan.append(case)

    # 2. Generate Brake Sweep Cases (from a coast-down)
    for brake in brake_levels:
        initial_speed_target = max(speed_targets)
        speed_name = _speed_token(initial_speed_target)

        case = {
            'case_name': f"brake_{brake}_from_{speed_name}mps",
            'description': f"Accelerate to >{initial_speed_target}m/s, then apply {brake}% brake.",
            'domain': 'longitudinal_calibration',
            'actuator': 'brake',
            'gear_location': 1,
            'sampling_rate_hz': sampling_rate,
            'input_signals': [
                'ctl_throttle', 'ctl_brake', 'ctl_gear'
            ],
            'output_signals': [
                'speed_mps', 'ins_speed_mps', 'imu_accel_y',
                'actual_gear', 'throttle_pct', 'brake_pct'
            ],
            'steps': [
                {
                    'command': {'throttle': default_throttle, 'brake': 0.0},
                    'trigger': {'type': 'speed_greater_than', 'value': float(initial_speed_target)},
                    'timeout_sec': accel_timeout,
                    'hold_duration_ms': hold_duration_ms
                },
                {
                    'command': {'throttle': 0.0, 'brake': float(brake)},
                    'trigger': {'type': 'speed_less_than', 'value': 0.1},
                    'timeout_sec': decel_timeout
                }
            ]
        }
        plan.append(case)

    destination = getattr(args, "output", None)
    validate_longitudinal_plan(plan)
    if destination:
        with open(destination, 'w') as plan_file:
            yaml.safe_dump(
                plan, plan_file, sort_keys=False, default_flow_style=False,
                indent=2)
        print(f"OK: Successfully generated calibration plan with {len(plan)} cases.")
        print(f"File saved to: {destination}")
    return plan

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a YAML plan for vehicle longitudinal calibration.")

    parser.add_argument('-o', '--output', type=str, default='calibration_plan.yaml', help="Output YAML file name.")
    parser.add_argument('--dynamic', action='store_true',
                        help="Generate one open-loop dynamic command-profile case.")
    parser.add_argument('--mode', choices=['step', 'ramp', 'pulse', 'triangle',
                                           'hysteresis', 'single_sine', 'chirp',
                                           'sweep', 'multi_sine', 'prbs'], default='step')
    parser.add_argument('--actuator', choices=['throttle', 'brake'], default='throttle')
    parser.add_argument('--amplitude', type=float, default=20.0)
    parser.add_argument('--baseline', type=float, default=None)
    parser.add_argument('--duration-sec', type=float, default=10.0)
    parser.add_argument('--sampling-rate-hz', type=float, default=50.0)
    parser.add_argument('--frequency-start-hz', type=float, default=0.1)
    parser.add_argument('--frequency-end-hz', type=float, default=2.0)
    parser.add_argument('--frequency-hz', type=float, default=0.5)
    parser.add_argument('--bit-duration-sec', type=float, default=0.1)
    parser.add_argument('--prbs-seed', type=int, default=7)
    parser.add_argument('--initial-speed-mps', type=float,
                        help="Required preparation speed for dynamic brake tests.")
    parser.add_argument('--preparation-throttle', type=float, default=20.0)
    parser.add_argument('--preparation-timeout-sec', type=float, default=30.0)
    parser.add_argument('--max-speed-mps', type=float, default=5.0)

    # Throttle parameters
    parser.add_argument('--throttle-min', type=int, default=0, help="Minimum throttle command (percent) to test.")
    parser.add_argument('--throttle-max', type=int, default=80, help="Maximum throttle command (percent) to test.")
    parser.add_argument('--throttle-num-steps', type=int, default=5, help="Number of throttle steps to generate.")

    # Brake parameters
    parser.add_argument('--brake-min', type=int, default=0, help="Minimum brake command (percent) to test.")
    parser.add_argument('--brake-max', type=int, default=50, help="Maximum brake command (percent) to test.")
    parser.add_argument('--brake-num-steps', type=int, default=5, help="Number of brake steps to generate.")

    # Test dynamics parameters
    parser.add_argument('--speed-targets', nargs='+', type=float, default=[1.0, 3.0, 5.0], help="List of target speeds (m/s) for acceleration tests.")
    parser.add_argument('--default-throttle', type=float, default=80.0, help="Default throttle command (percent) used to accelerate the vehicle before brake test steps.")
    parser.add_argument('--default-brake', type=float, default=30.0, help="Default brake command (percent) used to stop the vehicle after a test step.")
    parser.add_argument('--hold-duration-ms', type=int, default=0, help="Hold duration (ms) after trigger condition is met before switching to next step.")

    # Safety and timeout
    parser.add_argument('--accel-timeout', type=float, default=30.0, help="Timeout in seconds for acceleration steps.")
    parser.add_argument('--decel-timeout', type=float, default=30.0, help="Timeout in seconds for deceleration steps.")

    args = parser.parse_args()
    if args.dynamic:
        generate_dynamic_plan(args)
    else:
        generate_calibration_plan(args)
