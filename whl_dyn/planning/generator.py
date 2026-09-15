import argparse
from dataclasses import asdict, dataclass
from typing import List, Optional
import yaml
import numpy as np


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
    baseline: float = 0.0
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
    baseline = float(values.get("baseline", values.get("offset", 0.0)))
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
        "input_signals": ["command"],
        "output_signals": ["speed_mps", "imu_accel_y"],
    }
    plan = [case]
    destination = values.get("output") if output is None else output
    if destination:
        with open(destination, "w") as plan_file:
            yaml.safe_dump(plan, plan_file, sort_keys=False, default_flow_style=False)
    return plan


def generate_calibration_plan(args):
    """
    Generates a comprehensive longitudinal calibration plan in YAML format.
    """
    plan = []

    # 1. Generate Throttle Sweep Cases
    throttle_steps = np.linspace(args.throttle_min, args.throttle_max, args.throttle_num_steps, dtype=int)

    for throttle in throttle_steps:
        if throttle == 0: continue # Skip zero throttle as it's a coasting case
        for speed_target in args.speed_targets:
            case = {
                'case_name': f"throttle_{throttle}_to_{int(speed_target)}mps",
                'description': f"Accelerate with {throttle}% throttle, target >{speed_target}m/s, then brake to stop.",
                'steps': [
                    {
                        'command': {'throttle': float(throttle), 'brake': 0.0},
                        'trigger': {'type': 'speed_greater_than', 'value': float(speed_target)},
                        'timeout_sec': args.accel_timeout,
                        'hold_duration_ms': args.hold_duration_ms
                    },
                    {
                        'command': {'throttle': 0.0, 'brake': args.default_brake},
                        'trigger': {'type': 'speed_less_than', 'value': 0.1},
                        'timeout_sec': args.decel_timeout
                    }
                ]
            }
            plan.append(case)

    # 2. Generate Brake Sweep Cases (from a coast-down)
    brake_steps = np.linspace(args.brake_min, args.brake_max, args.brake_num_steps, dtype=int)

    for brake in brake_steps:
        if brake == 0: continue # Skip zero brake as it's a coasting case

        # For brake tests, we usually need to reach a certain speed first.
        # This plan assumes the operator will manually accelerate, then trigger the test.
        # Or, we can make it a two-step process: auto-accelerate then brake.
        initial_speed_target = max(args.speed_targets) # Use the highest speed for brake tests

        case = {
            'case_name': f"brake_{brake}_from_{int(initial_speed_target)}mps",
            'description': f"Accelerate to >{initial_speed_target}m/s, then apply {brake}% brake.",
            'steps': [
                {
                    'command': {'throttle': args.default_throttle, 'brake': 0.0}, # Use configurable throttle to get to speed
                    'trigger': {'type': 'speed_greater_than', 'value': float(initial_speed_target)},
                    'timeout_sec': args.accel_timeout,
                    'hold_duration_ms': args.hold_duration_ms
                },
                {
                    'command': {'throttle': 0.0, 'brake': float(brake)},
                    'trigger': {'type': 'speed_less_than', 'value': 0.1},
                    'timeout_sec': args.decel_timeout
                }
            ]
        }
        plan.append(case)

    # 3. Save the plan to a YAML file
    with open(args.output, 'w') as f:
        yaml.dump(plan, f, sort_keys=False, default_flow_style=False, indent=2)

    print(f"OK: Successfully generated calibration plan with {len(plan)} cases.")
    print(f"File saved to: {args.output}")

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
    parser.add_argument('--baseline', type=float, default=0.0)
    parser.add_argument('--duration-sec', type=float, default=10.0)
    parser.add_argument('--sampling-rate-hz', type=float, default=50.0)
    parser.add_argument('--frequency-start-hz', type=float, default=0.1)
    parser.add_argument('--frequency-end-hz', type=float, default=2.0)
    parser.add_argument('--frequency-hz', type=float, default=0.5)
    parser.add_argument('--bit-duration-sec', type=float, default=0.1)
    parser.add_argument('--prbs-seed', type=int, default=7)

    # Throttle parameters
    parser.add_argument('--throttle-min', type=int, default=0, help="Minimum throttle command (%) to test.")
    parser.add_argument('--throttle-max', type=int, default=80, help="Maximum throttle command (%) to test.")
    parser.add_argument('--throttle-num-steps', type=int, default=5, help="Number of throttle steps to generate.")

    # Brake parameters
    parser.add_argument('--brake-min', type=int, default=0, help="Minimum brake command (%) to test.")
    parser.add_argument('--brake-max', type=int, default=50, help="Maximum brake command (%) to test.")
    parser.add_argument('--brake-num-steps', type=int, default=5, help="Number of brake steps to generate.")

    # Test dynamics parameters
    parser.add_argument('--speed-targets', nargs='+', type=float, default=[1.0, 3.0, 5.0], help="List of target speeds (m/s) for acceleration tests.")
    parser.add_argument('--default-throttle', type=float, default=80.0, help="Default throttle command (%) used to accelerate the vehicle before brake test steps.")
    parser.add_argument('--default-brake', type=float, default=30.0, help="Default brake command (%) used to stop the vehicle after a test step.")
    parser.add_argument('--hold-duration-ms', type=int, default=0, help="Hold duration (ms) after trigger condition is met before switching to next step.")

    # Safety and timeout
    parser.add_argument('--accel-timeout', type=float, default=30.0, help="Timeout in seconds for acceleration steps.")
    parser.add_argument('--decel-timeout', type=float, default=30.0, help="Timeout in seconds for deceleration steps.")

    args = parser.parse_args()
    if args.dynamic:
        generate_dynamic_plan(args)
    else:
        generate_calibration_plan(args)
