"""Plans for the first three vehicle-handling test phases."""

from dataclasses import asdict, dataclass

import yaml


@dataclass
class OpenLoopPlanConfig:
    """Parameters shared by steering step and slow-ramp identification."""

    output: str = "open_loop_identification.yaml"
    target_speed_mps: float = 2.0
    speed_tolerance_mps: float = 0.15
    amplitude: float = 10.0
    step_hold_sec: float = 8.0
    ramp_rate: float = 1.0
    max_steering: float = 40.0
    max_steering_rate: float = 50.0


def generate_phase1_plan(output=None, **kwargs):
    """Create the complete Phase 1 steering identification suite.

    The suite deliberately combines time-domain and frequency-domain
    excitation while keeping one vehicle-agnostic signal contract.
    """

    values = asdict(OpenLoopPlanConfig())
    values.update(kwargs)
    values["output"] = ""
    cases = generate_open_loop_identification_plan(**values)
    for mode, duration, suffix in (
            ("chirp", 30.0, "chirp"),
            ("prbs", 30.0, "prbs")):
        from whl_dyn.planning.vehicle_dynamics import (
            generate_lateral_frequency_plan,
        )
        for direction in (1.0, -1.0):
            cases.extend(generate_lateral_frequency_plan(
                output="", mode=mode, duration_sec=duration,
                steering_amplitude=abs(float(values["amplitude"])),
                target_speed_mps=values["target_speed_mps"],
                speed_tolerance_mps=values["speed_tolerance_mps"],
                max_steering=values["max_steering"],
                max_steering_rate=values["max_steering_rate"],
                case_name="phase1_{0}_{1:+g}".format(suffix, direction)))
    if output:
        with open(output, "w") as plan_file:
            yaml.safe_dump(cases, plan_file, sort_keys=False)
    return cases


def generate_open_loop_identification_plan(config=None, output=None, **kwargs):
    """Create step and slow-ramp cases for delay, deadzone and rate limits."""

    values = asdict(config or OpenLoopPlanConfig())
    values.update(kwargs)
    amplitude = float(values["amplitude"])
    ramp_rate = float(values["ramp_rate"])
    maximum_rate = float(values["max_steering_rate"])
    if amplitude <= 0.0 or ramp_rate <= 0.0 or ramp_rate > maximum_rate:
        raise ValueError("amplitude/ramp rate violates steering limits")
    common = {
        "domain": "vehicle_dynamics",
        "phase": "open_loop_identification",
        "actuator": "steering",
        # Match the simulator's 20 ms control interval so command samples
        # correspond one-to-one with applied control updates.
        "sampling_rate_hz": 50.0,
        "input_signals": ["steering_command", "steering_feedback"],
        "output_signals": ["yaw_rate_radps", "lateral_accel_mps2"],
        "speed_gate": {
            "min_mps": 0.0,
            "max_mps": max(float(values["target_speed_mps"]) + 1.0, 3.0),
            "target_mps": float(values["target_speed_mps"]),
            "tolerance_mps": float(values["speed_tolerance_mps"]),
            "stable_duration_sec": 3.0,
            "max_wait_sec": 30.0,
        },
        "safety_limits": {
            "max_abs_steering": float(values["max_steering"]),
            "max_steering_rate": maximum_rate,
        },
    }
    # The ramp spans -amplitude to +amplitude, so its duration is the
    # two-amplitude span divided by the requested command rate.
    ramp_duration = 2.0 * amplitude / ramp_rate
    cases = []
    for direction in (1.0, -1.0):
        cases.append(dict(common, **{
            "case_name": "steering_step_{0:+g}".format(direction * amplitude),
            "test_type": "steering_step",
            "duration_sec": float(values["step_hold_sec"]),
            "command_profile": {
                "type": "step", "baseline": 0.0,
                "amplitude": direction * amplitude, "start_time_sec": 1.0,
            },
            # A step intentionally excites the command input; the measured
            # wheel-rate limit, not the artificial command derivative, is the
            # identification result.
            "allow_command_step": True,
        }))
        cases.append(dict(common, **{
            "case_name": "steering_slow_ramp_{0:+g}".format(direction * amplitude),
            "test_type": "steering_deadzone_rate",
            "duration_sec": ramp_duration,
            "command_profile": {
                "type": "ramp", "baseline": -direction * amplitude,
                "target": direction * amplitude, "ramp_start_sec": 0.0,
                "ramp_end_sec": ramp_duration,
            },
        }))
    destination = output if output is not None else values.get("output")
    if destination:
        with open(destination, "w") as plan_file:
            yaml.safe_dump(cases, plan_file, sort_keys=False)
    return cases


@dataclass
class SteadyStateCircleConfig:
    """Open-loop fixed-steering matrix for steady-state handling tests."""

    output: str = "steady_state_circles.yaml"
    steering_commands: tuple = (1.0, 2.0, 3.0)
    speed_targets_mps: tuple = (1.0, 2.0, 3.0)
    turn_count: float = 1.0
    max_duration_sec: float = 120.0
    longitudinal_mode: str = "speed"
    throttle_command: float = 0.0
    stable_speed_sec: float = 5.0
    steering_ramp_rate: float = 5.0
    max_lateral_accel_mps2: float = 1.5
    repeats: int = 3


def generate_steady_state_circle_plan(config=None, output=None, **kwargs):
    """Create pure open-loop, fixed-steering steady-state test cases.

    Steering is established before the vehicle accelerates.  The collector
    then holds the speed and steering while recording the requested number of
    vehicle turns.  A fixed steering angle determines the actual radius, so
    radius and lateral acceleration are measurements, not commands.
    """

    values = asdict(config or SteadyStateCircleConfig())
    values.update(kwargs)
    cases = []
    ramp_rate = float(values["steering_ramp_rate"])
    if ramp_rate <= 0.0:
        raise ValueError("steering ramp rate must be positive")
    turn_count = float(values["turn_count"])
    if turn_count <= 0.0:
        raise ValueError("turn count must be positive")
    max_duration = float(values["max_duration_sec"])
    if max_duration <= 0.0:
        raise ValueError("maximum duration must be positive")
    longitudinal_mode = str(values["longitudinal_mode"]).lower()
    if longitudinal_mode not in ("speed", "throttle"):
        raise ValueError("longitudinal mode must be speed or throttle")
    throttle_command = float(values["throttle_command"])
    if longitudinal_mode == "throttle" and not 0.0 <= throttle_command <= 100.0:
        raise ValueError("throttle command must be between 0 and 100")
    stable_speed_sec = float(values["stable_speed_sec"])
    if stable_speed_sec <= 0.0:
        raise ValueError("stable speed duration must be positive")
    for command in values["steering_commands"]:
        if float(command) <= 0.0:
            raise ValueError("steering command magnitudes must be positive")
        for speed in values["speed_targets_mps"]:
            if float(speed) <= 0.0:
                raise ValueError("speed targets must be positive")
            for direction in (1.0, -1.0):
                for repeat in range(1, int(values["repeats"]) + 1):
                    target = direction * float(command)
                    ramp_duration = abs(target) / ramp_rate
                    cases.append({
                        "case_name": "steady_turn_d{0:g}_v{1:g}_{2}_r{3}".format(
                            command, speed,
                            "left" if direction > 0.0 else "right", repeat),
                        "domain": "vehicle_dynamics",
                        "phase": "steady_state_handling",
                        "test_type": "fixed_steering_steady_state",
                        "actuator": "steering",
                        "duration_sec": max_duration,
                        "turn_count": turn_count,
                        "steering_before_speed": True,
                        "longitudinal_control": dict(
                            {
                                "mode": longitudinal_mode,
                                "throttle": throttle_command,
                                "brake": 0.0,
                            },
                            **({"speed_mps": float(speed)}
                               if longitudinal_mode == "speed" else {}),
                        ),
                        "sampling_rate_hz": 100.0,
                        "input_signals": ["steering_command", "steering_feedback"],
                        "output_signals": ["yaw_rate_radps", "lateral_accel_mps2"],
                        "speed_gate": {
                            "min_mps": 0.0,
                            "max_mps": float(speed) + 1.0,
                            "target_mps": float(speed),
                            "tolerance_mps": 0.15,
                            "stability_tolerance_mps": 0.15,
                            "stability_min_in_band_fraction": 0.8,
                            "stable_duration_sec": stable_speed_sec,
                            "max_wait_sec": 30.0,
                        },
                        "safety_limits": {
                            "max_abs_steering": abs(target),
                            "max_abs_feedback_steering": abs(target) * 1.2,
                            "max_steering_rate": ramp_rate,
                            "max_abs_lateral_accel_mps2": float(
                                values["max_lateral_accel_mps2"]),
                        },
                        "command_profile": {
                            "type": "ramp",
                            "baseline": 0.0,
                            "target": target,
                            "ramp_start_sec": 0.0,
                            "ramp_end_sec": ramp_duration,
                        },
                    })
    destination = output if output is not None else values.get("output")
    if destination:
        with open(destination, "w") as plan_file:
            yaml.safe_dump(cases, plan_file, sort_keys=False)
    return cases


@dataclass
class ClosedLoopCurveConfig:
    """One clothoid-to-circle closed-loop tracking case."""

    output: str = "closed_loop_curve.yaml"
    radius_m: float = 50.0
    speed_mps: float = 2.0
    straight_entry_length_m: float = 20.0
    entry_length_m: float = 15.0
    arc_angle_rad: float = 1.57
    duration_sec: float = 0.0
    direction: str = "left"


def generate_closed_loop_curve_plan(config=None, output=None, **kwargs):
    """Create a continuous Clothoid -> circular arc -> Clothoid route case."""

    values = asdict(config or ClosedLoopCurveConfig())
    values.update(kwargs)
    direction = str(values["direction"]).lower()
    if direction not in ("left", "right"):
        raise ValueError("direction must be left or right")
    radius = float(values["radius_m"])
    speed = float(values["speed_mps"])
    if radius <= 0.0 or speed <= 0.0:
        raise ValueError("radius and speed must be positive")
    straight_entry = float(values["straight_entry_length_m"])
    entry_length = float(values["entry_length_m"])
    arc_angle = float(values["arc_angle_rad"])
    if straight_entry < 0.0 or entry_length <= 0.0 or arc_angle <= 0.0:
        raise ValueError("straight entry must be non-negative and curve lengths positive")
    transition_distance = straight_entry + entry_length + radius * arc_angle
    duration = float(values["duration_sec"])
    if duration <= 0.0:
        duration = transition_distance / speed
    case = {
        "case_name": "closed_loop_R{0:g}_{1}".format(
            float(values["radius_m"]), direction),
        "domain": "vehicle_dynamics",
        "phase": "closed_loop_constant_curvature",
        "test_type": "clothoid_circle_tracking",
        "duration_sec": duration,
        "max_alignment_skew_sec": 0.05,
        "trajectory": {
            "type": "clothoid_circle",
            "radius_m": radius,
            "speed_mps": speed,
            "straight_entry_length_m": straight_entry,
            "entry_length_m": entry_length,
            "arc_angle_rad": arc_angle,
            "direction": 1.0 if direction == "left" else -1.0,
            "horizon_sec": 8.0,
            "publish_rate_hz": 20.0,
        },
    }
    destination = output if output is not None else values.get("output")
    if destination:
        with open(destination, "w") as plan_file:
            yaml.safe_dump([case], plan_file, sort_keys=False)
    return [case]
