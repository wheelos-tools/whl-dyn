#!/usr/bin/env python3
"""
Command-line interface for whl-dyn.

This module provides the entry point for the 'whl-dyn' command.
It correctly launches streamlit without triggering warnings.
"""

import os
import argparse
import json
import sys
from pathlib import Path


def _parser():
    parser = argparse.ArgumentParser(prog="whl-dyn")
    subcommands = parser.add_subparsers(dest="command")
    plan = subcommands.add_parser("plan-lateral")
    plan.add_argument("--output", default="lateral_frequency_plan.yaml")
    plan.add_argument("--mode", choices=("pulse", "single_sine", "chirp", "sweep", "prbs"), default="chirp")
    plan.add_argument("--duration-sec", type=float, default=120.0)
    plan.add_argument("--sampling-rate-hz", type=float, default=100.0)
    plan.add_argument("--frequency-start-hz", type=float, default=0.05)
    plan.add_argument("--frequency-end-hz", type=float, default=2.0)
    plan.add_argument("--steering-amplitude", type=float, default=2.0)
    plan.add_argument("--speed-min-mps", type=float, default=0.0)
    plan.add_argument("--speed-max-mps", type=float, default=3.0)
    plan.add_argument("--target-speed-mps", type=float, default=2.0)
    plan.add_argument("--speed-tolerance-mps", type=float, default=0.15)
    plan.add_argument("--stable-speed-sec", type=float, default=3.0)
    plan.add_argument("--max-speed-wait-sec", type=float, default=30.0)
    plan.add_argument("--bit-duration-sec", type=float, default=0.25)
    plan.add_argument("--prbs-seed", type=int, default=7)
    plan.add_argument("--pulse-duration-sec", type=float, default=1.0)
    plan.add_argument("--sine-frequency-hz", type=float, default=0.5)
    plan.add_argument("--max-steering", type=float, default=20.0)
    plan.add_argument("--max-steering-rate", type=float, default=30.0)

    collect = subcommands.add_parser("collect-lateral")
    collect.add_argument("--plan", required=True)
    collect.add_argument("--signal-config", required=True)
    collect.add_argument("--output-dir", default="vehicle_dynamics_runs")
    collect.add_argument("--execute", action="store_true")
    collect.add_argument("--arm", action="store_true")
    validate = subcommands.add_parser("validate-open-loop")
    validate.add_argument("--plan", required=True)
    validate.add_argument("--signal-config")
    validate.add_argument("--active", action="store_true")

    analyze = subcommands.add_parser("analyze-lateral")
    analyze.add_argument("--run-dir", required=True)
    analyze.add_argument("--steering-column")
    analyze.add_argument("--sampling-rate-hz", type=float)
    open_loop = subcommands.add_parser("plan-open-loop")
    open_loop.add_argument("--output", default="open_loop_identification.yaml")
    open_loop.add_argument("--target-speed-mps", type=float, default=2.0)
    open_loop.add_argument("--amplitude", type=float, default=2.0)
    open_loop.add_argument("--ramp-rate", type=float, default=1.0)

    circles = subcommands.add_parser("plan-circles")
    circles.add_argument("--output", default="steady_state_circles.yaml")
    circles.add_argument("--steering-commands", default="1,2,3")
    circles.add_argument("--speed-targets-mps", default="1,2,3")
    circles.add_argument("--steering-ramp-rate", type=float, default=0.5)
    circles.add_argument("--max-lateral-accel-mps2", type=float, default=1.5)

    steady = subcommands.add_parser("analyze-steady-state")
    steady.add_argument("--run-dir", required=True)
    steady.add_argument("--target-speed-mps", required=True, type=float)
    steady.add_argument("--speed-tolerance-mps", type=float, default=0.15)
    steady.add_argument("--steering-wheel-column", default="steering_feedback")
    steady.add_argument("--road-wheel-column")
    steady.add_argument("--sideslip-column", default="sideslip_rad")
    steady.add_argument("--esc-column", default="esc_active")
    steady.add_argument("--wheel-slip-column", default="wheel_slip_ratio")
    steady.add_argument("--wheelbase-m", type=float)
    steady.add_argument("--max-abs-sideslip-rad", type=float)
    steady.add_argument("--max-sideslip-rate-radps", type=float)
    steady.add_argument("--max-yaw-rate-error-radps", type=float)
    steady.add_argument("--max-abs-wheel-slip", type=float)

    curve = subcommands.add_parser("plan-closed-loop")
    curve.add_argument("--output", default="closed_loop_curve.yaml")
    curve.add_argument("--radius-m", type=float, default=50.0)
    curve.add_argument("--speed-mps", type=float, default=2.0)
    curve.add_argument("--straight-entry-length-m", type=float, default=20.0)
    curve.add_argument("--entry-length-m", type=float, default=15.0)
    curve.add_argument("--arc-angle-rad", type=float, default=1.57)
    curve.add_argument("--duration-sec", type=float, default=0.0)
    curve.add_argument("--direction", choices=("left", "right"), default="left")

    run_curve = subcommands.add_parser("run-closed-loop")
    run_curve.add_argument("--plan", required=True)
    run_curve.add_argument("--output-dir", default="vehicle_dynamics_runs")
    return parser


def _run_lateral_collection(args):
    import yaml
    from cyber.python.cyber_py3 import cyber
    from whl_dyn.collection.lateral import LateralSignalCollector, load_signal_config

    with open(args.plan) as plan_file:
        plan = yaml.safe_load(plan_file) or []
    if not isinstance(plan, list) or not plan:
        raise ValueError("plan must contain at least one case")
    if args.execute and not args.arm:
        raise ValueError("--execute requires --arm")
    from whl_dyn.planning.preflight import (
        validate_active_signal_config,
        validate_open_loop_plan,
    )

    validate_open_loop_plan(plan)
    signal_config = load_signal_config(args.signal_config)
    if args.execute:
        validate_active_signal_config(signal_config)
    cyber.init("whl_dyn_lateral_collector")
    try:
        collector = LateralSignalCollector(
            cyber.Node("whl_dyn_lateral_collector"),
            signal_config,
        )
        collector.subscribe()
        for case in plan:
            run_path = collector.collect_case(
                case, args.output_dir, execute=args.execute, arm=args.arm)
            print(run_path)
    finally:
        cyber.shutdown()


def _validate_open_loop(args):
    import yaml
    from whl_dyn.planning.preflight import (
        validate_active_signal_config,
        validate_open_loop_plan,
    )

    with open(args.plan) as plan_file:
        plan = yaml.safe_load(plan_file) or []
    validate_open_loop_plan(plan)
    if args.active:
        if not args.signal_config:
            raise ValueError("--active requires --signal-config")
        from whl_dyn.collection.lateral import load_signal_config

        validate_active_signal_config(load_signal_config(args.signal_config))
    print("open-loop preflight passed")


def _float_list(value):
    return tuple(float(item) for item in str(value).split(",") if item.strip())


def _run_closed_loop(args):
    import yaml
    from cyber.python.cyber_py3 import cyber
    from whl_dyn.collection.closed_loop import ClosedLoopTrajectoryRunner

    with open(args.plan) as plan_file:
        cases = yaml.safe_load(plan_file) or []
    if not isinstance(cases, list) or not cases:
        raise ValueError("plan must contain at least one case")
    cyber.init("whl_dyn_closed_loop_trajectory")
    try:
        runner = ClosedLoopTrajectoryRunner(cyber.Node("whl_dyn_closed_loop_trajectory"))
        runner.subscribe()
        for case in cases:
            print(runner.run_case(case, args.output_dir))
    finally:
        cyber.shutdown()


def _launch_ui():
    """Launch the whl-dyn Streamlit application."""
    # Get the package root directory
    # This file is at whl_dyn/cli.py
    # The app.py is at whl_dyn/ui/app.py
    package_root = Path(__file__).parent

    # Path to app.py
    app_py = package_root / "ui" / "app.py"

    # Build streamlit command
    cmd = [
        sys.executable,
        "-m",
        "streamlit",
        "run",
        str(app_py),
        "--server.headless",
        "true",
        "--server.runOnSave",
        "false",
        "--server.fileWatcherType",
        "none",
    ] + sys.argv[1:]

    # Replace current process with streamlit
    # This prevents multiple processes from being created
    os.execvp(sys.executable, cmd)


def main():
    """Run a lateral-data command or launch the Streamlit workbench."""

    known_commands = {
        "plan-lateral", "collect-lateral", "validate-open-loop", "analyze-lateral", "plan-open-loop",
        "plan-circles", "analyze-steady-state", "plan-closed-loop", "run-closed-loop",
    }
    if len(sys.argv) > 1 and sys.argv[1] in known_commands:
        args = _parser().parse_args()
        if args.command == "plan-lateral":
            from whl_dyn.planning.vehicle_dynamics import generate_lateral_frequency_plan

            generate_lateral_frequency_plan(
                output=args.output,
                mode=args.mode,
                duration_sec=args.duration_sec,
                sampling_rate_hz=args.sampling_rate_hz,
                frequency_start_hz=args.frequency_start_hz,
                frequency_end_hz=args.frequency_end_hz,
                steering_amplitude=args.steering_amplitude,
                speed_min_mps=args.speed_min_mps,
                speed_max_mps=args.speed_max_mps,
                target_speed_mps=args.target_speed_mps,
                speed_tolerance_mps=args.speed_tolerance_mps,
                stable_speed_sec=args.stable_speed_sec,
                max_speed_wait_sec=args.max_speed_wait_sec,
                bit_duration_sec=args.bit_duration_sec,
                prbs_seed=args.prbs_seed,
                pulse_duration_sec=args.pulse_duration_sec,
                sine_frequency_hz=args.sine_frequency_hz,
                max_steering=args.max_steering,
                max_steering_rate=args.max_steering_rate,
            )
            print(args.output)
            return
        if args.command == "collect-lateral":
            _run_lateral_collection(args)
            return
        if args.command == "analyze-steady-state":
            import pandas as pd
            from whl_dyn.processing.handling import (
                fixed_steering_steady_state_metrics,
                select_steady_state_samples,
            )

            run_dir = Path(args.run_dir)
            selected = select_steady_state_samples(
                pd.read_csv(run_dir / "samples.csv"),
                args.target_speed_mps, args.speed_tolerance_mps)
            metrics = fixed_steering_steady_state_metrics(
                selected, steering_wheel_column=args.steering_wheel_column,
                road_wheel_column=args.road_wheel_column,
                sideslip_column=args.sideslip_column, esc_column=args.esc_column,
                wheel_slip_column=args.wheel_slip_column, wheelbase_m=args.wheelbase_m,
                max_abs_sideslip_rad=args.max_abs_sideslip_rad,
                max_sideslip_rate_radps=args.max_sideslip_rate_radps,
                max_yaw_rate_error_radps=args.max_yaw_rate_error_radps,
                max_abs_wheel_slip=args.max_abs_wheel_slip)
            output = run_dir / "analysis"
            output.mkdir(exist_ok=True)
            path = output / "steady_state_metrics.json"
            path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
            print(path)
            return
        if args.command == "validate-open-loop":
            _validate_open_loop(args)
            return
        if args.command == "plan-open-loop":
            from whl_dyn.planning.handling import generate_open_loop_identification_plan

            generate_open_loop_identification_plan(
                output=args.output, target_speed_mps=args.target_speed_mps,
                amplitude=args.amplitude, ramp_rate=args.ramp_rate)
            print(args.output)
            return
        if args.command == "plan-circles":
            from whl_dyn.planning.handling import generate_steady_state_circle_plan

            generate_steady_state_circle_plan(
                output=args.output,
                steering_commands=_float_list(args.steering_commands),
                speed_targets_mps=_float_list(args.speed_targets_mps),
                steering_ramp_rate=args.steering_ramp_rate,
                max_lateral_accel_mps2=args.max_lateral_accel_mps2)
            print(args.output)
            return
        if args.command == "plan-closed-loop":
            from whl_dyn.planning.handling import generate_closed_loop_curve_plan

            generate_closed_loop_curve_plan(
                output=args.output, radius_m=args.radius_m,
                speed_mps=args.speed_mps,
                straight_entry_length_m=args.straight_entry_length_m,
                entry_length_m=args.entry_length_m,
                arc_angle_rad=args.arc_angle_rad,
                duration_sec=args.duration_sec, direction=args.direction)
            print(args.output)
            return
        if args.command == "run-closed-loop":
            _run_closed_loop(args)
            return
        from whl_dyn.processing.lateral_dynamics import write_lateral_frequency_report

        output, summary = write_lateral_frequency_report(
            args.run_dir, args.steering_column, args.sampling_rate_hz)
        print(output)
        print(summary)
        return
    _launch_ui()


if __name__ == "__main__":
    main()
