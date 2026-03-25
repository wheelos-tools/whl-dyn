import argparse
import yaml
import numpy as np

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
                        'timeout_sec': args.accel_timeout
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
                    'command': {'throttle': 80.0, 'brake': 0.0}, # A strong throttle to get to speed quickly
                    'trigger': {'type': 'speed_greater_than', 'value': float(initial_speed_target)},
                    'timeout_sec': args.accel_timeout
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
    parser.add_argument('--default-brake', type=float, default=30.0, help="Default brake command (%) used to stop the vehicle after a test step.")

    # Safety and timeout
    parser.add_argument('--accel-timeout', type=float, default=30.0, help="Timeout in seconds for acceleration steps.")
    parser.add_argument('--decel-timeout', type=float, default=30.0, help="Timeout in seconds for deceleration steps.")

    args = parser.parse_args()
    generate_calibration_plan(args)
