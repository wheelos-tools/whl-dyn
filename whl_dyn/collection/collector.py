#!/usr/bin/env python3
"""
Production-Ready, Plan-Driven Data Collector for Longitudinal Calibration

This script automates vehicle dynamics data collection by executing a predefined
YAML test plan. It incorporates fail-safes, robust state management, and clear
operator feedback, adhering to industry best practices.
"""

import argparse
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.control_msgs import control_cmd_pb2
from modules.common_msgs.localization_msgs import localization_pb2


# --- Use dataclasses for clear state management ---
@dataclass
class VehicleState:
    """Snapshot of all relevant vehicle data at a point in time"""
    timestamp: float = 0.0
    speed_mps: float = 0.0
    imu_accel_y: float = 0.0
    driving_mode: int = 0
    actual_gear: int = 0
    throttle_pct: float = 0.0
    brake_pct: float = 0.0


@dataclass
class ControlState:
    """Stores the last sent control command"""
    throttle: float = 0.0
    brake: float = 0.0
    gear: int = chassis_pb2.Chassis.GEAR_DRIVE
    # TODO(leafyleong): re-enable after motion mode supported
    # motion_mode: int = chassis_pb2.Chassis.MOTION_ACKERMANN


class AdvancedDataCollector:
    """Ensures data quality and automation by executing calibration plans"""

    def __init__(self, node, output_dir="./calibration_data_logs", auto_start=False):
        """Initialization"""
        self.node = node
        self.control_pub = node.create_writer('/apollo/control',
                                              control_cmd_pb2.ControlCommand)
        self.output_dir = output_dir
        self.auto_start = auto_start

        # State management variables
        self.vehicle_state = VehicleState()
        self.last_sent_control = ControlState()
        self.localization_received = False
        self.chassis_received = False

        # Plan execution variables
        self.plan = None
        self.active_case = None
        self.active_step_idx = 0
        self.step_start_time = 0.0
        self.is_collecting = False
        self.output_file = None
        self.sequence_num = 0
        self.abort_signal_received = False

        time.sleep(0.5)

    def setup_and_run(self, plan_path: str):
        """Main entry point for loading, checking, and running the plan"""
        if not self._load_plan(plan_path):
            return
        self._setup_subscriptions()
        # enter to auto driving mode and stop the vehicle
        self._send_control_command(safe_stop=True)
        if not self.check_vehicle_ready():
            return
        self.run_plan()

    def _load_plan(self, plan_path: str) -> bool:
        """Load and validate calibration plan from YAML file"""
        try:
            with open(plan_path, 'r') as f:
                self.plan = yaml.safe_load(f)
            print(f"OK: Calibration plan loaded from '{plan_path}'")
            return True
        except (FileNotFoundError, yaml.YAMLError) as e:
            print(f"ERROR: Failed to load or parse plan file: {e}")
            return False

    def _setup_subscriptions(self):
        """Initialize all CyberRT subscribers"""
        self.node.create_reader('/apollo/localization/pose',
                                localization_pb2.LocalizationEstimate,
                                self._callback_localization)
        self.node.create_reader('/apollo/canbus/chassis', chassis_pb2.Chassis,
                                self._callback_chassis)

    def check_vehicle_ready(self, timeout_sec=10) -> bool:
        """Ensure the vehicle is in a safe, ready state for testing"""
        print("INFO: Checking vehicle readiness...")
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if not (self.localization_received and self.chassis_received):
                print("  - Waiting for localization and chassis messages...")
            elif self.vehicle_state.driving_mode != chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE:
                mode_name = chassis_pb2.Chassis.DrivingMode.Name(
                    self.vehicle_state.driving_mode)
                print(
                    f"  - Warning: Vehicle not in auto drive mode (Current: {mode_name})."
                )
            elif abs(self.vehicle_state.speed_mps) > 0.1:
                print(
                    f'  - Warning: Vehicle not stationary (Current speed: {self.vehicle_state.speed_mps:.2f} m/s).'
                )
            else:
                print("OK: Vehicle is ready.")
                return True
            time.sleep(1)
        print("ERROR: Vehicle readiness check timed out.")
        return False

    def run_plan(self):
        """Execute all test cases defined in the loaded plan"""
        for i, case_config in enumerate(self.plan):
            if self.abort_signal_received:
                break
            print(f"\n{'='*80}")
            print(
                f"INFO: Preparing case {i+1}/{len(self.plan)}: {case_config['case_name']}"
            )
            print(
                f"      Description: {case_config.get('description', 'N/A')}")

            if not self.auto_start:
                user_input = input(
                    "      Press Enter to start, 's' to skip, 'q' to quit: "
                ).lower()
                if user_input == 's':
                    continue
                if user_input == 'q':
                    break
            else:
                print("INFO: Auto-start enabled, executing case immediately.")

            self._execute_case(case_config)

        print(f"\n{'='*80}\nINFO: Calibration plan execution completed.")

    def _execute_case(self, case_config: dict):
        """Manage the lifecycle of a single data collection case"""
        self.active_case = case_config
        self.active_step_idx = 0

        if not self._prepare_output_file(case_config['case_name']):
            return

        with open(self.output_file_path, 'w') as f:
            self.output_file = f
            self._write_header()

            self._send_control_command(reset=True)
            time.sleep(0.2)

            self.is_collecting = True
            self.step_start_time = cyber_time.Time.now().to_sec()

            while self.is_collecting and not self.abort_signal_received and cyber.ok(
            ):
                loop_start_time = cyber_time.Time.now().to_sec()
                self._state_machine_tick()
                sleep_time = 0.01 - (cyber_time.Time.now().to_sec() -
                                     loop_start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        self.output_file = None
        if not self.abort_signal_received:
            # Clear last live status line
            sys.stdout.write("\r" + " " * 80 + "\r")
            sys.stdout.flush()
            print(f"OK: Case data saved to '{self.output_file_path}'")

    def _prepare_output_file(self, case_name: str) -> bool:
        """Create a unique, descriptive output filename"""
        try:
            # Use the output directory from instance variable or default to "./calibration_data_logs"
            output_dir = Path(
                getattr(self, 'output_dir', "./calibration_data_logs"))
            output_dir.mkdir(exist_ok=True)
            i = 0
            while True:
                filename = f"{case_name}_{i}.csv"
                filepath = output_dir / filename
                if not filepath.exists():
                    self.output_file_path = filepath
                    return True
                i += 1
        except OSError as e:
            print(f"ERROR: Unable to create output directory or file: {e}")
            return False

    def _write_header(self):
        """Write CSV file header"""
        self.output_file.write(
            "time,speed_mps,imu_accel_y,driving_mode,actual_gear,"
            "throttle_pct,brake_pct,ctl_throttle,ctl_brake\n")

    def _print_live_status(self):
        """Print and refresh live status line in terminal"""
        if not self.active_case: return
        step = self.active_case['steps'][self.active_step_idx]
        cmd = step['command']
        trigger = step['trigger']
        elapsed = time.time() - self.step_start_time

        status_str = (
            f"\r>> Step {self.active_step_idx + 1}: "
            f"Speed: {self.vehicle_state.speed_mps:5.2f} m/s | "
            f"Trigger: {trigger['type'].replace('_', ' ')} {trigger['value']:.1f} | "
            f"Command: Throttle={cmd.get('throttle', 0):.0f}% Brake={cmd.get('brake', 0):.0f}% | "
            f"Elapsed: {elapsed:4.1f}s / {step['timeout_sec']:.0f}s")
        sys.stdout.write(status_str)
        sys.stdout.flush()

    def _state_machine_tick(self):
        """Core logic of the state machine, handles state transitions and command publishing"""
        self._print_live_status()

        current_step = self.active_case['steps'][self.active_step_idx]

        # Check if step timed out
        if time.time() - self.step_start_time > current_step['timeout_sec']:
            sys.stdout.write("\r" + " " * 80 + "\r")  # Clear status line
            print(f"ERROR: Step timed out. Aborting current case.")
            self.is_collecting = False
            self._send_control_command(safe_stop=True)
            return

        # Check if trigger condition is met
        trigger = current_step['trigger']
        speed = self.vehicle_state.speed_mps
        trigger_met = False
        if trigger['type'] == 'speed_greater_than' and speed > trigger['value']:
            trigger_met = True
        elif trigger['type'] == 'speed_less_than' and speed < trigger['value']:
            trigger_met = True

        if trigger_met:
            sys.stdout.write("\r" + " " * 80 + "\r")  # Clear status line
            print(f"INFO: Trigger met at speed {speed:.2f} m/s.")
            if self.active_step_idx + 1 < len(self.active_case['steps']):
                self.active_step_idx += 1
                self.step_start_time = time.time()
                print(f"      Entering step {self.active_step_idx + 1}...")
            else:
                self.is_collecting = False
                self._send_control_command(default=True)
                return

        # Publish current step command
        self._send_control_command(command_dict=current_step['command'])

    def _send_control_command(self,
                              command_dict=None,
                              reset=False,
                              default=False,
                              safe_stop=False):
        """Construct and publish ControlCommand message"""
        cmd = control_cmd_pb2.ControlCommand()
        cmd.header.module_name = "advanced_collector"
        cmd.header.sequence_num = self.sequence_num
        cmd.header.timestamp_sec = cyber_time.Time.now().to_sec()

        if reset:
            cmd.pad_msg.action = 2
        else:
            cmd.pad_msg.action = 1
            if default:
                self.last_sent_control = ControlState(throttle=0.0, brake=0.0)
            elif safe_stop:
                # Define a safe stop command
                self.last_sent_control = ControlState(throttle=0.0, brake=30.0)
            elif command_dict:
                self.last_sent_control = ControlState(
                    throttle=float(command_dict.get('throttle', 0.0)),
                    brake=float(command_dict.get('brake', 0.0)))

        cmd.throttle = self.last_sent_control.throttle
        cmd.brake = self.last_sent_control.brake
        cmd.gear_location = self.last_sent_control.gear
        # TODO(leafyleong): re-enable after motion mode supported
        # cmd.motion_mode = self.last_sent_control.motion_mode

        self.control_pub.write(cmd)
        self.sequence_num += 1

    def emergency_stop(self):
        """Called by signal handler to safely stop collection"""
        print(
            "\nINFO: Emergency stop signal received. Sending safe command...")
        self.abort_signal_received = True
        self.is_collecting = False
        self._send_control_command(safe_stop=True)
        print("INFO: Safe stop command sent. Collected data will be saved.")

    def _callback_localization(self,
                               data: localization_pb2.LocalizationEstimate):
        """Handle localization messages"""
        self.vehicle_state.imu_accel_y = data.pose.linear_acceleration_vrf.y
        self.localization_received = True

    def _callback_chassis(self, data: chassis_pb2.Chassis):
        """Handle chassis messages, main trigger for writing data"""
        self.vehicle_state = VehicleState(
            timestamp=data.header.timestamp_sec,
            speed_mps=data.speed_mps,
            imu_accel_y=self.vehicle_state.imu_accel_y,
            driving_mode=data.driving_mode,
            actual_gear=data.gear_location,
            throttle_pct=data.throttle_percentage,
            brake_pct=data.brake_percentage,
        )
        self.chassis_received = True

        if self.is_collecting and self.output_file and not self.output_file.closed:
            self._write_log_entry()

    def _write_log_entry(self):
        """Write a complete, atomic snapshot of vehicle state to file"""
        vs = self.vehicle_state
        cs = self.last_sent_control
        self.output_file.write(
            f"{vs.timestamp:.4f},{vs.speed_mps:.4f},{vs.imu_accel_y:.4f},"
            f"{vs.driving_mode},{vs.actual_gear},{vs.throttle_pct:.2f},"
            f"{vs.brake_pct:.2f},{cs.throttle:.2f},{cs.brake:.2f}\n")


def main():
    """Main function, runs the data collection process"""
    parser = argparse.ArgumentParser(
        description="Production-Ready, Plan-Driven Data Collector for Apollo.")
    parser.add_argument(
        "-p",
        "--plan",
        type=str,
        default="calibration_plan.yaml",
        help=
        "Path to the YAML calibration plan file (default: calibration_plan.yaml)"
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=str,
        default="./calibration_data_logs",
        help=
        "Output directory for collected data files (default: ./calibration_data_logs)"
    )
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Start each case automatically without interactive prompt."
    )
    args = parser.parse_args()

    cyber.init()
    node = cyber.Node("advanced_calibration_collector")
    collector = AdvancedDataCollector(node, output_dir=args.output_dir, auto_start=args.auto_start)

    # --- Robust shutdown handler ---
    def shutdown_handler(signum, frame):
        collector.emergency_stop()
        time.sleep(1)  # Wait for stop command to be sent
        cyber.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    collector.setup_and_run(args.plan)
    cyber.shutdown()


if __name__ == '__main__':
    main()
