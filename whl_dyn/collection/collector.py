#!/usr/bin/env python3
"""
Production-Ready, Plan-Driven Data Collector for Longitudinal Calibration

This script automates vehicle dynamics data collection by executing a predefined
YAML test plan. It incorporates fail-safes, robust state management, and clear
operator feedback, adhering to industry best practices.
"""

import argparse
import math
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
SUPPORTED_DYNAMIC_PROFILES = (
    "step", "ramp", "pulse", "triangle", "hysteresis",
    "single_sine", "chirp", "sweep", "multi_sine",
)


def _profile_number(profile, names, default=0.0):
    """Return the first present numeric profile parameter."""
    for name in names:
        if name in profile and profile[name] is not None:
            return float(profile[name])
    return float(default)


def evaluate_command_profile(profile, elapsed_sec):
    """Evaluate a generic open-loop command profile at elapsed time.

    This pure helper is kept independent of CyberRT so plans can be validated
    and tested on a workstation.  The collector clamps the resulting command
    to the actuator's valid range before publishing it.
    """
    if not isinstance(profile, dict):
        raise ValueError("command profile must be a mapping")
    profile_type = str(profile.get(
        "type", profile.get("profile_type", profile.get("mode", "step")))).lower()
    profile_type = profile_type.replace("-", "_").replace(" ", "_")
    if profile_type not in SUPPORTED_DYNAMIC_PROFILES:
        raise ValueError("unsupported dynamic profile: {0}".format(profile_type))

    t = max(0.0, float(elapsed_sec))
    baseline = _profile_number(profile, ("baseline", "offset", "start_value"), 0.0)
    amplitude = _profile_number(profile, ("amplitude", "level", "height", "peak"), 0.0)
    start = _profile_number(profile, ("start_time_sec", "start_sec"), 0.0)
    end = _profile_number(profile, ("end_time_sec", "end_sec"), start)

    if profile_type == "step":
        return baseline if t < start else baseline + amplitude
    if profile_type == "ramp":
        ramp_start = _profile_number(profile, ("ramp_start_sec", "start_time_sec"), 0.0)
        ramp_end = _profile_number(profile, ("ramp_end_sec", "end_time_sec"), 1.0)
        target = _profile_number(profile, ("end_value", "target", "final_value"),
                                 baseline + amplitude)
        if t <= ramp_start:
            return baseline
        if t >= ramp_end:
            return target
        fraction = (t - ramp_start) / max(ramp_end - ramp_start, 1e-9)
        return baseline + fraction * (target - baseline)
    if profile_type == "pulse":
        pulse_start = _profile_number(profile, ("pulse_start_sec", "start_time_sec"), 0.0)
        pulse_duration = _profile_number(profile, ("pulse_duration_sec", "width_sec"), 1.0)
        return baseline + amplitude if pulse_start <= t < pulse_start + pulse_duration else baseline
    if profile_type == "triangle":
        period = max(_profile_number(profile, ("period_sec", "period"), 2.0), 1e-9)
        low = _profile_number(profile, ("min_value", "low"), baseline - abs(amplitude))
        high = _profile_number(profile, ("max_value", "high"), baseline + abs(amplitude))
        phase = (t - start) % period / period
        value = 2.0 * phase if phase < 0.5 else 2.0 * (1.0 - phase)
        return low + (high - low) * value
    if profile_type == "hysteresis":
        period = max(_profile_number(profile, ("period_sec", "period"), 2.0), 1e-9)
        low = _profile_number(profile, ("low", "min_value"), baseline)
        high = _profile_number(profile, ("high", "max_value"), baseline + amplitude)
        return high if ((t - start) % period) >= period / 2.0 else low
    if profile_type == "single_sine":
        frequency = _profile_number(profile, ("frequency_hz", "frequency"), 1.0)
        phase = _profile_number(profile, ("phase_rad", "phase"), 0.0)
        return baseline + amplitude * math.sin(2.0 * math.pi * frequency * t + phase)
    if profile_type in ("chirp", "sweep"):
        f0 = _profile_number(profile, ("frequency_start_hz", "f0_hz", "start_frequency_hz"), 0.1)
        f1 = _profile_number(profile, ("frequency_end_hz", "f1_hz", "end_frequency_hz"), 2.0)
        sweep_duration = max(_profile_number(profile, ("sweep_duration_sec", "duration_sec"), 10.0), 1e-9)
        local_t = min(t, sweep_duration)
        method = str(profile.get("method", "linear")).lower()
        if (method in ("log", "logarithmic", "exponential") and
                f0 > 0 and f1 > 0 and abs(f1 - f0) > 1e-12):
            ratio = f1 / f0
            phase = 2.0 * math.pi * f0 * sweep_duration / math.log(ratio)
            phase *= math.pow(ratio, local_t / sweep_duration) - 1.0
        else:
            phase = 2.0 * math.pi * (f0 * local_t +
                                     0.5 * (f1 - f0) * local_t * local_t / sweep_duration)
        return baseline + amplitude * math.sin(phase)
    # multi_sine
    frequencies = profile.get("frequencies_hz", profile.get("frequencies", [1.0]))
    amplitudes = profile.get("amplitudes", [amplitude] * len(frequencies))
    phases = profile.get("phases_rad", profile.get("phases", []))
    result = baseline
    for index, frequency in enumerate(frequencies):
        amp = float(amplitudes[index]) if index < len(amplitudes) else amplitude
        phase = float(phases[index]) if index < len(phases) else 0.0
        result += amp * math.sin(2.0 * math.pi * float(frequency) * t + phase)
    return result


def is_dynamic_case(case_config):
    """Identify a dynamic case while accepting old calibration case schemas."""
    return bool(case_config.get("dynamic") or case_config.get("domain") in
                ("actuator_characterization", "frequency_response") or
                "command_profile" in case_config or "profile" in case_config)


@dataclass
class VehicleState:
    """Snapshot of all relevant vehicle data at a point in time"""
    timestamp: float = 0.0
    speed_mps: float = 0.0
    ins_speed_mps: float = 0.0
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

    def __init__(self,
                 node,
                 output_dir="./calibration_data_logs",
                 auto_start=False):
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
        self.trigger_met_time = None  # Track when trigger condition was met
        self.dynamic_start_time = 0.0

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
                loaded_plan = yaml.safe_load(f)
            if isinstance(loaded_plan, dict):
                loaded_plan = loaded_plan.get("cases", [loaded_plan])
            self.plan = loaded_plan or []
            if not isinstance(self.plan, list):
                raise yaml.YAMLError("plan must be a case list or a case mapping")
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
        self.trigger_met_time = None  # Reset trigger time for new case
        self.dynamic_start_time = 0.0

        if not self._prepare_output_file(case_config['case_name']):
            return

        with open(self.output_file_path, 'w') as f:
            self.output_file = f
            self._write_header()

            self._send_control_command(reset=True)
            time.sleep(0.2)

            self.is_collecting = True
            self.step_start_time = cyber_time.Time.now().to_sec()
            self.dynamic_start_time = time.monotonic()
            sample_rate = float(case_config.get("sampling_rate_hz", 100.0))
            loop_period = 1.0 / max(sample_rate, 1.0)

            while self.is_collecting and not self.abort_signal_received and cyber.ok(
            ):
                loop_start_time = cyber_time.Time.now().to_sec()
                self._state_machine_tick()
                sleep_time = loop_period - (cyber_time.Time.now().to_sec() -
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
        header = (
            "time,speed_mps,ins_speed_mps,imu_accel_y,driving_mode,actual_gear,"
            "throttle_pct,brake_pct,ctl_throttle,ctl_brake"
        )
        if is_dynamic_case(self.active_case):
            header += ",command,case_name,domain,mode,actuator,profile_type"
        self.output_file.write(header + "\n")

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

    def _dynamic_profile(self):
        return self.active_case.get("command_profile", self.active_case.get("profile", {}))

    def _dynamic_command(self, elapsed_sec):
        profile = self._dynamic_profile()
        value = evaluate_command_profile(profile, elapsed_sec)
        actuator = str(self.active_case.get("actuator", "throttle")).lower()
        value = max(0.0, min(100.0, value))
        if actuator == "brake":
            return {"throttle": 0.0, "brake": value}
        if actuator == "both":
            return {"throttle": value if value >= 0.0 else 0.0,
                    "brake": abs(value) if value < 0.0 else 0.0}
        return {"throttle": value, "brake": 0.0}

    def _dynamic_state_machine_tick(self):
        """Run a timed profile without consulting vehicle speed triggers."""
        elapsed = time.monotonic() - self.dynamic_start_time
        profile = self._dynamic_profile()
        duration = float(self.active_case.get(
            "duration_sec", profile.get("duration_sec", 0.0)))
        if duration > 0.0 and elapsed >= duration:
            self.is_collecting = False
            self._send_control_command(safe_stop=True)
            return
        self._send_control_command(command_dict=self._dynamic_command(elapsed))

    def _state_machine_tick(self):
        """Core logic of the state machine, handles state transitions and command publishing"""
        if is_dynamic_case(self.active_case):
            self._dynamic_state_machine_tick()
            return
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
            if self.trigger_met_time is None:
                # First time trigger is met
                self.trigger_met_time = time.time()
                hold_duration_ms = current_step.get('hold_duration_ms', 0)
                if hold_duration_ms > 0:
                    sys.stdout.write("\r" + " " * 80 + "\r")
                    print(
                        f"INFO: Trigger met at speed {speed:.2f} m/s, holding for {hold_duration_ms}ms..."
                    )
            else:
                # Trigger was already met, check if hold duration has passed
                hold_duration_ms = current_step.get('hold_duration_ms', 0)
                hold_duration_sec = hold_duration_ms / 1000.0

                if time.time() - self.trigger_met_time >= hold_duration_sec:
                    # Hold duration complete, transition to next step
                    sys.stdout.write("\r" + " " * 80 + "\r")
                    print(f"INFO: Hold complete at speed {speed:.2f} m/s.")
                    if self.active_step_idx + 1 < len(
                            self.active_case['steps']):
                        self.active_step_idx += 1
                        self.step_start_time = time.time()
                        self.trigger_met_time = None  # Reset for next step
                        print(
                            f"      Entering step {self.active_step_idx + 1}..."
                        )
                    else:
                        self.is_collecting = False
                        self._send_control_command(default=True)
                        return
        else:
            # Trigger condition not met, reset trigger time if it was set
            # (in case speed drops below threshold after being met)
            pass

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
        # Note: linear_velocity_vrf field does NOT exist in localization protobuf
        # Calculate speed from linear_velocity (map reference frame)
        if (hasattr(data.pose, 'linear_velocity')
                and data.pose.HasField('linear_velocity')):
            vx = data.pose.linear_velocity.x
            vy = data.pose.linear_velocity.y
            # Use magnitude for actual vehicle speed (speedometer equivalent)
            self.vehicle_state.ins_speed_mps = (vx**2 + vy**2)**0.5
        self.localization_received = True

    def _callback_chassis(self, data: chassis_pb2.Chassis):
        """Handle chassis messages, main trigger for writing data"""
        self.vehicle_state = VehicleState(
            timestamp=data.header.timestamp_sec,
            speed_mps=data.speed_mps,
            ins_speed_mps=self.vehicle_state.ins_speed_mps,
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
        row = (
            f"{vs.timestamp:.4f},{vs.speed_mps:.4f},{vs.ins_speed_mps:.4f},{vs.imu_accel_y:.4f},"
            f"{vs.driving_mode},{vs.actual_gear},{vs.throttle_pct:.2f},"
            f"{vs.brake_pct:.2f},{cs.throttle:.2f},{cs.brake:.2f}"
        )
        if is_dynamic_case(self.active_case):
            row += (
                f",{cs.throttle - cs.brake:.2f},"
                f"{self.active_case.get('case_name', '')},"
                f"{self.active_case.get('domain', 'calibration')},"
                f"{self.active_case.get('mode', 'calibration')},"
                f"{self.active_case.get('actuator', '')},"
                f"{self._dynamic_profile().get('type', '')}"
            )
        self.output_file.write(row + "\n")


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
        help="Start each case automatically without interactive prompt.")
    args = parser.parse_args()

    cyber.init()
    node = cyber.Node("advanced_calibration_collector")
    collector = AdvancedDataCollector(node,
                                      output_dir=args.output_dir,
                                      auto_start=args.auto_start)

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
