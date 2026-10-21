#!/usr/bin/env python3
"""
Plan-Driven Data Collector with Selectable Braking Interface

This script automates vehicle dynamics data collection by executing a predefined
YAML test plan. Adds selectable pedal/deceleration commands to the uploaded single-loop version.
Each case now enters AUTO, collects, stops, parks, resets and confirms MANUAL.
Single publishing loop; no background publishing thread or service.
"""

import argparse
import fcntl
import json
import os
import tempfile
import math
import signal
import sys
import time
import threading
from dataclasses import dataclass, replace
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
    ins_speed_mps: float = 0.0
    imu_accel_y: float = 0.0
    driving_mode: int = 0
    actual_gear: int = 0
    throttle_pct: float = 0.0
    brake_pct: float = 0.0
    parking_brake: bool = False
    error_code: int = -1


@dataclass
class ControlState:
    """Stores the last sent control command"""
    throttle: float = 0.0
    brake: float = 0.0
    gear: int = chassis_pb2.Chassis.GEAR_DRIVE
    acceleration: float = 0.0  # 实际发送值，m/s²；驱动阶段明确清零。
    brake_sent: float = 0.0  # 实际ControlCommand.brake；brake保留旧CSV请求轴。
    # TODO(leafyleong): re-enable after motion mode supported
    # motion_mode: int = chassis_pb2.Chassis.MOTION_ACKERMANN


def finite_number(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(name + "必须是有限数值")
    return value


def normalize_braking(command, interface, fullscale_decel, max_decel):
    """通用接口适配。pedal保留原百分比；deceleration将请求转为负加速度。

    brake: 百分比；deceleration: 正的m/s²。两个非零/显式制动接口不能混用。
    brake保留等效百分比用于旧CSV；brake_sent才是实际消息字段。
    """
    if not isinstance(command, dict):
        raise ValueError("command必须是字典")
    unknown = set(command) - {"throttle", "brake", "deceleration"}
    if unknown:
        raise ValueError("不支持的command字段: " + str(sorted(unknown)))
    throttle = finite_number(command.get("throttle", 0), "throttle")
    brake = finite_number(command.get("brake", 0), "brake")
    if not 0 <= throttle <= 100 or not 0 <= brake <= 100:
        raise ValueError("throttle/brake必须在0～100之间")
    if interface == "pedal":
        if "deceleration" in command:
            raise ValueError("pedal模式不接受deceleration；请显式选择deceleration模式")
        acceleration, brake_sent = 0.0, brake
    elif interface == "deceleration":
        if "deceleration" in command:
            if brake != 0:
                raise ValueError("brake和deceleration不能同时指定非零制动")
            decel = finite_number(command["deceleration"], "deceleration")
            if decel < 0:
                raise ValueError("deceleration使用正值，单位m/s²")
            brake = decel / fullscale_decel * 100.0
        else:
            decel = brake / 100.0 * fullscale_decel
        if decel > max_decel:
            raise ValueError("减速度请求超过max_decel")
        # 此模式按本次T45B接口适配，严格小于-0.30才进入BRAKE。
        if 0 < decel <= 0.30:
            raise ValueError("T45B非零减速度必须大于0.30m/s²")
        acceleration, brake_sent = -decel, 0.0
    else:
        raise ValueError("未知brake_interface: " + str(interface))
    if throttle > 0 and brake > 0:
        raise ValueError("不能同时请求油门和制动")
    return ControlState(throttle=throttle, brake=brake,
                        acceleration=acceleration, brake_sent=brake_sent)


class SequenceAllocator:
    """先持久化预留号码，再使用；进程异常退出只会跳号，不重复使用。

    文件锁只保护号码分配，不允许多个采集进程同时控制同一车辆。
    """
    LIMIT = 1 << 32

    def __init__(self, state_path, block_size=65536):
        if type(block_size) is not int or block_size <= 0:
            raise ValueError("block_size必须为正整数")
        self.path = Path(state_path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.block_size = block_size
        self.next_sequence = 0
        self.end_sequence = 0
        # 第一次磁盘预留在构造时完成，尚未进入AUTO。
        self._reserve()

    def _reserve(self):
        # 状态文件会被原子替换，所以必须使用独立且不删除的锁文件。
        lock_path = self.path.with_suffix(self.path.suffix + ".lock")
        with lock_path.open("a+") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
            if self.path.exists():
                state = json.loads(self.path.read_text(encoding="utf-8"))
                if not isinstance(state, dict):
                    raise ValueError("序列号状态文件格式错误，禁止回退到0")
                start = state.get("next_sequence")
            else:
                start = 0
            if type(start) is not int or not 0 <= start <= self.LIMIT:
                raise ValueError("序列号状态无效，禁止回退到0")
            if start == self.LIMIT:
                raise RuntimeError("uint32序列号已耗尽，禁止回绕")
            end = min(start + self.block_size, self.LIMIT)
            fd, temp_path = tempfile.mkstemp(
                prefix=self.path.name + ".", dir=str(self.path.parent))
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as stream:
                    json.dump({"next_sequence": end}, stream)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temp_path, str(self.path))
                directory_fd = os.open(str(self.path.parent), os.O_RDONLY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            finally:
                if os.path.exists(temp_path):
                    os.unlink(temp_path)
            # 所有持久化操作成功后，才向本进程授予号码。
            self.next_sequence = start
            self.end_sequence = end

    def take(self):
        if self.next_sequence >= self.end_sequence:
            self._reserve()
        sequence = self.next_sequence
        self.next_sequence += 1
        return sequence


class AdvancedDataCollector:
    """Ensures data quality and automation by executing calibration plans"""

    def __init__(self,
                 node,
                 output_dir="./calibration_data_logs",
                 auto_start=False,
                 brake_interface="pedal",
                 brake_fullscale_decel=3.0,
                 max_decel=5.0,
                 safe_stop_brake=30.0,
                 safe_stop_deceleration=1.0):
        """Initialization"""
        self.node = node
        self.control_pub = node.create_writer('/apollo/control',
                                              control_cmd_pb2.ControlCommand)
        self.output_dir = output_dir
        self.auto_start = auto_start
        self.default_brake_interface = brake_interface
        self.brake_interface = brake_interface
        self.brake_fullscale_decel = finite_number(brake_fullscale_decel, "brake_fullscale_decel")
        self.max_decel = finite_number(max_decel, "max_decel")
        self.default_brake_fullscale_decel = self.brake_fullscale_decel
        self.default_max_decel = self.max_decel
        self.safe_stop_brake = finite_number(safe_stop_brake, "safe_stop_brake")
        self.safe_stop_deceleration = finite_number(safe_stop_deceleration, "safe_stop_deceleration")
        if self.brake_fullscale_decel <= 0 or self.max_decel <= 0:
            raise ValueError("减速度范围必须大于0")
        # 两种停车配置均提前验证，避免进入AUTO之后才发现错误。
        normalize_braking({"brake": self.safe_stop_brake}, "pedal",
                          self.brake_fullscale_decel, self.max_decel)
        if self.safe_stop_brake <= 0 or self.safe_stop_deceleration <= 0:
            raise ValueError("停车请求必须大于0")
        normalize_braking({"deceleration": self.safe_stop_deceleration}, "deceleration",
                          self.brake_fullscale_decel, self.max_decel)

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
        self.sequence_allocator = SequenceAllocator(
            Path(__file__).resolve().parents[2]
            / ".whl_dyn_runtime" / "control_sequence.json")
        self.abort_signal_received = False
        self.trigger_met_time = None  # Track when trigger condition was met
        # 生命周期相关状态；锁仅保护Cyber回调/文件，不创建发布线程。
        self._state_lock = threading.RLock()
        self._chassis_rx = None
        self._chassis_generation = 0
        self._readers = []
        self._auto_requested = False
        self.phase = "IDLE"
        self.last_error = ""
        self.feedback_timeout_sec = 0.5
        self.transition_timeout_sec = 5.0
        self.stop_timeout_sec = 30.0
        self.stopped_hold_sec = 0.5
        self.stopped_speed_mps = 0.1

        time.sleep(0.5)

    def setup_and_run(self, plan_path: str):
        """只建立订阅；每个用例自行进入和退出AUTO。"""
        if not self._load_plan(plan_path):
            return 1
        self._setup_subscriptions()
        return self.run_plan()

    def _load_plan(self, plan_path: str) -> bool:
        """Load and validate calibration plan from YAML file"""
        try:
            with open(plan_path, 'r') as f:
                self.plan = yaml.safe_load(f)
            if not isinstance(self.plan, list) or not self.plan:
                raise ValueError("计划必须是非空用例列表")
            # 每次加载都回退到启动默认值，避免上一计划配置泄漏。
            configs = set()
            for case in self.plan:
                interface = case.get("brake_interface", self.default_brake_interface)
                fullscale = finite_number(case.get("brake_fullscale_decel", self.default_brake_fullscale_decel), "brake_fullscale_decel")
                limit = finite_number(case.get("max_decel", self.default_max_decel), "max_decel")
                if fullscale <= 0 or limit <= 0:
                    raise ValueError("减速度配置必须大于0")
                configs.add((interface, fullscale, limit))
            if len(configs) != 1:
                raise ValueError("同一计划必须使用相同制动接口和减速度配置")
            selected_interface, fullscale, limit = configs.pop()
            # 校验真实收尾请求，不将停车减速度误当成固定30%的换算。
            if selected_interface == "deceleration":
                normalize_braking({"deceleration": self.safe_stop_deceleration},
                                  selected_interface, fullscale, limit)
            else:
                normalize_braking({"brake": self.safe_stop_brake},
                                  selected_interface, fullscale, limit)
            self.brake_fullscale_decel = fullscale
            self.max_decel = limit
            interfaces = set()
            for case in self.plan:
                interface = case.get("brake_interface", self.default_brake_interface)
                if interface not in ("pedal", "deceleration"):
                    raise ValueError("brake_interface必须为pedal或deceleration")
                interfaces.add(interface)
                if not case.get("steps"):
                    raise ValueError("用例必须包含非空steps")
                for step in case["steps"]:
                    normalize_braking(step["command"], interface,
                                      self.brake_fullscale_decel, self.max_decel)
            if len(interfaces) != 1:
                raise ValueError("一次计划必须使用同一种制动接口；不同底盘请分开运行")
            self.brake_interface = interfaces.pop()
            print("INFO: brake_interface=" + self.brake_interface)
            print(f"OK: Calibration plan loaded from '{plan_path}'")
            return True
        except (OSError, yaml.YAMLError, ValueError, TypeError, KeyError) as e:
            print(f"ERROR: Failed to load or parse plan file: {e}")
            return False

    def _setup_subscriptions(self):
        # 保留reader对象，避免订阅生命周期依赖临时返回值。
        self._readers = [
            self.node.create_reader('/apollo/localization/pose',
                                    localization_pb2.LocalizationEstimate,
                                    self._callback_localization),
            self.node.create_reader('/apollo/canbus/chassis', chassis_pb2.Chassis,
                                    self._callback_chassis),
        ]

    def _snapshot_chassis(self):
        with self._state_lock:
            state = replace(self.vehicle_state)
            received_at = self._chassis_rx
            generation = self._chassis_generation
        if received_at is None:
            raise RuntimeError("尚未收到有效底盘反馈")
        if time.monotonic() - received_at > self.feedback_timeout_sec:
            raise RuntimeError("底盘反馈接收超时")
        source_age = cyber_time.Time.now().to_sec() - state.timestamp
        if not math.isfinite(source_age) or abs(source_age) > self.feedback_timeout_sec:
            raise RuntimeError("底盘反馈消息头时间过期或异常")
        if state.error_code != chassis_pb2.Chassis.NO_ERROR:
            raise RuntimeError("底盘故障，error_code=" + str(state.error_code))
        return state, generation

    def _wait_initial_feedback(self):
        """尚未接管时只等反馈，不发送START，不覆盖其他控制来源。"""
        self.phase = "WAIT_FEEDBACK"
        deadline = time.monotonic() + self.transition_timeout_sec
        while cyber.ok() and not self.abort_signal_received:
            if self.chassis_received and self.localization_received:
                return self._snapshot_chassis()[0]
            if time.monotonic() >= deadline:
                raise RuntimeError("等待定位和底盘反馈超时")
            time.sleep(0.02)
        raise RuntimeError("等待反馈期间中止或Cyber关闭")

    def _wait_with_control(self, phase, condition, timeout_sec,
                           gear, parking_brake, action=None,
                           hold_sec=0.0, cleanup=False, require_auto=False):
        """主线程约50Hz发送停车请求，等待新的反馈确认目标状态。"""
        self.phase = phase
        print("PHASE: " + phase, flush=True)
        started = time.monotonic()
        _, initial_generation = self._snapshot_chassis()
        matched_since = None
        while cyber.ok():
            if self.abort_signal_received and not cleanup:
                raise RuntimeError("操作者中止")
            if time.monotonic() - started >= timeout_sec:
                raise RuntimeError(phase + "超时，未确认目标状态")
            state, _ = self._snapshot_chassis()
            if state.driving_mode == chassis_pb2.Chassis.EMERGENCY_MODE:
                raise RuntimeError("底盘已进入紧急模式，不自动RESET或重新接管")
            if require_auto and state.driving_mode != chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE:
                raise RuntimeError(phase + "期间退出AUTO")
            self._send_control_command(safe_stop=True, action=action,
                                       gear=gear, parking_brake=parking_brake)
            state, generation = self._snapshot_chassis()
            if generation > initial_generation and condition(state):
                if matched_since is None:
                    matched_since = time.monotonic()
                if time.monotonic() - matched_since >= hold_sec:
                    return
            else:
                matched_since = None
            time.sleep(0.02)
        raise RuntimeError("Cyber关闭，无法确认" + phase)

    def check_vehicle_ready(self, timeout_sec=10) -> bool:
        """每个用例调用：手动停稳→停车请求+START→确认AUTO→D/EPB释放。"""
        state = self._wait_initial_feedback()
        if state.driving_mode != chassis_pb2.Chassis.COMPLETE_MANUAL:
            raise RuntimeError("用例开始前必须为手动模式，禁止自动接管已有AUTO任务")
        if abs(state.speed_mps) > self.stopped_speed_mps:
            raise RuntimeError("用例开始前车辆未停稳")
        self._auto_requested = True
        self._wait_with_control(
            "ENTERING_AUTO",
            lambda s: s.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE,
            timeout_sec, state.actual_gear, state.parking_brake, action=1)
        self._wait_with_control(
            "PREPARING",
            lambda s: (s.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE
                       and s.actual_gear == chassis_pb2.Chassis.GEAR_DRIVE
                       and not s.parking_brake
                       and abs(s.speed_mps) <= self.stopped_speed_mps),
            self.transition_timeout_sec, chassis_pb2.Chassis.GEAR_DRIVE, False,
            hold_sec=self.stopped_hold_sec, require_auto=True)
        return True

    def _finish_case(self):
        """仅使用当前制动接口；停稳前不挂P，确认手动后不再发布。"""
        state, _ = self._snapshot_chassis()
        if state.driving_mode == chassis_pb2.Chassis.COMPLETE_MANUAL:
            # 只接受进入失败时仍然手动驻车的状态，不能宣称已完成采集。
            if (abs(state.speed_mps) > self.stopped_speed_mps
                    or state.actual_gear != chassis_pb2.Chassis.GEAR_PARKING
                    or not state.parking_brake):
                raise RuntimeError("已为手动但未确认停稳驻车，请人工接管")
        elif state.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE:
            self._wait_with_control(
                "STOPPING", lambda s: abs(s.speed_mps) <= self.stopped_speed_mps,
                self.stop_timeout_sec, state.actual_gear, state.parking_brake,
                hold_sec=self.stopped_hold_sec, cleanup=True, require_auto=True)
            self._wait_with_control(
                "PARKING",
                lambda s: (abs(s.speed_mps) <= self.stopped_speed_mps
                           and s.actual_gear == chassis_pb2.Chassis.GEAR_PARKING
                           and s.parking_brake),
                self.transition_timeout_sec, chassis_pb2.Chassis.GEAR_PARKING, True,
                hold_sec=self.stopped_hold_sec, cleanup=True, require_auto=True)
        else:
            raise RuntimeError("无法在当前模式完成自动收尾，请人工接管")
        self._wait_with_control(
            "EXITING",
            lambda s: (s.driving_mode == chassis_pb2.Chassis.COMPLETE_MANUAL
                       and abs(s.speed_mps) <= self.stopped_speed_mps
                       and s.actual_gear == chassis_pb2.Chassis.GEAR_PARKING
                       and s.parking_brake),
            self.transition_timeout_sec, chassis_pb2.Chassis.GEAR_PARKING, True,
            action=2, cleanup=True)
        self._auto_requested = False
        self.phase = "PARKED_MANUAL"
        print("PHASE: PARKED_MANUAL", flush=True)

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

            if not self._execute_case(case_config):
                print("ERROR: 用例失败或收尾失败，停止后续用例", flush=True)
                return 130 if self.abort_signal_received else 1

        print(f"\n{'='*80}\nINFO: Calibration plan execution completed.")
        return 130 if self.abort_signal_received else 0

    def _execute_case(self, case_config: dict):
        """原采集步骤保留，进入和收尾独立包围每一个用例。"""
        self.active_case = case_config
        self.active_step_idx = 0
        self.trigger_met_time = None
        self.last_error = ""
        self._auto_requested = False
        success = False
        if not self._prepare_output_file(case_config['case_name']):
            return False
        try:
            # 文件准备在AUTO之前完成，避免打开文件造成发布空窗。
            with open(self.output_file_path, 'w') as f:
                with self._state_lock:
                    self.output_file = f
                    self._write_header()
                try:
                    self.check_vehicle_ready()
                    self.phase = "COLLECTING"
                    self.is_collecting = True
                    # 保留原步骤超时/触发/保持算法，不在此次重写。
                    self.step_start_time = cyber_time.Time.now().to_sec()
                    while self.is_collecting and not self.abort_signal_received and cyber.ok():
                        loop_start_time = cyber_time.Time.now().to_sec()
                        state, _ = self._snapshot_chassis()
                        if (state.driving_mode != chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE
                                or state.actual_gear != chassis_pb2.Chassis.GEAR_DRIVE
                                or state.parking_brake):
                            raise RuntimeError("采集期间模式/档位/EPB状态异常")
                        self._state_machine_tick()
                        sleep_time = 0.01 - (cyber_time.Time.now().to_sec() - loop_start_time)
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                    if self.abort_signal_received:
                        raise RuntimeError("操作者中止")
                    if not cyber.ok():
                        raise RuntimeError("Cyber关闭")
                    success = True
                finally:
                    # 收尾不写入拟合CSV。先执行收尾，再关闭文件/刷新终端。
                    with self._state_lock:
                        self.is_collecting = False
                    if self._auto_requested:
                        self._finish_case()
        except Exception as exc:
            success = False
            self.last_error = str(exc)
            print("ERROR: " + self.last_error, flush=True)
            if self._auto_requested:
                print("ERROR: 未确认手动驻车，必须人工核实车辆状态", flush=True)
        finally:
            with self._state_lock:
                self.is_collecting = False
                self.output_file = None
        label = "COMPLETED" if success else "FAILED"
        print("RESULT: %s %s" % (label, self.output_file_path), flush=True)
        return success

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
            "time,speed_mps,ins_speed_mps,imu_accel_y,driving_mode,actual_gear,"
            "throttle_pct,brake_pct,ctl_throttle,ctl_brake,ctl_acceleration,ctl_brake_sent,brake_interface\n")

    def _print_live_status(self):
        """Print and refresh live status line in terminal"""
        if not self.active_case: return
        step = self.active_case['steps'][self.active_step_idx]
        cmd = step['command']
        actual = normalize_braking(cmd, self.brake_interface,
                                   self.brake_fullscale_decel, self.max_decel)
        trigger = step['trigger']
        elapsed = time.time() - self.step_start_time

        status_str = (
            f"\r>> Step {self.active_step_idx + 1}: "
            f"Speed: {self.vehicle_state.speed_mps:5.2f} m/s | "
            f"Trigger: {trigger['type'].replace('_', ' ')} {trigger['value']:.1f} | "
            f"Command: Throttle={actual.throttle:.1f}% BrakeSent={actual.brake_sent:.1f}% "
            f"Acceleration={actual.acceleration:.2f}m/s² [{self.brake_interface}] | "
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
            self.is_collecting = False
            raise RuntimeError("Step timed out. Aborting current case.")

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
                              safe_stop=False,
                              action=None,
                              gear=None,
                              parking_brake=None):
        """Construct and publish ControlCommand message"""
        cmd = control_cmd_pb2.ControlCommand()
        cmd.header.module_name = "advanced_collector1"
        cmd.header.sequence_num = self.sequence_allocator.take()
        cmd.header.timestamp_sec = cyber_time.Time.now().to_sec()

        if reset:
            if action is not None:
                raise ValueError("reset与action不能同时指定")
            action = 2
        if action is not None:
            cmd.pad_msg.action = action
        if default:
            command_dict = {"throttle": 0.0, "brake": 0.0}
        elif safe_stop:
            command_dict = ({"throttle": 0.0, "brake": self.safe_stop_brake}
                            if self.brake_interface == "pedal" else
                            {"throttle": 0.0, "deceleration": self.safe_stop_deceleration})
        if command_dict is not None:
            self.last_sent_control = normalize_braking(
                command_dict, self.brake_interface,
                self.brake_fullscale_decel, self.max_decel)
        if gear is not None:
            self.last_sent_control.gear = gear

        cmd.throttle = self.last_sent_control.throttle
        cmd.brake = self.last_sent_control.brake_sent
        cmd.gear_location = self.last_sent_control.gear
        if parking_brake is not None:
            cmd.parking_brake = parking_brake

        # 原踏板模式不设置acceleration，保持原版消息字段行为。
        if self.brake_interface == "deceleration":
            cmd.acceleration = self.last_sent_control.acceleration
        # TODO(leafyleong): re-enable after motion mode supported
        # cmd.motion_mode = self.last_sent_control.motion_mode

        result = self.control_pub.write(cmd)
        if result is False:
            raise RuntimeError("控制命令发布失败")

    def emergency_stop(self):
        """仅请求中止；主流程继续完成停车/驻车/退出，不在信号回调中睡眠。"""
        self.abort_signal_received = True

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
        """生命周期确认只使用字段齐全的底盘反馈。"""
        required = ('header', 'speed_mps', 'driving_mode', 'gear_location',
                    'parking_brake', 'error_code')
        if not all(data.HasField(name) for name in required):
            return
        if (not data.header.HasField('timestamp_sec')
                or not math.isfinite(data.header.timestamp_sec)
                or not math.isfinite(data.speed_mps)):
            return
        with self._state_lock:
            self.vehicle_state = VehicleState(
                timestamp=data.header.timestamp_sec,
                speed_mps=data.speed_mps,
                ins_speed_mps=self.vehicle_state.ins_speed_mps,
                imu_accel_y=self.vehicle_state.imu_accel_y,
                driving_mode=data.driving_mode,
                actual_gear=data.gear_location,
                throttle_pct=data.throttle_percentage,
                brake_pct=data.brake_percentage,
                parking_brake=data.parking_brake,
                error_code=data.error_code,
            )
            self._chassis_rx = time.monotonic()
            self._chassis_generation += 1
            self.chassis_received = True
            if self.is_collecting and self.output_file and not self.output_file.closed:
                self._write_log_entry()

    def _write_log_entry(self):
        """Write a complete, atomic snapshot of vehicle state to file"""
        vs = self.vehicle_state
        cs = self.last_sent_control
        self.output_file.write(
            f"{vs.timestamp:.4f},{vs.speed_mps:.4f},{vs.ins_speed_mps:.4f},{vs.imu_accel_y:.4f},"
            f"{vs.driving_mode},{vs.actual_gear},{vs.throttle_pct:.2f},"
            f"{vs.brake_pct:.2f},{cs.throttle:.2f},{cs.brake:.2f},"
            f"{cs.acceleration:.4f},{cs.brake_sent:.2f},{self.brake_interface}\n")


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
    parser.add_argument("--brake-interface", choices=("pedal", "deceleration"), default="pedal",
                        help="默认pedal保持原踏板接口；T45B选择deceleration；用例可显式覆盖")
    parser.add_argument("--brake-fullscale-decel", type=float, default=3.0,
                        help="deceleration模式旧brake百分比100对应的m/s²")
    parser.add_argument("--max-decel", type=float, default=3.0)
    parser.add_argument("--safe-stop-brake", type=float, default=30.0)
    parser.add_argument("--safe-stop-deceleration", type=float, default=0.9)
    args = parser.parse_args()

    cyber.init()
    node = cyber.Node("advanced_calibration_collector")
    collector = AdvancedDataCollector(node,
                                      output_dir=args.output_dir,
                                      auto_start=args.auto_start,
                                      brake_interface=args.brake_interface,
                                      brake_fullscale_decel=args.brake_fullscale_decel,
                                      max_decel=args.max_decel,
                                      safe_stop_brake=args.safe_stop_brake,
                                      safe_stop_deceleration=args.safe_stop_deceleration)

    # SIGINT/SIGTERM只置标志，主流程在finally中完成收尾尝试。
    def shutdown_handler(signum, frame):
        collector.emergency_stop()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    try:
        return collector.setup_and_run(args.plan)
    finally:
        cyber.shutdown()


if __name__ == '__main__':
    sys.exit(main())
