"""CyberRT collection for generic steering-to-vehicle dynamics tests.

Vehicle-specific protobuf names and field paths are supplied by YAML.  The
stored samples therefore remain portable across chassis implementations.
"""

import importlib
import math
import threading
import time
from collections import deque
from pathlib import Path

import yaml

from whl_dyn.collection.run_storage import RunStorage


DEFAULT_TOPICS = {
    "chassis": "/apollo/canbus/chassis",
    "chassis_detail": "/apollo/canbus/chassis_detail",
    "localization": "/apollo/localization/pose",
    "control": "/apollo/control",
}


def nested_value(message, path, default=None):
    """Read a dot-separated protobuf attribute path without vehicle knowledge."""

    current = message
    for part in str(path).split("."):
        if not part:
            continue
        if current is None or not hasattr(current, part):
            return default
        current = getattr(current, part)
    return current if current is not None else default


def message_time(message, fallback):
    """Prefer source time while retaining receive time for diagnostics."""

    measurement = getattr(message, "measurement_time", 0.0)
    if measurement:
        return float(measurement)
    header_time = nested_value(message, "header.timestamp_sec", 0.0)
    return float(header_time) if header_time else float(fallback)


def localization_signals(message, received_time):
    """Normalize localization output into the public collection schema."""

    pose = getattr(message, "pose", None)
    world_vx = nested_value(message, "pose.linear_velocity.x")
    world_vy = nested_value(message, "pose.linear_velocity.y")
    heading = nested_value(message, "pose.heading")
    yaw_rate = nested_value(message, "pose.angular_velocity_vrf.z")
    lateral_accel_vrf_right = nested_value(
        message, "pose.linear_acceleration_vrf.x")
    required = (world_vx, world_vy, heading, yaw_rate,
                lateral_accel_vrf_right)
    valid = all(value is not None and math.isfinite(float(value))
                for value in required)
    world_vx = float(world_vx) if world_vx is not None else float("nan")
    world_vy = float(world_vy) if world_vy is not None else float("nan")
    heading = float(heading) if heading is not None else float("nan")
    speed = math.hypot(
        world_vx, world_vy,
    )
    cos_heading = math.cos(heading)
    sin_heading = math.sin(heading)
    return {
        "localization_source_time_sec": message_time(message, received_time),
        "localization_signals_valid": valid,
        "speed_mps": speed,
        "longitudinal_velocity_mps": (
            world_vx * cos_heading + world_vy * sin_heading),
        "lateral_velocity_mps": (
            -world_vx * sin_heading + world_vy * cos_heading),
        "yaw_rate_radps": float(yaw_rate) if yaw_rate is not None else float("nan"),
        "lateral_accel_mps2": (
            -float(lateral_accel_vrf_right)
            if lateral_accel_vrf_right is not None else float("nan")),
        "roll_rad": float(nested_value(message, "pose.euler_angles.x", 0.0)),
    }


class LateralSignalCollector:
    """Collect synchronized snapshots and optionally inject steering profiles."""

    def __init__(self, node, signal_config):
        self.node = node
        self.config = signal_config
        self.topics = dict(DEFAULT_TOPICS, **signal_config.get("topics", {}))
        self._lock = threading.Lock()
        self._latest = {}
        self._detail_class = self._load_detail_class(signal_config.get("detail_message"))
        self._writer = None
        self._readers = []
        self._sequence_num = 0

    @staticmethod
    def _load_detail_class(detail_message):
        if not detail_message:
            return None
        module_name = detail_message.get("module")
        class_name = detail_message.get("class")
        if not module_name or not class_name:
            raise ValueError("detail_message requires module and class")
        module = importlib.import_module(str(module_name))
        return getattr(module, str(class_name))

    def subscribe(self):
        """Subscribe to generic Apollo sources after runtime imports are available."""

        try:
            from modules.common_msgs.chassis_msgs import chassis_detail_pb2, chassis_pb2
            from modules.common_msgs.localization_msgs import localization_pb2
        except ModuleNotFoundError:
            from wheelos_msgs.chassis_msgs import chassis_detail_pb2, chassis_pb2
            from wheelos_msgs.localization_msgs import localization_pb2

        self._readers = [
            self.node.create_reader(
                self.topics["chassis"], chassis_pb2.Chassis, self._on_chassis),
            self.node.create_reader(
                self.topics["chassis_detail"], chassis_detail_pb2.ChassisDetail,
                self._on_chassis_detail),
            self.node.create_reader(
                self.topics["localization"], localization_pb2.LocalizationEstimate,
                self._on_localization),
        ]

    def _put(self, values):
        values["received_time_sec"] = time.time()
        with self._lock:
            self._latest.update(values)

    def _on_chassis(self, message):
        now = time.time()
        values = {
            "chassis_source_time_sec": message_time(message, now),
            "chassis_speed_mps": float(getattr(message, "speed_mps", 0.0)),
            "driving_mode": int(getattr(message, "driving_mode", 0)),
        }
        for signal_name, field_path in self.config.get(
                "chassis_fields", {}).items():
            value = nested_value(message, field_path)
            if value is not None:
                normalized = float(value)
                if signal_name == "steering_feedback":
                    normalized *= float(
                        self.config.get("steering_feedback_scale", 1.0))
                values[str(signal_name)] = normalized
        self._put(values)

    def _on_chassis_detail(self, message):
        if not message.HasField("chassis_extension"):
            return
        detail = self._detail_from_extension(message.chassis_extension)
        if detail is None:
            return
        mapping = self.config.get("detail_fields", {})
        values = {
            "chassis_detail_source_time_sec": message_time(message, time.time()),
        }
        for signal_name, field_path in mapping.items():
            value = nested_value(detail, field_path)
            if value is not None:
                normalized = float(value)
                if signal_name == "steering_feedback":
                    normalized *= float(
                        self.config.get("steering_feedback_scale", 1.0))
                values[str(signal_name)] = normalized
        self._put(values)

    def _detail_from_extension(self, extension):
        """Resolve an Any payload from its type URL before using YAML fallback."""

        type_name = str(extension.type_url).rsplit("/", 1)[-1]
        detail_class = None
        if self._detail_class is not None:
            configured_name = self._detail_class.DESCRIPTOR.full_name
            if configured_name == type_name:
                detail_class = self._detail_class
        if detail_class is None:
            from google.protobuf import symbol_database

            for module_name in (
                    "modules.canbus.vehicle.zhongji_container.proto.zhongji_container_pb2",
                    "modules.canbus.vehicle.zhongji.proto.zhongji_pb2",
                    "modules.common_msgs.chassis_msgs.chassis_detail_pb2"):
                try:
                    importlib.import_module(module_name)
                except ModuleNotFoundError:
                    continue
            try:
                detail_class = symbol_database.Default().GetSymbol(type_name)
            except KeyError:
                return None
        detail = detail_class()
        return detail if extension.Unpack(detail) else None

    def _on_localization(self, message):
        self._put(localization_signals(message, time.time()))

    def _steering_feedback_source(self):
        if "steering_feedback" in self.config.get("detail_fields", {}):
            return "chassis_detail"
        if "steering_feedback" in self.config.get("chassis_fields", {}):
            return "chassis"
        return None

    def snapshot(self):
        """Return one fixed-time snapshot with alignment diagnostics.

        Values are the newest received values at the snapshot instant. Source
        timestamps and the maximum source-time spread are persisted so offline
        processing can reject rows that exceed the configured skew.
        """

        now_wall = time.time()
        now_mono = time.monotonic()
        with self._lock:
            sample = dict(self._latest)
        sample["collector_time_sec"] = now_wall
        sample["sample_time_sec"] = now_wall
        sample["collector_monotonic_sec"] = now_mono
        source_times = []
        for source in ("localization", "chassis", "chassis_detail"):
            source_time = sample.get("{0}_source_time_sec".format(source))
            if source_time:
                source_times.append(float(source_time))
            sample["{0}_age_sec".format(source)] = (
                now_wall - float(source_time) if source_time else float("nan"))
        if source_times:
            sample["source_time_min_sec"] = min(source_times)
            sample["source_time_max_sec"] = max(source_times)
            sample["alignment_skew_sec"] = max(source_times) - min(source_times)
            required_sources = ["localization_source_time_sec",
                                "chassis_source_time_sec"]
            feedback_source = self._steering_feedback_source()
            if "steering_feedback" in sample and feedback_source:
                required_sources.append(
                    "{}_source_time_sec".format(feedback_source))
            sample["time_aligned"] = (
                all(name in sample for name in required_sources) and
                sample["alignment_skew_sec"] <= float(
                    self.config.get("max_alignment_skew_sec", 0.05)))
        else:
            sample["source_time_min_sec"] = float("nan")
            sample["source_time_max_sec"] = float("nan")
            sample["alignment_skew_sec"] = float("nan")
            sample["time_aligned"] = False
        return sample

    def wait_for_sources(self, timeout_sec, require_steering_feedback=False):
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            source_ready = ("localization_source_time_sec" in snapshot and
                            "chassis_source_time_sec" in snapshot)
            feedback_source = self._steering_feedback_source() or "chassis"
            feedback_ready = (
                "steering_feedback" in snapshot and
                snapshot.get("{}_age_sec".format(feedback_source),
                             float("inf")) <= float(
                                 self.config.get("max_feedback_age_sec", 0.5)))
            if source_ready and (not require_steering_feedback or feedback_ready):
                return True
            time.sleep(0.05)
        return False

    def _speed_in_gate(self, gate):
        sample = self.snapshot()
        speed = sample.get("chassis_speed_mps", sample.get("speed_mps"))
        if speed is None:
            return False
        return (float(gate.get("min_mps", 0.0)) <= float(speed) <=
                float(gate.get("max_mps", float("inf"))))

    def _speed_at_target(self, gate):
        sample = self.snapshot()
        speed = sample.get("chassis_speed_mps", sample.get("speed_mps"))
        if speed is None:
            return False
        return abs(float(speed) - float(gate["target_mps"])) <= float(
            gate.get("tolerance_mps", 0.0))

    def wait_for_speed_stable(self, gate, longitudinal, timeout_sec,
                              steering_command=0.0):
        """Hold longitudinal input until speed is stable for the configured time."""

        required = ("min_mps", "max_mps", "target_mps")
        if any(name not in gate for name in required):
            raise ValueError("active lateral tests require a complete speed gate")
        stable_duration = float(gate.get("stable_duration_sec", 0.0))
        stability_tolerance = float(gate.get(
            "stability_tolerance_mps", gate.get("tolerance_mps", 0.15)))
        min_in_band_fraction = float(gate.get(
            "stability_min_in_band_fraction", 0.8))
        if stable_duration <= 0.0 or stability_tolerance < 0.0:
            raise ValueError("speed stability settings must be non-negative")
        if not 0.0 <= min_in_band_fraction <= 1.0:
            raise ValueError("speed stability in-band fraction must be between 0 and 1")
        deadline = time.monotonic() + float(timeout_sec)
        stable_samples = deque()
        while time.monotonic() < deadline:
            self.publish_control(
                float(steering_command),
                longitudinal=longitudinal,
            )
            now = time.monotonic()
            snapshot = self.snapshot()
            speed = snapshot.get("chassis_speed_mps", snapshot.get("speed_mps"))
            in_gate = (
                speed is not None and
                float(gate["min_mps"]) <= float(speed) <= float(gate["max_mps"]))
            if in_gate:
                stable_samples.append((now, float(speed)))
                speeds = [value for _, value in stable_samples]
                if (stable_samples and
                        now - stable_samples[0][0] >= stable_duration):
                    in_band_count = sum(
                        abs(value - float(gate["target_mps"])) <=
                        stability_tolerance for value in speeds)
                    if in_band_count / len(speeds) >= min_in_band_fraction:
                        return float(sum(speeds) / len(speeds))
                while (stable_samples and
                       len(stable_samples) > 1 and
                       now - stable_samples[0][0] > stable_duration):
                    stable_samples.popleft()
            else:
                stable_samples.clear()
            time.sleep(0.05)
        return None

    def prepare_steering_before_speed(self, profile, sampling_rate,
                                      maximum, maximum_rate):
        """Reach a fixed steering target before enabling longitudinal motion."""

        target = float(profile.get("target", 0.0))
        ramp_end = float(profile.get("ramp_end_sec", 0.0))
        started = time.monotonic()
        previous = 0.0
        while True:
            elapsed = time.monotonic() - started
            if elapsed >= ramp_end:
                break
            command = target * elapsed / ramp_end if ramp_end else target
            if abs(command) > maximum:
                raise RuntimeError("steering profile exceeds configured limit")
            command_rate = abs(command - previous) * sampling_rate
            if command_rate > maximum_rate + 1e-9:
                raise RuntimeError("steering profile exceeds configured rate")
            self.publish_control(command, 0.0)
            previous = command
            time.sleep(1.0 / sampling_rate)
        self.publish_control(target, 0.0)

    def _control_writer(self):
        if self._writer is None:
            try:
                from modules.common_msgs.control_msgs import control_cmd_pb2
            except ModuleNotFoundError:
                from wheelos_msgs.control_msgs import control_cmd_pb2

            self._writer = self.node.create_writer(
                self.topics["control"], control_cmd_pb2.ControlCommand)
        return self._writer

    def publish_control(self, steering_command, speed_target_mps=0.0,
                        longitudinal=None):
        """Publish coupled longitudinal hold and lateral excitation commands."""

        try:
            from modules.common_msgs.control_msgs import control_cmd_pb2
        except ModuleNotFoundError:
            from wheelos_msgs.control_msgs import control_cmd_pb2

        command_scale = float(self.config.get("control_steering_scale", 1.0))
        message = control_cmd_pb2.ControlCommand()
        self._sequence_num += 1
        message.header.timestamp_sec = time.time()
        message.header.sequence_num = self._sequence_num
        message.header.module_name = "whl_dyn_lateral"
        control = longitudinal or {
            "mode": "speed",
            "speed_mps": speed_target_mps,
            "throttle": 0.0,
            "brake": 0.0,
        }
        control.setdefault(
            "gear_location", int(self.config.get("control_gear_location", 3)))
        mode = str(control.get("mode", "speed")).lower()
        if mode not in ("speed", "throttle"):
            raise ValueError("longitudinal control mode must be speed or throttle")
        message.speed = float(control.get("speed_mps", 0.0)) if mode == "speed" else 0.0
        message.throttle = float(control.get("throttle", 0.0)) if mode == "throttle" else 0.0
        message.brake = float(control.get("brake", 0.0))
        message.gear_location = int(control.get(
            "gear_location", self.config.get("control_gear_location", 3)))
        message.steering_target = float(steering_command) * command_scale
        message.pad_msg.driving_mode = 1
        message.pad_msg.action = 1
        self._control_writer().write(message)

    def collect_case(self, case, output_root, execute=False, arm=False,
                     source_timeout_sec=10.0):
        """Collect one plan case into a new directory.

        ``execute`` is deliberately opt-in.  Record-only operation captures
        externally generated steering profiles with the exact same schema.
        """

        if execute and not arm:
            raise ValueError("steering execution requires explicit arm=True")
        if execute:
            from whl_dyn.planning.preflight import validate_active_signal_config

            validate_active_signal_config(self.config)

        profile = case.get("command_profile", {})
        sampling_rate = float(case.get("sampling_rate_hz", 100.0))
        duration = float(case.get("duration_sec", profile.get("duration_sec", 0.0)))
        if sampling_rate <= 0.0 or duration <= 0.0:
            raise ValueError("case duration and sampling rate must be positive")
        safety = case.get("safety_limits", {})
        maximum = abs(float(safety.get("max_abs_steering", float("inf"))))
        maximum_feedback = abs(float(
            safety.get("max_abs_feedback_steering", maximum)))
        maximum_rate = abs(float(safety.get("max_steering_rate", float("inf"))))

        storage = RunStorage(output_root, case.get("case_name", "lateral"), {
            "case": case,
            "collection_mode": "execute" if execute else "record_only",
            "signal_config": self.config,
            "created_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        })
        samples = []
        abort_reason = None
        stage = "waiting_for_sources"
        steering_before_speed = bool(case.get("steering_before_speed", False))
        steering_target = float(profile.get("target", 0.0))
        longitudinal = dict(case.get("longitudinal_control", {
            "mode": "speed",
            "speed_mps": case.get("speed_gate", {}).get("target_mps", 0.0),
            "throttle": 0.0,
            "brake": 0.0,
        }))
        stable_speed_mean = None
        turn_count_reached = False
        turn_count = float(case.get("turn_count", 0.0)) if execute else 0.0
        try:
            if not self.wait_for_sources(source_timeout_sec, execute):
                raise RuntimeError(
                    "timed out waiting for chassis, localization and steering feedback")
            speed_gate = case.get("speed_gate", {})
            if execute:
                stage = "waiting_for_stable_speed"
                if steering_before_speed:
                    stage = "establishing_steering"
                    self.prepare_steering_before_speed(
                        profile, sampling_rate, maximum, maximum_rate)
                stable_speed_mean = self.wait_for_speed_stable(
                    speed_gate,
                    longitudinal,
                    float(speed_gate.get("max_wait_sec", source_timeout_sec)),
                    steering_command=(
                        steering_target if steering_before_speed else 0.0),
                )
                if stable_speed_mean is None:
                    raise RuntimeError("timed out waiting for stable speed")
                storage.write_metadata({
                    "case": case,
                    "collection_mode": "execute" if execute else "record_only",
                    "signal_config": self.config,
                    "created_at_utc": time.strftime(
                        "%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "stable_speed_mean_mps": stable_speed_mean,
                    "stable_duration_sec": speed_gate.get("stable_duration_sec"),
                })

            stage = "collecting"
            started = time.monotonic()
            next_sample = started
            previous_command = 0.0
            target_turn_angle = 2.0 * math.pi * turn_count
            accumulated_turn_angle = 0.0
            turn_count_reached = False
            previous_sample_time = None
            while True:
                now = time.monotonic()
                elapsed = now - started
                if elapsed >= duration:
                    break
                if now < next_sample:
                    time.sleep(min(next_sample - now, 0.005))
                    continue
                if execute:
                    from whl_dyn.collection.collector import evaluate_command_profile

                    command = (
                        steering_target if steering_before_speed else
                        float(evaluate_command_profile(profile, elapsed))
                    )
                    if abs(command) > maximum:
                        raise RuntimeError("steering profile exceeds configured limit")
                    if (not case.get("allow_command_step", False) and
                            samples and
                            abs(command - previous_command) / (1.0 / sampling_rate) >
                            maximum_rate):
                        raise RuntimeError("steering profile exceeds configured rate")
                    if not self._speed_in_gate(speed_gate):
                        raise RuntimeError("vehicle left configured speed range")
                    feedback_snapshot = self.snapshot()
                    if not feedback_snapshot.get("time_aligned", False):
                        raise RuntimeError(
                            "source timestamps exceeded alignment skew")
                    if not feedback_snapshot.get("localization_signals_valid", False):
                        raise RuntimeError(
                            "localization dynamics signals are unavailable")
                    for source in ("localization", "chassis"):
                        age = feedback_snapshot.get(
                            "{}_age_sec".format(source), float("inf"))
                        if not math.isfinite(float(age)) or float(age) > float(
                                self.config.get("max_source_age_sec", 0.5)):
                            raise RuntimeError(
                                "{} source became stale".format(source))
                    feedback = feedback_snapshot.get("steering_feedback")
                    feedback_source = self._steering_feedback_source() or "chassis"
                    feedback_age = feedback_snapshot.get(
                        "{}_age_sec".format(feedback_source), float("inf"))
                    if float(feedback_age) > float(
                            self.config.get("max_feedback_age_sec", 0.5)):
                        raise RuntimeError("steering feedback became stale")
                    if (feedback is None or not math.isfinite(float(feedback)) or
                            abs(float(feedback)) > maximum_feedback):
                        raise RuntimeError(
                            "actual steering feedback exceeds configured limit")
                    max_lateral_accel = safety.get("max_abs_lateral_accel_mps2")
                    lateral_accel = feedback_snapshot.get("lateral_accel_mps2")
                    if (lateral_accel is None or
                            not math.isfinite(float(lateral_accel))):
                        raise RuntimeError(
                            "lateral acceleration became unavailable")
                    if (max_lateral_accel is not None and
                            abs(float(lateral_accel)) >
                            float(max_lateral_accel)):
                        raise RuntimeError(
                            "vehicle exceeded lateral acceleration limit")
                    self.publish_control(command, longitudinal=longitudinal)
                    previous_command = command
                sample = self.snapshot()
                sample["elapsed_sec"] = elapsed
                sample["sample_index"] = len(samples)
                sample["steering_command"] = previous_command if execute else float("nan")
                sample["case_phase"] = (
                    "steady" if steering_before_speed else
                    ("baseline" if profile.get("type") == "step" and
                     elapsed < float(profile.get("start_time_sec", 0.0))
                     else ("step_response" if profile.get("type") == "step"
                           else _case_phase(profile, elapsed))))
                if execute and turn_count > 0.0:
                    sample_time = float(sample["collector_monotonic_sec"])
                    if previous_sample_time is not None:
                        yaw_rate = sample.get("yaw_rate_radps")
                        if yaw_rate is None:
                            raise RuntimeError(
                                "turn-count collection requires yaw_rate_radps")
                        accumulated_turn_angle += abs(float(yaw_rate)) * max(
                            0.0, sample_time - previous_sample_time)
                    previous_sample_time = sample_time
                    sample["accumulated_turn_angle_rad"] = accumulated_turn_angle
                samples.append(sample)
                next_sample += 1.0 / sampling_rate
                if execute and turn_count > 0.0 and (
                        accumulated_turn_angle >= target_turn_angle):
                    turn_count_reached = True
                    break
        except BaseException as error:
            abort_reason = str(error) or (
                "interrupted by user" if isinstance(error, KeyboardInterrupt)
                else error.__class__.__name__)
            raise
        finally:
            if execute:
                self.publish_control(
                    0.0,
                    longitudinal={
                        "mode": "speed",
                        "speed_mps": 0.0,
                        "throttle": 0.0,
                        "brake": float(self.config.get("stop_brake", 30.0)),
                        "gear_location": int(self.config.get(
                            "control_gear_location", 3)),
                    },
                )
            if samples:
                storage.write_samples(samples)
            storage.write_status({
                "completed": abort_reason is None,
                "stage": "completed" if abort_reason is None else stage,
                "abort_reason": abort_reason,
                "sample_count": len(samples),
                "turn_count_reached": (
                    turn_count_reached if execute and turn_count > 0.0
                    else None),
                "stable_speed_mean_mps": stable_speed_mean,
            })
        return storage.path


def load_signal_config(path):
    """Load a reusable vehicle mapping without embedding it in Python code."""

    with Path(path).open() as config_file:
        config = yaml.safe_load(config_file) or {}
    if not isinstance(config, dict):
        raise ValueError("signal configuration must be a mapping")
    return config


def _case_phase(profile, elapsed_sec):
    """Label transient and steady portions without changing test behavior."""

    profile_type = str(profile.get("type", "")).lower()
    if profile_type == "ramp":
        ramp_end = float(profile.get("ramp_end_sec", 0.0))
        return "ramp" if elapsed_sec < ramp_end else "steady"
    if profile_type == "step":
        return "step_hold"
    return "excitation"
