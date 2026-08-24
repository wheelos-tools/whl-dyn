"""CyberRT collection for generic steering-to-vehicle dynamics tests.

Vehicle-specific protobuf names and field paths are supplied by YAML.  The
stored samples therefore remain portable across chassis implementations.
"""

import importlib
import math
import threading
import time
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
    velocity = getattr(pose, "linear_velocity", None)
    speed = math.hypot(
        float(getattr(velocity, "x", 0.0)),
        float(getattr(velocity, "y", 0.0)),
    )
    return {
        "localization_source_time_sec": message_time(message, received_time),
        "speed_mps": speed,
        "yaw_rate_radps": float(nested_value(
            message, "pose.angular_velocity_vrf.z", 0.0)),
        "lateral_accel_mps2": float(nested_value(
            message, "pose.linear_acceleration_vrf.x", 0.0)),
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

        from modules.common_msgs.chassis_msgs import chassis_detail_pb2, chassis_pb2
        from modules.common_msgs.localization_msgs import localization_pb2

        self.node.create_reader(
            self.topics["chassis"], chassis_pb2.Chassis, self._on_chassis)
        self.node.create_reader(
            self.topics["chassis_detail"], chassis_detail_pb2.ChassisDetail,
            self._on_chassis_detail)
        self.node.create_reader(
            self.topics["localization"], localization_pb2.LocalizationEstimate,
            self._on_localization)

    def _put(self, values):
        values["received_time_sec"] = time.time()
        with self._lock:
            self._latest.update(values)

    def _on_chassis(self, message):
        now = time.time()
        self._put({
            "chassis_source_time_sec": message_time(message, now),
            "chassis_speed_mps": float(getattr(message, "speed_mps", 0.0)),
            "driving_mode": int(getattr(message, "driving_mode", 0)),
        })

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
            if "steering_feedback" in sample:
                required_sources.append("chassis_detail_source_time_sec")
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
            feedback_ready = ("steering_feedback" in snapshot and
                              snapshot.get("chassis_detail_age_sec",
                                           float("inf")) <= float(
                                               self.config.get(
                                                   "max_feedback_age_sec", 0.5)))
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

    def wait_for_speed_target(self, gate, timeout_sec):
        """Hold a commanded speed inside its tolerance before lateral excitation."""

        required = ("min_mps", "max_mps", "target_mps")
        if any(name not in gate for name in required):
            raise ValueError("active lateral tests require a complete speed gate")
        stable_duration = float(gate.get("stable_duration_sec", 0.0))
        deadline = time.monotonic() + float(timeout_sec)
        stable_since = None
        while time.monotonic() < deadline:
            self.publish_control(0.0, float(gate["target_mps"]))
            if self._speed_in_gate(gate) and self._speed_at_target(gate):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= stable_duration:
                    return True
            else:
                stable_since = None
            time.sleep(0.05)
        return False

    def _control_writer(self):
        if self._writer is None:
            from modules.common_msgs.control_msgs import control_cmd_pb2

            self._writer = self.node.create_writer(
                self.topics["control"], control_cmd_pb2.ControlCommand)
        return self._writer

    def publish_control(self, steering_command, speed_target_mps):
        """Publish coupled longitudinal hold and lateral excitation commands."""

        from modules.common_msgs.control_msgs import control_cmd_pb2

        command_scale = float(self.config.get("control_steering_scale", 1.0))
        message = control_cmd_pb2.ControlCommand()
        self._sequence_num += 1
        message.header.timestamp_sec = time.time()
        message.header.sequence_num = self._sequence_num
        message.header.module_name = "whl_dyn_lateral"
        message.speed = float(speed_target_mps)
        message.throttle = 0.0
        message.brake = 0.0
        message.steering_target = float(steering_command) * command_scale
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
        try:
            if not self.wait_for_sources(source_timeout_sec, execute):
                raise RuntimeError(
                    "timed out waiting for chassis, localization and steering feedback")
            speed_gate = case.get("speed_gate", {})
            if execute:
                stage = "waiting_for_target_speed"
                if not self.wait_for_speed_target(
                        speed_gate,
                        float(speed_gate.get("max_wait_sec", source_timeout_sec))):
                    raise RuntimeError("timed out waiting for target speed")

            stage = "collecting"
            started = time.monotonic()
            next_sample = started
            previous_command = 0.0
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

                    command = float(evaluate_command_profile(profile, elapsed))
                    if abs(command) > maximum:
                        raise RuntimeError("steering profile exceeds configured limit")
                    if (not case.get("allow_command_step", False) and elapsed > 0.0 and
                            abs(command - previous_command) / (1.0 / sampling_rate) >
                            maximum_rate):
                        raise RuntimeError("steering profile exceeds configured rate")
                    if not self._speed_in_gate(speed_gate):
                        raise RuntimeError("vehicle left configured speed range")
                    feedback_snapshot = self.snapshot()
                    feedback = feedback_snapshot.get("steering_feedback")
                    feedback_age = feedback_snapshot.get(
                        "chassis_detail_age_sec", float("inf"))
                    if float(feedback_age) > float(
                            self.config.get("max_feedback_age_sec", 0.5)):
                        raise RuntimeError("steering feedback became stale")
                    if (feedback is None or abs(float(feedback)) >
                            maximum_feedback):
                        raise RuntimeError(
                            "actual steering feedback exceeds configured limit")
                    max_lateral_accel = safety.get("max_abs_lateral_accel_mps2")
                    lateral_accel = self.snapshot().get("lateral_accel_mps2")
                    if (max_lateral_accel is not None and lateral_accel is not None
                            and abs(float(lateral_accel)) >
                            float(max_lateral_accel)):
                        raise RuntimeError(
                            "vehicle exceeded lateral acceleration limit")
                    self.publish_control(command, float(speed_gate["target_mps"]))
                    previous_command = command
                sample = self.snapshot()
                sample["elapsed_sec"] = elapsed
                sample["sample_index"] = len(samples)
                sample["steering_command"] = previous_command if execute else float("nan")
                sample["case_phase"] = _case_phase(profile, elapsed)
                samples.append(sample)
                next_sample += 1.0 / sampling_rate
        except BaseException as error:
            abort_reason = str(error) or (
                "interrupted by user" if isinstance(error, KeyboardInterrupt)
                else error.__class__.__name__)
            raise
        finally:
            if execute:
                self.publish_control(0.0, 0.0)
            if samples:
                storage.write_samples(samples)
            storage.write_status({
                "completed": abort_reason is None,
                "stage": "completed" if abort_reason is None else stage,
                "abort_reason": abort_reason,
                "sample_count": len(samples),
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
