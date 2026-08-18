"""Direct planning-trajectory execution for closed-loop handling tests."""

import math
import time

from whl_dyn.collection.run_storage import RunStorage
from whl_dyn.trajectory.apollo import ContinuousTrajectoryPublisher
from whl_dyn.trajectory.continuous import CirclePath, ClothoidCirclePath


def build_path_from_case(trajectory, x0, y0, theta0, duration_sec):
    """Build the immutable geometry once, before publishing the first frame."""

    kind = trajectory.get("type")
    if kind == "circle":
        return CirclePath(x0, y0, theta0, trajectory["curvature_1pm"])
    if kind == "clothoid_circle":
        path = ClothoidCirclePath(
            x0, y0, theta0, trajectory["radius_m"],
            trajectory["entry_length_m"], trajectory["arc_angle_rad"],
            trajectory.get("exit_length_m", 0.0), trajectory.get("direction", 1.0),
            trajectory.get("straight_entry_length_m", 0.0),
            trajectory.get("straight_exit_length_m", 0.0))
        required_length = float(trajectory["speed_mps"]) * float(duration_sec)
        if required_length > path.length_m:
            raise ValueError(
                "clothoid path is shorter than test duration plus horizon")
        return path
    raise ValueError("unsupported closed-loop trajectory type: {0}".format(kind))


class ClosedLoopTrajectoryRunner:
    """Publish continuous planning windows and record controller responses."""

    def __init__(self, node, planning_topic="/apollo/planning",
                 control_topic="/apollo/control"):
        self.node = node
        self.planning_topic = planning_topic
        self.control_topic = control_topic
        self._latest_localization = None
        self._samples = []

    def subscribe(self):
        from modules.common_msgs.chassis_msgs import chassis_pb2
        from modules.common_msgs.control_msgs import control_cmd_pb2
        from modules.common_msgs.localization_msgs import localization_pb2

        self.node.create_reader(
            "/apollo/localization/pose", localization_pb2.LocalizationEstimate,
            self._on_localization)
        self.node.create_reader(
            "/apollo/canbus/chassis", chassis_pb2.Chassis, self._on_chassis)
        self.node.create_reader(
            self.control_topic, control_cmd_pb2.ControlCommand, self._on_control)

    def _on_localization(self, message):
        self._latest_localization = message
        pose = message.pose
        self._samples.append({
            "source": "localization",
            "source_time_sec": float(
                message.measurement_time or message.header.timestamp_sec),
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "heading_rad": float(pose.heading),
            "yaw_rate_radps": float(pose.angular_velocity_vrf.z),
            "lateral_accel_mps2": float(pose.linear_acceleration_vrf.x),
        })

    def _on_chassis(self, message):
        self._samples.append({
            "source": "chassis",
            "source_time_sec": float(message.header.timestamp_sec),
            "speed_mps": float(message.speed_mps),
            "driving_mode": int(message.driving_mode),
            "steering_feedback": float(message.steering_percentage),
        })

    def _on_control(self, message):
        debug = getattr(getattr(message, "debug", None), "simple_mpc_debug", None)
        self._samples.append({
            "source": "control",
            "source_time_sec": float(message.header.timestamp_sec),
            "steering_command": float(message.steering_target),
            "control_speed_target_mps": float(message.speed),
            "lateral_error_m": float(getattr(debug, "lateral_error", float("nan"))),
            "heading_error_rad": float(getattr(debug, "heading_error", float("nan"))),
            "reference_kappa_1pm": float(getattr(debug, "curvature", float("nan"))),
            "steer_feedforward": float(
                getattr(debug, "steer_angle_feedforward", float("nan"))),
            "steer_feedback_control": float(
                getattr(debug, "steer_angle_feedback", float("nan"))),
        })

    def _wait_for_localization(self, timeout_sec):
        deadline = time.monotonic() + float(timeout_sec)
        while self._latest_localization is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._latest_localization is None:
            raise RuntimeError("timed out waiting for localization")
        pose = self._latest_localization.pose
        return float(pose.position.x), float(pose.position.y), float(pose.heading)

    def run_case(self, case, output_root, localization_timeout_sec=10.0):
        """Run one direct-planning case and persist raw source rows."""

        trajectory = case.get("trajectory", {})
        duration = float(case["duration_sec"])
        x0, y0, theta0 = self._wait_for_localization(localization_timeout_sec)
        path = build_path_from_case(trajectory, x0, y0, theta0, duration)
        storage = RunStorage(output_root, case["case_name"], {
            "case": case,
            "trajectory_anchor": {"x": x0, "y": y0, "theta": theta0},
            "publisher_contract": (
                "Overlapping windows are sampled from one immutable path; "
                "geometry at equal experiment time is frame-invariant."),
        })
        publisher = ContinuousTrajectoryPublisher(self.node, self.planning_topic)
        publish_period = 1.0 / float(trajectory.get("publish_rate_hz", 20.0))
        start = time.monotonic()
        next_tick = start
        abort_reason = None
        try:
            while time.monotonic() - start < duration:
                elapsed = time.monotonic() - start
                publisher.publish(
                    path, elapsed, trajectory["speed_mps"],
                    trajectory.get("horizon_sec", 8.0),
                    planning_cycle_time_sec=publish_period)
                next_tick += publish_period
                time.sleep(max(0.0, next_tick - time.monotonic()))
        except BaseException as error:
            abort_reason = str(error)
            raise
        finally:
            self._send_safe_stop()
            if self._samples:
                storage.write_samples(self._samples)
            storage.write_status({
                "completed": abort_reason is None,
                "abort_reason": abort_reason,
                "sample_count": len(self._samples),
            })
        return storage.path

    def _send_safe_stop(self):
        """Publish a bounded direct stop after a direct-planning experiment."""

        from modules.common_msgs.control_msgs import control_cmd_pb2

        writer = self.node.create_writer(self.control_topic,
                                         control_cmd_pb2.ControlCommand)
        for _ in range(20):
            command = control_cmd_pb2.ControlCommand()
            command.header.timestamp_sec = time.time()
            command.header.module_name = "whl_dyn_closed_loop_stop"
            command.speed = 0.0
            command.steering_target = 0.0
            command.brake = 30.0
            writer.write(command)
            time.sleep(0.05)
