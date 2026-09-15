import sys
import types

import numpy as np

from whl_dyn.collection.closed_loop import ClosedLoopTrajectoryRunner
from whl_dyn.trajectory.apollo import ContinuousTrajectoryPublisher
from whl_dyn.trajectory.continuous import CirclePath


class _FakeWriter:
    def __init__(self):
        self.messages = []

    def write(self, message):
        self.messages.append(message)


class _FakeNode:
    def __init__(self):
        self.readers = []
        self.writers = []

    def create_reader(self, topic, message_type, callback):
        self.readers.append((topic, message_type, callback))

    def create_writer(self, topic, message_type):
        writer = _FakeWriter()
        self.writers.append((topic, message_type, writer))
        return writer


class _RepeatedPoints(list):
    def add(self):
        point = types.SimpleNamespace(path_point=types.SimpleNamespace())
        self.append(point)
        return point


class _ADCTrajectory:
    NORMAL = 1

    def __init__(self):
        self.header = types.SimpleNamespace()
        self.trajectory_point = _RepeatedPoints()


class _ControlCommand:
    def __init__(self):
        self.header = types.SimpleNamespace()
        self.pad_msg = types.SimpleNamespace()


class _DebugMessage:
    def __init__(self, field=None):
        self.simple_lat_debug = types.SimpleNamespace(
            lateral_error=0.2, heading_error=0.1, curvature=0.02)
        self.simple_mpc_debug = types.SimpleNamespace(
            lateral_error=9.0, heading_error=9.0, curvature=9.0)
        self._field = field

    def HasField(self, name):
        return name == self._field


class _ControlWithDebug:
    def __init__(self, field):
        self.header = types.SimpleNamespace(timestamp_sec=1.0)
        self.steering_target = 2.0
        self.speed = 2.0
        self.debug = _DebugMessage(field)


def _install_generated_messages(monkeypatch):
    modules = {
        "wheelos_msgs": types.ModuleType("wheelos_msgs"),
        "wheelos_msgs.chassis_msgs": types.ModuleType(
            "wheelos_msgs.chassis_msgs"),
        "wheelos_msgs.control_msgs": types.ModuleType(
            "wheelos_msgs.control_msgs"),
        "wheelos_msgs.localization_msgs": types.ModuleType(
            "wheelos_msgs.localization_msgs"),
        "wheelos_msgs.planning_msgs": types.ModuleType(
            "wheelos_msgs.planning_msgs"),
        "wheelos_msgs.chassis_msgs.chassis_pb2": types.ModuleType(
            "wheelos_msgs.chassis_msgs.chassis_pb2"),
        "wheelos_msgs.control_msgs.control_cmd_pb2": types.ModuleType(
            "wheelos_msgs.control_msgs.control_cmd_pb2"),
        "wheelos_msgs.localization_msgs.localization_pb2": types.ModuleType(
            "wheelos_msgs.localization_msgs.localization_pb2"),
        "wheelos_msgs.planning_msgs.planning_pb2": types.ModuleType(
            "wheelos_msgs.planning_msgs.planning_pb2"),
    }
    modules["wheelos_msgs.chassis_msgs.chassis_pb2"].Chassis = type(
        "Chassis", (), {"GEAR_DRIVE": 3})
    modules["wheelos_msgs.control_msgs.control_cmd_pb2"].ControlCommand = (
        _ControlCommand)
    modules["wheelos_msgs.localization_msgs.localization_pb2"].LocalizationEstimate = (
        type("LocalizationEstimate", (), {}))
    modules["wheelos_msgs.planning_msgs.planning_pb2"].ADCTrajectory = (
        _ADCTrajectory)
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)


def test_closed_loop_runtime_uses_wheelos_messages_and_drive_gear(monkeypatch):
    _install_generated_messages(monkeypatch)
    node = _FakeNode()
    runner = ClosedLoopTrajectoryRunner(node)

    runner.subscribe()
    publisher = ContinuousTrajectoryPublisher(node)
    trajectory = publisher.publish(
        CirclePath(0.0, 0.0, 0.0, 0.02), 0.0, 2.0, horizon_sec=0.1)
    runner._send_safe_stop()

    assert [topic for topic, _, _ in node.readers] == [
        "/apollo/localization/pose", "/apollo/canbus/chassis", "/apollo/control"]
    assert trajectory.gear == 3
    assert node.writers[-1][2].messages[0].gear_location == 3


def test_closed_loop_reads_active_lat_debug_and_does_not_default_to_mpc(monkeypatch):
    runner = ClosedLoopTrajectoryRunner(None)
    runner._on_control(_ControlWithDebug("simple_lat_debug"))

    assert runner._latest["control_debug_available"]
    assert runner._latest["lateral_error_m"] == 0.2
    assert runner._latest["heading_error_rad"] == 0.1
    assert runner._latest["reference_kappa_1pm"] == 0.02


def test_closed_loop_marks_missing_debug_as_unavailable():
    runner = ClosedLoopTrajectoryRunner(None)
    runner._on_control(_ControlWithDebug(None))

    assert not runner._latest["control_debug_available"]
    assert np.isnan(runner._latest["lateral_error_m"])
    assert np.isnan(runner._latest["reference_kappa_1pm"])
