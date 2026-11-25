from types import SimpleNamespace

from wheelos_msgs.planning_msgs import planning_pb2

from whl_dyn.trajectory.apollo import ContinuousTrajectoryPublisher
from whl_dyn.trajectory.continuous import CirclePath


def test_publisher_writes_serializable_generated_planning_message():
    messages = []
    writer = SimpleNamespace(write=messages.append)
    node = SimpleNamespace(create_writer=lambda topic, message_type: writer)
    publisher = ContinuousTrajectoryPublisher(node)

    trajectory = publisher.publish(
        CirclePath(0.0, 0.0, 0.0, 0.02), 0.0, 2.0, horizon_sec=0.1)

    decoded = planning_pb2.ADCTrajectory.FromString(
        messages[0].SerializeToString())
    assert decoded == trajectory
    assert len(decoded.trajectory_point) > 1
    assert decoded.trajectory_point[-1].path_point.s > 0.0
