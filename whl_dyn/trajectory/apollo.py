"""Apollo CyberRT publisher for continuous direct planning trajectories."""

import time

from whl_dyn.trajectory.continuous import build_trajectory_window


class ContinuousTrajectoryPublisher:
    """Publish overlapping ADCTrajectory windows from one immutable path."""

    def __init__(self, node, topic="/apollo/planning", module_name="whl_dyn_path"):
        from modules.common_msgs.planning_msgs import planning_pb2

        self._planning_pb2 = planning_pb2
        self._writer = node.create_writer(topic, planning_pb2.ADCTrajectory)
        self._module_name = module_name
        self._sequence = 0

    def publish(self, path, elapsed_sec, speed_mps, horizon_sec=8.0,
                point_interval_sec=0.05):
        trajectory = self._planning_pb2.ADCTrajectory()
        self._sequence += 1
        trajectory.header.timestamp_sec = time.time()
        trajectory.header.sequence_num = self._sequence
        trajectory.header.module_name = self._module_name
        trajectory.total_path_time = float(horizon_sec)
        trajectory.total_path_length = float(speed_mps) * float(horizon_sec)
        trajectory.is_replan = False
        trajectory.trajectory_type = self._planning_pb2.ADCTrajectory.NORMAL
        for relative_time, global_s, sample in build_trajectory_window(
                path, elapsed_sec, speed_mps, horizon_sec, point_interval_sec):
            point = trajectory.trajectory_point.add()
            point.path_point.x = sample.x
            point.path_point.y = sample.y
            point.path_point.theta = sample.theta
            # s is local to this window; geometry is global and continuous.
            point.path_point.s = global_s - float(elapsed_sec) * float(speed_mps)
            point.path_point.kappa = sample.kappa
            point.v = float(speed_mps)
            point.a = 0.0
            point.relative_time = relative_time
        self._writer.write(trajectory)
        return trajectory
