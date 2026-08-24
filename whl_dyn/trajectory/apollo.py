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
                point_interval_sec=0.05, planning_cycle_time_sec=0.05):
        """Publish the future window beginning at the next planning cycle.

        Apollo's TrajectoryStitcher time-matches the preceding trajectory,
        preserves valid points, then resets ``path_point.s`` at the stitch
        point.  This direct test publisher has no planner-owned prior path to
        splice, so it instead samples one immutable global reference at that
        same future boundary.  Successive windows therefore overlap with
        identical geometry while their local ``s`` starts at zero.
        """

        if planning_cycle_time_sec < 0.0:
            raise ValueError("planning_cycle_time_sec must be non-negative")
        trajectory = self._planning_pb2.ADCTrajectory()
        self._sequence += 1
        trajectory.header.timestamp_sec = time.time()
        trajectory.header.sequence_num = self._sequence
        trajectory.header.module_name = self._module_name
        trajectory.total_path_time = (
            float(horizon_sec) + float(planning_cycle_time_sec))
        trajectory.total_path_length = float(speed_mps) * float(horizon_sec)
        trajectory.is_replan = False
        trajectory.trajectory_type = self._planning_pb2.ADCTrajectory.NORMAL
        window_start = float(elapsed_sec) + float(planning_cycle_time_sec)
        points = build_trajectory_window(
            path, window_start, speed_mps, horizon_sec, point_interval_sec,
            clamp_at_path_end=True)
        first_s = points[0][1]
        for relative_time, global_s, sample in points:
            point = trajectory.trajectory_point.add()
            point.path_point.x = sample.x
            point.path_point.y = sample.y
            point.path_point.theta = sample.theta
            # s is local to this window; geometry is global and continuous.
            point.path_point.s = global_s - first_s
            point.path_point.kappa = sample.kappa
            point.v = (0.0 if hasattr(path, "length_m") and
                       global_s >= float(path.length_m) else float(speed_mps))
            point.a = 0.0
            point.relative_time = relative_time + float(planning_cycle_time_sec)
        self._writer.write(trajectory)
        return trajectory
