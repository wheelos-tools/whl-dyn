"""Continuous, time-parameterized paths for direct ADCTrajectory publication."""

import math
from dataclasses import dataclass

import numpy as np


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class PathSample:
    """One geometric path state in the map frame."""

    s: float
    x: float
    y: float
    theta: float
    kappa: float
    phase: str = ""


class CirclePath:
    """An indefinitely long constant-curvature path anchored once at startup."""

    def __init__(self, x0, y0, theta0, kappa):
        if abs(float(kappa)) < 1e-9:
            raise ValueError("circle curvature must be non-zero")
        self.x0 = float(x0)
        self.y0 = float(y0)
        self.theta0 = float(theta0)
        self.kappa = float(kappa)

    def sample(self, s):
        distance = float(s)
        theta = self.theta0 + self.kappa * distance
        return PathSample(
            s=distance,
            x=self.x0 + (math.sin(theta) - math.sin(self.theta0)) / self.kappa,
            y=self.y0 - (math.cos(theta) - math.cos(self.theta0)) / self.kappa,
            theta=_normalize_angle(theta),
            kappa=self.kappa,
            phase="arc",
        )


class ClothoidCirclePath:
    """Straight -> clothoid -> circular arc -> clothoid transition path.

    The numerical centerline is generated once in the initial localization
    frame.  Publishing later windows only samples this immutable path, which
    keeps common points identical between planning frames.
    """

    def __init__(self, x0, y0, theta0, radius_m, entry_length_m,
                 arc_angle_rad, exit_length_m, direction=1.0,
                 straight_entry_length_m=0.0, straight_exit_length_m=0.0,
                 resolution_m=0.05):
        if radius_m <= 0.0 or entry_length_m <= 0.0 or arc_angle_rad <= 0.0:
            raise ValueError("radius, entry length and arc angle must be positive")
        if (exit_length_m < 0.0 or resolution_m <= 0.0 or
                straight_entry_length_m < 0.0 or straight_exit_length_m < 0.0):
            raise ValueError("exit length must be non-negative and resolution positive")
        self._kappa_peak = math.copysign(1.0 / float(radius_m), direction)
        self.straight_entry_length_m = float(straight_entry_length_m)
        self.entry_length_m = float(entry_length_m)
        arc_length = float(radius_m) * float(arc_angle_rad)
        self.arc_length_m = arc_length
        self.exit_length_m = float(exit_length_m)
        self.straight_exit_length_m = float(straight_exit_length_m)
        total = (self.straight_entry_length_m + self.entry_length_m + arc_length +
                 self.exit_length_m + self.straight_exit_length_m)
        steps = int(math.ceil(total / float(resolution_m))) + 1
        distances = np.linspace(0.0, total, steps)
        states = []
        x, y, theta = float(x0), float(y0), float(theta0)
        previous_s = 0.0
        for distance in distances:
            midpoint = 0.5 * (previous_s + float(distance))
            kappa = self._curvature(midpoint)
            ds = float(distance) - previous_s
            theta_mid = theta + 0.5 * kappa * ds
            x += ds * math.cos(theta_mid)
            y += ds * math.sin(theta_mid)
            theta += kappa * ds
            states.append(PathSample(float(distance), x, y,
                                     _normalize_angle(theta), kappa,
                                     self._phase(midpoint)))
            previous_s = float(distance)
        self._samples = states
        self.length_m = total

    def _curvature(self, s):
        entry_start = self.straight_entry_length_m
        arc_start = entry_start + self.entry_length_m
        exit_start = arc_start + self.arc_length_m
        exit_end = exit_start + self.exit_length_m
        if s < entry_start:
            return 0.0
        if s < arc_start:
            return self._kappa_peak * (s - entry_start) / self.entry_length_m
        if s < exit_start:
            return self._kappa_peak
        if self.exit_length_m > 0.0 and s < exit_end:
            return self._kappa_peak * (exit_end - s) / self.exit_length_m
        return 0.0

    def _phase(self, s):
        entry_start = self.straight_entry_length_m
        arc_start = entry_start + self.entry_length_m
        exit_start = arc_start + self.arc_length_m
        exit_end = exit_start + self.exit_length_m
        if s < entry_start:
            return "straight_entry"
        if s < arc_start:
            return "clothoid_entry"
        if s < exit_start or self.exit_length_m == 0.0:
            return "arc"
        if self.exit_length_m > 0.0 and s < exit_end:
            return "clothoid_exit"
        return "straight_exit"

    def sample(self, s):
        distance = float(s)
        if distance < 0.0 or distance > self.length_m:
            raise ValueError("trajectory window exceeds clothoid path length")
        index = min(
            len(self._samples) - 1,
            max(1, int(np.searchsorted([point.s for point in self._samples], distance))),
        )
        before, after = self._samples[index - 1], self._samples[index]
        ratio = ((distance - before.s) / (after.s - before.s)
                 if after.s > before.s else 0.0)
        theta_delta = _normalize_angle(after.theta - before.theta)
        return PathSample(
            s=distance,
            x=before.x + ratio * (after.x - before.x),
            y=before.y + ratio * (after.y - before.y),
            theta=_normalize_angle(before.theta + ratio * theta_delta),
            kappa=before.kappa + ratio * (after.kappa - before.kappa),
            phase=self._phase(distance),
        )


def build_trajectory_window(path, elapsed_sec, speed_mps, horizon_sec,
                            point_interval_sec=0.05, clamp_at_path_end=False):
    """Sample a fixed global path into one forward planning-time window.

    ``elapsed_sec`` is experiment time, not a per-frame reset.  A point at
    absolute experiment time ``t`` always maps to the same geometry, no matter
    which ADCTrajectory frame contains it.
    """

    if speed_mps <= 0.0 or horizon_sec <= 0.0 or point_interval_sec <= 0.0:
        raise ValueError("speed, horizon and point interval must be positive")
    count = int(math.floor(horizon_sec / point_interval_sec)) + 1
    points = []
    for index in range(count):
        relative_time = index * point_interval_sec
        global_s = (float(elapsed_sec) + relative_time) * float(speed_mps)
        if clamp_at_path_end and hasattr(path, "length_m"):
            global_s = min(global_s, float(path.length_m))
        sample = path.sample(global_s)
        points.append((relative_time, global_s, sample))
    return points
