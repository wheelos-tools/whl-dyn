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
        )


class ClothoidCirclePath:
    """Straight -> clothoid -> circular arc -> clothoid transition path.

    The numerical centerline is generated once in the initial localization
    frame.  Publishing later windows only samples this immutable path, which
    keeps common points identical between planning frames.
    """

    def __init__(self, x0, y0, theta0, radius_m, entry_length_m,
                 arc_angle_rad, exit_length_m, direction=1.0,
                 resolution_m=0.05):
        if radius_m <= 0.0 or entry_length_m <= 0.0 or arc_angle_rad <= 0.0:
            raise ValueError("radius, entry length and arc angle must be positive")
        if exit_length_m <= 0.0 or resolution_m <= 0.0:
            raise ValueError("exit length and resolution must be positive")
        self._kappa_peak = math.copysign(1.0 / float(radius_m), direction)
        arc_length = float(radius_m) * float(arc_angle_rad)
        total = float(entry_length_m) + arc_length + float(exit_length_m)
        steps = int(math.ceil(total / float(resolution_m))) + 1
        distances = np.linspace(0.0, total, steps)
        states = []
        x, y, theta = float(x0), float(y0), float(theta0)
        previous_s = 0.0
        for distance in distances:
            midpoint = 0.5 * (previous_s + float(distance))
            kappa = self._curvature(midpoint, entry_length_m, arc_length,
                                    exit_length_m)
            ds = float(distance) - previous_s
            theta_mid = theta + 0.5 * kappa * ds
            x += ds * math.cos(theta_mid)
            y += ds * math.sin(theta_mid)
            theta += kappa * ds
            states.append(PathSample(float(distance), x, y,
                                     _normalize_angle(theta), kappa))
            previous_s = float(distance)
        self._samples = states
        self.length_m = total

    def _curvature(self, s, entry_length, arc_length, exit_length):
        if s < entry_length:
            return self._kappa_peak * s / entry_length
        if s < entry_length + arc_length:
            return self._kappa_peak
        remaining = max(0.0, entry_length + arc_length + exit_length - s)
        return self._kappa_peak * remaining / exit_length

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
        )


def build_trajectory_window(path, elapsed_sec, speed_mps, horizon_sec,
                            point_interval_sec=0.05):
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
        sample = path.sample(global_s)
        points.append((relative_time, global_s, sample))
    return points
