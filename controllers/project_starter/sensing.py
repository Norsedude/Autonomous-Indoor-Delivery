"""Team extension: robustness under degraded LiDAR.

``SensorDegrader`` is the *experimental condition*.  It corrupts the LiDAR
data in software with a seeded random generator, so the same degradation is
reproducible in Webots and in the offline simulator:

* whole-scan dropouts that start with probability ``scan_dropout_prob`` per
  step and last ``scan_dropout_burst_steps`` steps (no data at all);
* per-beam dropouts (``beam_dropout_fraction``): the beam reports no return,
  exactly like an out-of-range beam;
* extra zero-mean Gaussian range noise (``range_noise_std_m``);
* spurious short returns (``outlier_fraction``): the range is replaced by a
  uniform value between the minimum range and the true range (dust, glare).

``RobustScanFilter`` is the *mitigation* ("prediction and denoise" in the
proposal):

1. Prediction during dropouts: when a scan is missing or nearly empty, the
   last good scan is re-projected into the current robot frame with the
   odometry motion since it was taken and given to the local avoider for up
   to ``hold_max_steps``.  The localizer never sees a held scan.
2. Denoising: a median of three adjacent ranges (only across angularly
   adjacent beams) removes isolated outliers and reduces noise.
3. Noise estimation: the range noise is estimated online from second
   differences of adjacent ranges,
       e_i = r_i - (r_(i-1) + r_(i+1)) / 2,  sigma = 1.4826 MAD(e) / sqrt(1.5),
   which the localizer uses to widen its likelihood field and inlier gate.
4. Free-space evidence: a beam without a return clears obstacle cells only
   if it belongs to a run of at least ``free_ray_min_run`` adjacent
   no-return beams; isolated dropouts are not evidence of open space.
"""

from __future__ import annotations

import math
from random import Random


DEFAULT_DEGRADATION: dict = {
    "enabled": False,
    "name": "none",
    "scan_dropout_prob": 0.0,
    "scan_dropout_burst_steps": 1,
    "beam_dropout_fraction": 0.0,
    "range_noise_std_m": 0.0,
    "outlier_fraction": 0.0,
    "minimum_range_m": 0.05,
}

DEFAULT_ROBUST: dict = {
    "robust": False,
    "hold_max_steps": 20,
    "min_points_fraction": 0.25,
    "median_window": 3,
    "adjacent_beam_gap_rad": 0.12,
    "noise_ema_alpha": 0.1,
    "noise_floor_m": 0.0,
    "free_ray_min_run": 3,
}


class SensorDegrader:
    def __init__(self, settings: dict | None, seed: int) -> None:
        self.settings = dict(DEFAULT_DEGRADATION)
        if settings:
            self.settings.update(settings)
        self.enabled = bool(self.settings["enabled"])
        self.random = Random(7919 * int(seed) + 13)
        self.burst_remaining = 0
        self.dropped_scans = 0

    def apply(self, scan_points, free_bearings):
        if not self.enabled:
            return scan_points, free_bearings
        s = self.settings
        rng = self.random
        if self.burst_remaining > 0:
            self.burst_remaining -= 1
            self.dropped_scans += 1
            return [], []
        if rng.random() < float(s["scan_dropout_prob"]):
            self.burst_remaining = max(0, int(s["scan_dropout_burst_steps"]) - 1)
            self.dropped_scans += 1
            return [], []
        beam_dropout = float(s["beam_dropout_fraction"])
        noise = float(s["range_noise_std_m"])
        outliers = float(s["outlier_fraction"])
        minimum = float(s["minimum_range_m"])
        points: list[tuple[float, float]] = []
        free = list(free_bearings or [])
        for x, y in scan_points:
            distance = math.hypot(x, y)
            if distance <= 0.0:
                continue
            if rng.random() < beam_dropout:
                free.append(math.atan2(y, x))
                continue
            if rng.random() < outliers:
                corrupted = rng.uniform(minimum, distance)
            else:
                corrupted = max(minimum, distance + rng.gauss(0.0, noise)) if noise > 0.0 else distance
            scale = corrupted / distance
            points.append((x * scale, y * scale))
        return points, free


class RobustScanFilter:
    def __init__(self, settings: dict | None) -> None:
        self.settings = dict(DEFAULT_ROBUST)
        if settings:
            self.settings.update({k: v for k, v in settings.items() if k in DEFAULT_ROBUST})
        self.enabled = bool(self.settings["robust"])
        self.held_points: list[tuple[float, float]] = []
        self.held_pose: tuple[float, float, float] | None = None
        self.held_steps = 0
        self.typical_count = 0.0
        self.noise_estimate = 0.0
        self.held = False

    @staticmethod
    def _reproject(points, from_pose, to_pose):
        """Express robot-frame points taken at ``from_pose`` in ``to_pose``'s frame."""
        dx = from_pose[0] - to_pose[0]
        dy = from_pose[1] - to_pose[1]
        c1 = math.cos(to_pose[2])
        s1 = math.sin(to_pose[2])
        tx = c1 * dx + s1 * dy
        ty = -s1 * dx + c1 * dy
        dtheta = from_pose[2] - to_pose[2]
        c = math.cos(dtheta)
        s = math.sin(dtheta)
        return [(tx + c * x - s * y, ty + s * x + c * y) for x, y in points]

    def _estimate_noise(self, ranges, bearings) -> float | None:
        gap = float(self.settings["adjacent_beam_gap_rad"])
        residuals = []
        for i in range(1, len(ranges) - 1):
            if abs(math.remainder(bearings[i] - bearings[i - 1], 2 * math.pi)) > gap:
                continue
            if abs(math.remainder(bearings[i + 1] - bearings[i], 2 * math.pi)) > gap:
                continue
            residuals.append(abs(ranges[i] - 0.5 * (ranges[i - 1] + ranges[i + 1])))
        if len(residuals) < 20:
            return None
        residuals.sort()
        mad = residuals[len(residuals) // 2]
        return 1.4826 * mad / math.sqrt(1.5)

    def _median(self, ranges, bearings):
        gap = float(self.settings["adjacent_beam_gap_rad"])
        output = list(ranges)
        for i in range(1, len(ranges) - 1):
            if (
                abs(math.remainder(bearings[i] - bearings[i - 1], 2 * math.pi)) <= gap
                and abs(math.remainder(bearings[i + 1] - bearings[i], 2 * math.pi)) <= gap
            ):
                output[i] = sorted((ranges[i - 1], ranges[i], ranges[i + 1]))[1]
        return output

    def _contiguous_free(self, free_bearings, beam_count):
        minimum_run = int(self.settings["free_ray_min_run"])
        if not free_bearings or minimum_run <= 1:
            return list(free_bearings or [])
        step = 2.0 * math.pi / max(1, beam_count)
        ordered = sorted(math.remainder(b, 2 * math.pi) for b in free_bearings)
        runs: list[list[float]] = [[ordered[0]]]
        for bearing in ordered[1:]:
            if bearing - runs[-1][-1] <= 1.5 * step:
                runs[-1].append(bearing)
            else:
                runs.append([bearing])
        if len(runs) > 1 and ordered[0] + 2 * math.pi - ordered[-1] <= 1.5 * step:
            runs[0] = runs.pop() + runs[0]
        return [b for run in runs if len(run) >= minimum_run for b in run]

    def process(self, scan_points, free_bearings, odometry_pose):
        """Return ``(scan_for_control, free_bearings, held)``."""
        s = self.settings
        pose = (odometry_pose.x, odometry_pose.y, odometry_pose.theta)
        count = len(scan_points)
        too_few = count < 10 or (self.typical_count > 0 and count < float(s["min_points_fraction"]) * self.typical_count)
        if too_few:
            self.held = False
            if self.held_pose is not None and self.held_steps < int(s["hold_max_steps"]):
                self.held_steps += 1
                self.held = True
                return self._reproject(self.held_points, self.held_pose, pose), [], True
            return list(scan_points), [], False

        self.typical_count = count if self.typical_count == 0 else 0.95 * self.typical_count + 0.05 * count
        ranges = [math.hypot(x, y) for x, y in scan_points]
        bearings = [math.atan2(y, x) for x, y in scan_points]
        noise = self._estimate_noise(ranges, bearings)
        if noise is not None:
            alpha = float(s["noise_ema_alpha"])
            self.noise_estimate = noise if self.noise_estimate == 0.0 else (1 - alpha) * self.noise_estimate + alpha * noise
        filtered = self._median(ranges, bearings)
        points = [
            (r * math.cos(b), r * math.sin(b)) if r != r0 else p
            for r, r0, b, p in zip(filtered, ranges, bearings, scan_points)
        ]
        free = self._contiguous_free(free_bearings, count + len(free_bearings or []))
        self.held_points = points
        self.held_pose = pose
        self.held_steps = 0
        self.held = False
        return points, free, False
