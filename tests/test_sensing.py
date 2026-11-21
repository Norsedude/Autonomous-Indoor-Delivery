"""Tests for the degraded-sensor extension (sensing.py)."""

from __future__ import annotations

import math
import random
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "controllers" / "project_starter"))

from odometry import Pose2D  # noqa: E402
from sensing import RobustScanFilter, SensorDegrader  # noqa: E402


def circle_scan(radius: float = 1.0, count: int = 360, noise: float = 0.0, seed: int = 0):
    rng = random.Random(seed)
    points = []
    for k in range(count):
        angle = math.pi - k * 2 * math.pi / count
        r = radius + rng.gauss(0.0, noise) if noise else radius
        points.append((r * math.cos(angle), r * math.sin(angle)))
    return points


class DegraderTests(unittest.TestCase):
    def test_disabled_is_identity(self) -> None:
        scan = circle_scan()
        degrader = SensorDegrader({"enabled": False}, seed=1)
        self.assertIs(degrader.apply(scan, [])[0], scan)

    def test_same_seed_is_reproducible(self) -> None:
        settings = {"enabled": True, "beam_dropout_fraction": 0.2, "range_noise_std_m": 0.03, "outlier_fraction": 0.05}
        a = SensorDegrader(settings, seed=3).apply(circle_scan(), [])
        b = SensorDegrader(settings, seed=3).apply(circle_scan(), [])
        self.assertEqual(a, b)

    def test_beam_dropout_becomes_no_return(self) -> None:
        degrader = SensorDegrader({"enabled": True, "beam_dropout_fraction": 0.3}, seed=2)
        points, free = degrader.apply(circle_scan(), [])
        self.assertEqual(len(points) + len(free), 360)
        self.assertAlmostEqual(len(free) / 360, 0.3, delta=0.06)

    def test_scan_dropout_lasts_a_burst(self) -> None:
        degrader = SensorDegrader({"enabled": True, "scan_dropout_prob": 1.0, "scan_dropout_burst_steps": 4}, seed=1)
        for _ in range(4):
            self.assertEqual(degrader.apply(circle_scan(), []), ([], []))
        self.assertEqual(degrader.dropped_scans, 4)


class RobustFilterTests(unittest.TestCase):
    def test_noise_is_estimated(self) -> None:
        robust = RobustScanFilter({"robust": True, "noise_ema_alpha": 1.0})
        robust.process(circle_scan(noise=0.04, seed=5), [], Pose2D(0, 0, 0))
        self.assertAlmostEqual(robust.noise_estimate, 0.04, delta=0.01)

    def test_median_removes_isolated_outliers(self) -> None:
        scan = circle_scan()
        scan[100] = (0.1 * scan[100][0], 0.1 * scan[100][1])
        points, _, held = RobustScanFilter({"robust": True}).process(scan, [], Pose2D(0, 0, 0))
        self.assertFalse(held)
        self.assertAlmostEqual(math.hypot(*points[100]), 1.0, places=6)

    def test_dropout_holds_and_reprojects_last_scan(self) -> None:
        robust = RobustScanFilter({"robust": True, "hold_max_steps": 2})
        wall_ahead = [(1.0, 0.01 * k) for k in range(-10, 30)]
        robust.process(wall_ahead, [], Pose2D(0, 0, 0))
        points, free, held = robust.process([], [], Pose2D(0.5, 0.0, 0.0))
        self.assertTrue(held)
        self.assertEqual(free, [])
        self.assertAlmostEqual(points[0][0], 0.5, places=6)
        robust.process([], [], Pose2D(0.5, 0.0, 0.0))
        self.assertFalse(robust.process([], [], Pose2D(0.5, 0.0, 0.0))[2])

    def test_isolated_free_rays_are_not_open_space(self) -> None:
        robust = RobustScanFilter({"robust": True})
        step = 2 * math.pi / 360
        isolated = [0.0, 1.0, 2.0]
        sector = [-1.0 + k * step for k in range(5)]
        _, free, _ = robust.process(circle_scan(count=352), isolated + sector, Pose2D(0, 0, 0))
        self.assertEqual(sorted(round(b, 6) for b in free), sorted(round(b, 6) for b in sector))


if __name__ == "__main__":
    unittest.main()
