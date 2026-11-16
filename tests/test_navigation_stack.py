"""Tests for the team navigation stack: A*, planning grid, MCL, avoidance, mission."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "controllers" / "project_starter"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from localization import StudentLocalizer  # noqa: E402
from navigation import ReactiveAvoider  # noqa: E402
from occupancy_grid import OccupancyGrid  # noqa: E402
from odometry import Pose2D  # noqa: E402
from planning import GlobalPlanner, astar, path_length  # noqa: E402
from simulate_baseline import STATIC_RECTANGLES, blocker_rectangle, lidar_scan  # noqa: E402
from simulate_navigation import lidar_scan_with_free_rays  # noqa: E402
from coordinate_frames import free_ray_bearings  # noqa: E402


def small_grid(rows: list[str], resolution: float = 0.1) -> OccupancyGrid:
    cells = [[100 if char == "#" else 0 for char in row] for row in rows]
    return OccupancyGrid(cells, resolution, 0.0, 0.0)


def cell_path_cost(path: list[tuple[int, int]]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


class AStarRequiredTests(unittest.TestCase):
    """The six cases listed in docs/astar_pseudocode.md."""

    def test_start_equals_goal(self) -> None:
        grid = small_grid(["...", "...", "..."])
        self.assertEqual(astar(grid, (1, 1), (1, 1)), [(1, 1)])

    def test_occupied_start_or_goal(self) -> None:
        grid = small_grid(["#..", "...", "..#"])
        self.assertEqual(astar(grid, (0, 0), (1, 1)), [])
        self.assertEqual(astar(grid, (1, 1), (2, 2)), [])

    def test_no_available_path(self) -> None:
        grid = small_grid(["..#..", "..#..", "..#.."])
        self.assertEqual(astar(grid, (1, 0), (1, 4)), [])

    def test_narrow_passage_removed_by_inflation(self) -> None:
        rows = [
            ".......",
            "###.###",
            ".......",
        ]
        grid = small_grid(rows)
        self.assertTrue(astar(grid, (0, 0), (2, 6)))
        self.assertEqual(astar(grid.inflated(0.1), (0, 0), (2, 6)), [])

    def test_diagonal_corner_cutting_is_rejected(self) -> None:
        grid = small_grid([".#", "#."])
        self.assertEqual(astar(grid, (0, 0), (1, 1)), [])
        grid = small_grid(["..", "#."])
        path = astar(grid, (0, 0), (1, 1))
        self.assertEqual(path, [(0, 0), (0, 1), (1, 1)])

    def test_known_optimum_length(self) -> None:
        grid = small_grid([".....", ".....", "....."])
        path = astar(grid, (0, 0), (2, 4))
        self.assertAlmostEqual(cell_path_cost(path), 2 * math.sqrt(2) + 2, places=6)
        walled = small_grid([".....", ".###.", "....."])
        path = astar(walled, (1, 0), (1, 4))
        # Diagonal moves into row 0 or 2 are forbidden next to the wall corner.
        self.assertAlmostEqual(cell_path_cost(path), 6.0, places=6)


class PlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.grid = OccupancyGrid.from_files(PROJECT_ROOT / "maps" / "building_map.yaml")

    def _assert_clear(self, planner: GlobalPlanner, waypoints) -> None:
        for a, b in zip(waypoints, waypoints[1:]):
            samples = max(1, int(math.dist(a, b) / 0.01))
            for k in range(samples + 1):
                t = k / samples
                x = a[0] + t * (b[0] - a[0])
                y = a[1] + t * (b[1] - a[1])
                clearance = min(
                    max(r.xmin - x, 0.0, x - r.xmax) ** 2 + max(r.ymin - y, 0.0, y - r.ymax) ** 2
                    for r in STATIC_RECTANGLES
                ) ** 0.5
                self.assertGreater(clearance, 0.037 + 0.02, (a, b, x, y))

    def test_every_mission_leg_has_a_collision_free_path(self) -> None:
        planner = GlobalPlanner(self.grid, 0.037)
        legs = [((-3.2, -2.2), (-3.2, 2.2)), ((-3.2, 2.2), (1.3, 2.2)), ((1.3, 2.2), (3.2, -2.2)), ((3.2, -2.2), (-3.2, -2.2))]
        for start, goal in legs:
            result = planner.plan(start, goal)
            self.assertEqual(result.status, "ok")
            self.assertEqual(result.waypoints[0], start)
            self.assertEqual(result.waypoints[-1], goal)
            self.assertLess(result.length_m, 1.4 * math.dist(start, goal) + 3.0)
            self._assert_clear(planner, result.waypoints)

    def test_lidar_obstacle_is_inserted_and_avoided(self) -> None:
        planner = GlobalPlanner(self.grid, 0.037)
        pose = Pose2D(-1.0, 0.0, 0.0)
        scan = lidar_scan(pose, STATIC_RECTANGLES + [blocker_rectangle()], ray_count=360)
        for _ in range(3):
            planner.update_dynamic(pose, scan)
        self.assertTrue(planner.path_blocked((-1.0, 0.0), [(1.0, 0.0)]))
        result = planner.plan((-1.0, 0.0), (1.0, 0.0))
        self.assertEqual(result.status, "ok")
        # Only the west face is visible from (-1, 0); the path must keep the
        # robot radius + margins (minus half a 5 cm cell) from every
        # observed obstacle cell.  The unseen far side is handled online by
        # re-observation and replanning (see the closed-loop scenarios).
        observed = planner.dynamic_points()
        self.assertTrue(observed)
        required = 0.037 + 0.035 + 0.02 - 0.025
        for a, b in zip(result.waypoints, result.waypoints[1:]):
            for k in range(101):
                t = k / 100
                x = a[0] + t * (b[0] - a[0])
                y = a[1] + t * (b[1] - a[1])
                self.assertGreater(min(math.hypot(x - ox, y - oy) for ox, oy in observed), required)

    def test_obstacle_cells_are_cleared_when_the_obstacle_disappears(self) -> None:
        planner = GlobalPlanner(self.grid, 0.037)
        pose = Pose2D(-1.0, 0.0, 0.0)
        with_blocker = lidar_scan(pose, STATIC_RECTANGLES + [blocker_rectangle()], ray_count=360)
        without, empty = lidar_scan_with_free_rays(pose, STATIC_RECTANGLES, 360)
        for _ in range(3):
            planner.update_dynamic(pose, with_blocker)
        self.assertTrue(planner.dynamic_points())
        for _ in range(8):
            planner.update_dynamic(pose, without, free_ray_bearings=empty)
        planner.refresh()
        self.assertFalse(planner.path_blocked((-1.0, 0.0), [(1.0, 0.0)]))

    def test_reports_when_no_path_exists(self) -> None:
        planner = GlobalPlanner(self.grid, 0.037)
        wall = [(0.0, y / 100.0) for y in range(-60, 61, 2)]
        planner.add_obstacle_points(Pose2D(0.0095, 0.0, 0.0), wall)
        self.assertEqual(planner.plan((-2.0, 0.0), (2.0, 0.0)).status, "no_path")


class LocalizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.grid = OccupancyGrid.from_files(PROJECT_ROOT / "maps" / "building_map.yaml")
        cls.rectangles = STATIC_RECTANGLES + [blocker_rectangle()]

    def _run(self, localizer, truth_poses, odometry_poses):
        estimate = None
        for truth, odometry in zip(truth_poses, odometry_poses):
            scan = lidar_scan(Pose2D(*truth), self.rectangles, ray_count=120)
            estimate = localizer.update(Pose2D(*odometry), (0.0, 0.0), scan, self.grid)
        return estimate

    def test_gaussian_belief_converges_while_turning(self) -> None:
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), 300, seed=1)
        localizer.initialize(self.grid)
        truth = [(-3.05, -2.1, 0.3 + 0.06 * k) for k in range(60)]
        odometry = [(-3.2, -2.2, 0.06 * k) for k in range(60)]
        estimate = self._run(localizer, truth, odometry)
        self.assertTrue(localizer.converged)
        self.assertLess(math.hypot(estimate.x - truth[-1][0], estimate.y - truth[-1][1]), 0.08)
        self.assertLess(abs(math.remainder(estimate.theta - truth[-1][2], 2 * math.pi)), 0.08)

    def test_odometry_baseline_passes_through(self) -> None:
        localizer = StudentLocalizer((0.0, 0.0, 0.0), 300, settings={"mode": "odometry_baseline"})
        localizer.initialize(self.grid)
        estimate = localizer.update(Pose2D(1.0, 2.0, 0.5), (0.0, 0.0), [], self.grid)
        self.assertEqual(estimate.as_tuple(), (1.0, 2.0, 0.5))

    def test_scan_search_contains_the_true_pose_and_its_congruent_twins(self) -> None:
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), 300, seed=1)
        localizer.initialize(self.grid)
        truth = (0.9, -2.1, math.pi)
        scan = lidar_scan(Pose2D(*truth), self.rectangles, ray_count=120)
        matches = localizer.scan_search(scan)
        best = matches[0][0]
        good = [m for m in matches if m[0] >= best - 1.0]
        self.assertTrue(any(math.hypot(m[1] - truth[0], m[2] - truth[1]) < 0.15 for m in good))
        self.assertGreaterEqual(len(good), 2)

    def test_kidnapping_is_detected(self) -> None:
        localizer = StudentLocalizer((-3.2, -2.2, 0.0), 300, seed=2)
        localizer.initialize(self.grid)
        truth = [(-3.2, -2.2, 0.06 * k) for k in range(40)]
        self._run(localizer, truth, truth)
        self.assertTrue(localizer.converged)
        kidnapped = [(0.9, -2.1, math.pi + 0.06 * k) for k in range(6)]
        odometry = [(-3.2, -2.2, 0.06 * (40 + k)) for k in range(6)]
        self._run(localizer, kidnapped, odometry)
        self.assertEqual(localizer.recovery_events, 1)
        self.assertTrue(localizer.recovering)
        self.assertTrue(
            any(math.hypot(h.estimate[0] - 0.9, h.estimate[1] + 2.1) < 0.2 for h in localizer.hypotheses)
        )


class FreeRayTests(unittest.TestCase):
    def test_missing_bearings_follow_scan_order_in_both_directions(self) -> None:
        count = 36
        for direction in (1.0, -1.0):
            angles = [direction * (-math.pi + k * 2 * math.pi / count) for k in range(count)]
            cloud = [(math.cos(a), math.sin(a)) if k % 5 else None for k, a in enumerate(angles)]
            bearings = free_ray_bearings(cloud)
            expected = [math.remainder(angles[k], 2 * math.pi) for k in range(count) if k % 5 == 0]
            self.assertEqual(len(bearings), len(expected))
            for got, want in zip(bearings, expected):
                self.assertAlmostEqual(math.remainder(got - want, 2 * math.pi), 0.0, places=6)


class AvoidanceTests(unittest.TestCase):
    def test_stops_turns_and_requests_replan_in_front_of_an_obstacle(self) -> None:
        avoider = ReactiveAvoider(0.037, replan_after_steps=3)
        scan = [(0.06, y / 100.0) for y in range(-10, 11)]
        requested = False
        for _ in range(3):
            v, w, _, request = avoider.filter_command(0.09, 0.0, scan)
            self.assertEqual(v, 0.0)
            self.assertNotEqual(w, 0.0)
            requested = requested or request
        self.assertTrue(requested)

    def test_rotation_in_place_is_never_blocked(self) -> None:
        avoider = ReactiveAvoider(0.037)
        v, w, _, request = avoider.filter_command(0.0, 0.5, [(0.05, 0.0)])
        self.assertEqual((v, w, request), (0.0, 0.5, False))

    def test_slows_down_when_approaching(self) -> None:
        avoider = ReactiveAvoider(0.037)
        v, _, _, _ = avoider.filter_command(0.09, 0.0, [(0.037 + 0.10, 0.0)])
        self.assertLess(v, 0.09)
        self.assertGreater(v, 0.0)


class ClosedLoopTests(unittest.TestCase):
    def test_team_system_reaches_first_goal_without_collision(self) -> None:
        from simulate_navigation import run_trial

        summary = run_trial("team", "nominal", 1, max_time_s=85.0)
        self.assertGreaterEqual(summary["goals_reached"], 1)
        self.assertEqual(summary["collision_episodes"], 0)
        self.assertLess(summary["position_rmse_m"], 0.10)


if __name__ == "__main__":
    unittest.main()
