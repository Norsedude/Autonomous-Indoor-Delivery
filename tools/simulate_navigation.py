#!/usr/bin/env python3
"""Offline closed-loop simulation of the full mission controller.

The simulator runs ``controllers/project_starter/mission.py`` (the same code
as the Webots controller) against the wall geometry of the world, with
seeded wheel slip, LiDAR noise, blocker motion, and kidnapping.  It is a fast
development and regression tool, not a replacement for Webots runs.

Examples::

    python tools/simulate_navigation.py --method team --scenario nominal --seed 1
    python tools/simulate_navigation.py --method baseline --scenario kidnapped --seed 3
    python tools/simulate_navigation.py --method team --all-scenarios --seeds 1 2 3 4 5 \
        --output-dir results/offline
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "controllers" / "project_starter"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from mission import MissionController, deep_merge, load_project_config  # noqa: E402
from occupancy_grid import OccupancyGrid  # noqa: E402
from odometry import Pose2D, wrap_angle  # noqa: E402
from simulate_baseline import (  # noqa: E402
    STATIC_RECTANGLES,
    blocker_rectangle,
    lidar_scan,
    move_toward,
    point_rectangle_distance,
)

EXPERIMENTS_PATH = PROJECT_ROOT / "config" / "experiments.json"

FIELDNAMES = [
    "time_s", "gt_x_m", "gt_y_m", "gt_theta_rad", "est_x_m", "est_y_m", "est_theta_rad",
    "position_error_m", "heading_error_rad", "mission_goal_index", "goals_reached_ground_truth",
    "active_waypoint", "minimum_lidar_range_m", "lidar_point_count", "controller_state",
    "safety_state", "escape_turn_direction", "distance_to_waypoint_m",
    "heading_error_to_waypoint_rad", "commanded_linear_speed_m_s",
    "commanded_angular_speed_rad_s", "left_encoder_rad", "right_encoder_rad",
    "left_wheel_speed_rad_s", "right_wheel_speed_rad_s", "contact_now", "collision_episodes",
    "ground_truth_path_length_m", "mission_complete_reported", "mode",
]


def lidar_scan_with_free_rays(pose, rectangles, ray_count):
    """Ideal scan plus the robot-frame bearings of rays with no return."""
    points = lidar_scan(pose, rectangles, ray_count=ray_count)
    step = 2.0 * math.pi / ray_count
    hit = {round((math.atan2(y, x) + math.pi) / step) % ray_count for x, y in points}
    empty = [-math.pi + index * step for index in range(ray_count) if index not in hit]
    return points, empty


def load_experiments() -> dict:
    return json.loads(EXPERIMENTS_PATH.read_text(encoding="utf-8"))


def build_config(
    method: str, scenario: str, seed: int, experiments: dict | None = None, degradation: str = "none"
) -> tuple[dict, dict]:
    """Return (controller config, simulation settings) for a method/scenario/seed."""
    experiments = experiments or load_experiments()
    config = load_project_config(overrides_path=None)
    config = deep_merge(config, experiments["methods"][method]["config"])
    scenario_spec = experiments["scenarios"][scenario]
    config = deep_merge(config, scenario_spec.get("config", {}))
    config = deep_merge(config, {"experiment": {"method": method, "scenario": scenario, "seed": seed}})
    if degradation != "none":
        config = deep_merge(config, {"sensor_degradation": experiments["degradations"][degradation]})
    simulation = deep_merge(experiments.get("simulation_defaults", {}), scenario_spec.get("simulation", {}))
    return config, simulation


def run_trial(
    method: str,
    scenario: str,
    seed: int,
    output_csv: Path | None = None,
    max_time_s: float | None = None,
    log_every_steps: int = 2,
    experiments: dict | None = None,
    config_overrides: dict | None = None,
    verbose: bool = False,
    degradation: str = "none",
) -> dict:
    config, sim = build_config(method, scenario, seed, experiments, degradation)
    if config_overrides:
        config = deep_merge(config, config_overrides)
    waypoints = json.loads((PROJECT_ROOT / "maps" / "waypoints.json").read_text(encoding="utf-8"))
    grid = OccupancyGrid.from_files(PROJECT_ROOT / "maps" / "building_map.yaml")
    dt = 0.032
    rng = random.Random(10_000 + seed)

    robot = config["robot"]
    wheel_radius = robot["wheel_radius_m"] * (1.0 + rng.gauss(0.0, sim["wheel_radius_error_std"]))
    track = robot["effective_track_width_m"] * (1.0 + rng.gauss(0.0, sim["track_error_std"]))
    slip_std = sim["wheel_slip_std"]
    lidar_noise = sim["lidar_noise_std_m"]
    lidar_rays = int(sim["lidar_rays"])
    max_time_s = float(sim["max_time_s"] if max_time_s is None else max_time_s)

    initial = config["coordinate_frame"]["initial_pose"]
    offset = sim.get("start_offset", [0.0, 0.0, 0.0])
    actual = Pose2D(initial[0] + offset[0], initial[1] + offset[1], wrap_angle(initial[2] + offset[2]))
    blocker = list(sim.get("blocker", [0.0, 0.0]))
    blocker_events = sorted(sim.get("blocker_events", []), key=lambda event: event[0])
    kidnap_time = sim.get("kidnap_time_s")
    kidnap_pose = sim.get("kidnap_pose", [0.9, -2.1, math.pi])

    controller = MissionController(config, waypoints, grid, dt, seed=seed)
    goal_points = [tuple(goal["position"]) for goal in controller.goals]
    tolerance = config["evaluation"]["position_goal_tolerance_m"]
    body_radius = robot["body_radius_m"]

    left_encoder = right_encoder = 0.0
    left_speed = right_speed = 0.0
    goals_reached = 0
    collision_episodes = 0
    contact_previous = False
    last_collision_time = -math.inf
    path_length = 0.0
    position_sq = heading_sq = 0.0
    samples = 0
    kidnapped_at = None
    recovery_time = None
    recovered_since = None
    complete_time = None
    rows: list[dict] = []
    time_s = 0.0
    step = 0
    wall_start = time.perf_counter()

    while time_s < max_time_s:
        rectangles = STATIC_RECTANGLES + ([blocker_rectangle(tuple(blocker))] if blocker is not None else [])
        while blocker_events and time_s >= blocker_events[0][0]:
            _, position = blocker_events.pop(0)
            blocker = position
            rectangles = STATIC_RECTANGLES + ([blocker_rectangle(tuple(blocker))] if blocker is not None else [])
            if verbose:
                print(f"t={time_s:.1f}s blocker moved to {blocker}")
        if kidnap_time is not None and kidnapped_at is None and time_s >= kidnap_time:
            actual = Pose2D(*kidnap_pose)
            left_speed = right_speed = 0.0
            kidnapped_at = time_s
            if verbose:
                print(f"t={time_s:.1f}s kidnapped to {kidnap_pose}")

        scan, empty_bearings = lidar_scan_with_free_rays(actual, rectangles, lidar_rays)
        if lidar_noise > 0.0:
            noisy = []
            for x, y in scan:
                distance = math.hypot(x, y)
                scale = max(0.0, distance + rng.gauss(0.0, lidar_noise)) / distance
                noisy.append((x * scale, y * scale))
            scan = noisy
        result = controller.step(left_encoder, right_encoder, scan, empty_bearings)

        maximum_change = robot["motor_acceleration_rad_s2"] * dt
        left_speed = move_toward(left_speed, result.left_speed, maximum_change)
        right_speed = move_toward(right_speed, result.right_speed, maximum_change)
        delta_left_angle = left_speed * dt
        delta_right_angle = right_speed * dt
        left_encoder += delta_left_angle
        right_encoder += delta_right_angle
        delta_left = wheel_radius * delta_left_angle * (1.0 + rng.gauss(0.0, slip_std))
        delta_right = wheel_radius * delta_right_angle * (1.0 + rng.gauss(0.0, slip_std))
        distance = 0.5 * (delta_left + delta_right)
        heading_change = (delta_right - delta_left) / track
        midpoint = actual.theta + 0.5 * heading_change
        new_x = actual.x + distance * math.cos(midpoint)
        new_y = actual.y + distance * math.sin(midpoint)
        clearance = min(point_rectangle_distance(new_x, new_y, r) - body_radius for r in rectangles)
        contact_now = clearance < 0.0
        if not contact_now:
            path_length += math.hypot(new_x - actual.x, new_y - actual.y)
            actual.x, actual.y = new_x, new_y
        actual.theta = wrap_angle(actual.theta + heading_change)
        if contact_now and not contact_previous and time_s - last_collision_time >= 0.5:
            collision_episodes += 1
            last_collision_time = time_s
        contact_previous = contact_now

        if goals_reached < len(goal_points) and math.dist((actual.x, actual.y), goal_points[goals_reached]) <= tolerance:
            goals_reached += 1
            if verbose:
                print(f"t={time_s:.1f}s ground-truth goal {goals_reached}/{len(goal_points)}")

        estimate = result.estimate
        position_error = math.hypot(estimate.x - actual.x, estimate.y - actual.y)
        heading_error = abs(wrap_angle(estimate.theta - actual.theta))
        position_sq += position_error ** 2
        heading_sq += heading_error ** 2
        samples += 1
        if kidnapped_at is not None and recovery_time is None:
            if position_error < 0.20 and heading_error < 0.30:
                recovered_since = time_s if recovered_since is None else recovered_since
                if time_s - recovered_since >= 3.0:
                    recovery_time = recovered_since - kidnapped_at
            else:
                recovered_since = None

        telemetry = result.telemetry
        if step % log_every_steps == 0 or telemetry["mission_complete"]:
            rows.append({
                "time_s": f"{time_s:.3f}",
                "gt_x_m": f"{actual.x:.6f}", "gt_y_m": f"{actual.y:.6f}", "gt_theta_rad": f"{actual.theta:.6f}",
                "est_x_m": f"{estimate.x:.6f}", "est_y_m": f"{estimate.y:.6f}", "est_theta_rad": f"{estimate.theta:.6f}",
                "position_error_m": f"{position_error:.6f}", "heading_error_rad": f"{heading_error:.6f}",
                "mission_goal_index": telemetry["mission_goal_index"],
                "goals_reached_ground_truth": goals_reached,
                "active_waypoint": telemetry["active_waypoint"],
                "minimum_lidar_range_m": telemetry["minimum_lidar_range"] if telemetry["minimum_lidar_range"] is not None else "",
                "lidar_point_count": telemetry["lidar_point_count"],
                "controller_state": telemetry["controller_state"],
                "safety_state": telemetry["safety_state"],
                "escape_turn_direction": telemetry["escape_turn_direction"],
                "distance_to_waypoint_m": telemetry["distance_to_waypoint"],
                "heading_error_to_waypoint_rad": telemetry["heading_error_to_waypoint"],
                "commanded_linear_speed_m_s": telemetry["commanded_linear_speed"],
                "commanded_angular_speed_rad_s": telemetry["commanded_angular_speed"],
                "left_encoder_rad": f"{left_encoder:.6f}", "right_encoder_rad": f"{right_encoder:.6f}",
                "left_wheel_speed_rad_s": f"{left_speed:.6f}", "right_wheel_speed_rad_s": f"{right_speed:.6f}",
                "contact_now": int(contact_now), "collision_episodes": collision_episodes,
                "ground_truth_path_length_m": f"{path_length:.6f}",
                "mission_complete_reported": int(telemetry["mission_complete"]),
                "mode": telemetry["mode"],
            })
        if telemetry["mission_complete"]:
            complete_time = time_s
            break
        if controller.planning_mode != "astar" and controller.safety.blocked and time_s > 30.0:
            # The baseline safety layer latches "blocked" forever; stop early.
            if controller.safety.state == "blocked" and step % 500 == 0 and time_s > 120.0:
                break
        time_s += dt
        step += 1

    home = goal_points[-1]
    final_home_error = math.dist((actual.x, actual.y), home)
    success = (
        complete_time is not None
        and goals_reached == len(goal_points)
        and collision_episodes == 0
        and final_home_error <= tolerance
    )
    summary = {
        "method": method,
        "scenario": scenario,
        "degradation": degradation,
        "seed": seed,
        "success": success,
        "goals_reached": goals_reached,
        "goals_total": len(goal_points),
        "mission_complete_reported": complete_time is not None,
        "completion_time_s": complete_time,
        "simulated_time_s": time_s,
        "path_length_m": path_length,
        "collision_episodes": collision_episodes,
        "position_rmse_m": math.sqrt(position_sq / max(1, samples)),
        "heading_rmse_rad": math.sqrt(heading_sq / max(1, samples)),
        "final_home_error_m": final_home_error,
        "kidnap_time_s": kidnapped_at,
        "recovery_time_s": recovery_time,
        "recovery_events": controller.localizer.recovery_events,
        "replans": controller.replan_count,
        "wall_clock_s": time.perf_counter() - wall_start,
    }
    if output_csv is not None:
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        with output_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
            writer.writeheader()
            writer.writerows(rows)
    return summary


def main() -> None:
    experiments = load_experiments()
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--method", choices=sorted(experiments["methods"]), default="team")
    parser.add_argument("--scenario", choices=sorted(experiments["scenarios"]), default="nominal")
    parser.add_argument("--all-scenarios", action="store_true", help="Run the four required scenarios")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--seeds", type=int, nargs="+", help="Run several seeds (overrides --seed)")
    parser.add_argument("--max-time-s", type=float)
    parser.add_argument("--degradation", choices=sorted(experiments["degradations"]), default="none")
    parser.add_argument("--output-dir", type=Path, help="Write one evaluation-format CSV per trial and summary.json")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    scenarios = experiments["required_scenarios"] if args.all_scenarios else [args.scenario]
    seeds = args.seeds or [args.seed]
    summaries = []
    for scenario in scenarios:
        for seed in seeds:
            csv_path = None
            if args.output_dir is not None:
                csv_path = args.output_dir / f"{args.method}_{scenario}_{args.degradation}_seed{seed}.csv"
            summary = run_trial(
                args.method, scenario, seed, csv_path, args.max_time_s,
                experiments=experiments, verbose=args.verbose, degradation=args.degradation,
            )
            summaries.append(summary)
            print(json.dumps(summary))
    if args.output_dir is not None:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / f"summary_{args.method}_{args.degradation}.json").write_text(json.dumps(summaries, indent=2) + "\n", encoding="utf-8")
    raise SystemExit(0 if all(s["success"] for s in summaries) else 1)


if __name__ == "__main__":
    main()
