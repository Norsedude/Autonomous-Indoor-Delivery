"""Mission layer: localization -> planning -> path following -> avoidance.

``MissionController`` contains all decision logic and no Webots calls, so the
same code runs inside the Webots controller (``project_starter.py``) and in
the offline closed-loop simulator (``tools/simulate_navigation.py``).

Two systems are selectable from ``config/project_config.json``:

* baseline: ``localization.mode = "odometry_baseline"`` and
  ``controller.planning_mode = "provided_waypoints"`` reproduces the supplied
  starter exactly (encoder odometry, 23 fixed waypoints, bounded safety turn);
* team system: ``localization.mode = "mcl"`` and
  ``controller.planning_mode = "astar"`` localizes with the particle filter,
  plans each leg with A*, follows it, avoids unmapped obstacles, and replans.

Team-system state machine::

    localize --converged--> plan --ok--> follow --goal reached--> plan ... --> complete
        ^                    |  no path -> no_path (retry, then forget obstacles)
        |                    ^
    relocalize <--kidnap detected (any state)       follow --blocked/jump--> plan
"""

from __future__ import annotations

import json
import math
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path

from localization import StudentLocalizer
from navigation import ReactiveAvoider, SafetyLayer, WaypointFollower, unicycle_to_wheels
from odometry import DifferentialDriveOdometry, Pose2D
from planning import GlobalPlanner
from sensing import RobustScanFilter, SensorDegrader


PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUN_OVERRIDES_PATH = PROJECT_ROOT / "config" / "run_overrides.json"


def deep_merge(base: dict, overrides: dict) -> dict:
    merged = deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = deepcopy(value)
    return merged


def load_project_config(overrides_path: Path | None = RUN_OVERRIDES_PATH) -> dict:
    """Load the project config and apply ``config/run_overrides.json`` if present."""
    config = json.loads((PROJECT_ROOT / "config" / "project_config.json").read_text(encoding="utf-8"))
    if overrides_path is not None and Path(overrides_path).is_file():
        overrides = json.loads(Path(overrides_path).read_text(encoding="utf-8"))
        config = deep_merge(config, overrides)
    return config


DEFAULT_MISSION_SETTINGS: dict = {
    "goal_tolerance_m": 0.08,
    "localize_turn_speed_rad_s": 0.5,
    "localize_min_time_s": 3.0,
    "localize_timeout_s": 60.0,
    "relocalize_timeout_s": 6.0,
    "replan_min_interval_s": 0.5,
    "blocked_persist_updates": 3,
    "jump_replan_m": 0.25,
    "no_path_retry_s": 2.0,
    "no_path_forget_after": 3,
}


@dataclass
class StepResult:
    left_speed: float
    right_speed: float
    linear_speed: float
    angular_speed: float
    estimate: Pose2D
    telemetry: dict = field(default_factory=dict)


class MissionController:
    def __init__(
        self,
        config: dict,
        waypoints_data: dict,
        occupancy_grid,
        time_step_s: float,
        seed: int | None = None,
    ) -> None:
        self.config = config
        self.grid = occupancy_grid
        self.dt = time_step_s
        robot = config["robot"]
        controller = config["controller"]
        localization = config["localization"]
        experiment = config.get("experiment", {})
        self.seed = int(experiment.get("seed", 185) if seed is None else seed)
        self.scenario = str(experiment.get("scenario", "nominal"))
        self.robot_config = robot
        self.controller_config = controller
        self.localization_mode = str(localization.get("mode", "odometry_baseline"))
        self.planning_mode = str(controller.get("planning_mode", "provided_waypoints"))
        self.mission_settings = dict(DEFAULT_MISSION_SETTINGS)
        self.mission_settings.update(config.get("mission", {}))
        self.lidar_offset = float(robot.get("lidar_offset_x_m", -0.0095))
        # Extension: degraded-sensor condition and robust sensing.
        self.degrader = SensorDegrader(config.get("sensor_degradation"), self.seed)
        self.scan_filter = RobustScanFilter(config.get("sensing"))
        self.scan_held = False

        initial_pose = tuple(config["coordinate_frame"]["initial_pose"])
        self.odometry = DifferentialDriveOdometry(
            robot["wheel_radius_m"], robot["effective_track_width_m"], initial_pose
        )
        localizer_settings = dict(localization)
        localizer_settings["lidar_offset_x_m"] = self.lidar_offset
        self.localizer = StudentLocalizer(
            initial_pose,
            int(localization.get("particle_count_recommendation", 300)),
            settings=localizer_settings,
            seed=self.seed,
        )
        self.localizer.initialize(occupancy_grid)

        self.follower = WaypointFollower(
            robot["cruise_speed_m_s"],
            controller["waypoint_slowdown_distance_m"],
            controller["minimum_drive_speed_m_s"],
            controller["rotate_enter_heading_error_rad"],
            controller["rotate_exit_heading_error_rad"],
            controller["heading_gain"],
            controller["maximum_angular_speed_rad_s"],
        )
        self.safety = SafetyLayer(
            controller["emergency_stop_distance_m"],
            controller["emergency_release_distance_m"],
            controller["avoidance_distance_m"],
            controller["emergency_turn_speed_rad_s"],
            controller["emergency_minimum_turn_steps"],
            controller["emergency_clear_steps"],
            controller["escape_forward_speed_m_s"],
            controller["escape_forward_steps"],
            controller["maximum_recoveries_per_waypoint"],
        )
        avoidance = config.get("avoidance", {})
        self.avoider = ReactiveAvoider(
            robot["body_radius_m"],
            stop_gap=float(avoidance.get("stop_gap_m", 0.035)),
            slow_gap=float(avoidance.get("slow_gap_m", 0.16)),
            side_margin=float(avoidance.get("side_margin_m", 0.025)),
            side_gain=float(avoidance.get("side_gain", 4.0)),
            turn_speed=float(avoidance.get("turn_speed_rad_s", 0.6)),
            replan_after_steps=int(avoidance.get("replan_after_steps", 12)),
        )
        self.planner = (
            GlobalPlanner(occupancy_grid, robot["body_radius_m"], config.get("planning", {}))
            if self.planning_mode == "astar"
            else None
        )

        self.base_detection_distance = (
            float(self.planner.settings["dynamic_detection_distance_m"]) if self.planner is not None else 0.0
        )

        self.route = [tuple(point) for point in waypoints_data["baseline_waypoints"]]
        goal_lookup = {goal["name"]: goal["position"] for goal in waypoints_data["delivery_goals"]}
        goal_lookup["HOME"] = waypoints_data["home"]["position"]
        order = waypoints_data.get("required_goal_order", ["A", "B", "C", "HOME"])
        self.goals = [{"name": name, "position": goal_lookup[name]} for name in order]

        self.step_count = 0
        self.time_s = 0.0
        self.active_waypoint = 0
        self.mission_goal_index = 0
        self.mission_complete = False
        self.controller_state = "localize" if self.localization_mode == "mcl" else "plan"
        self.state_time = 0.0
        self.path: list[tuple[float, float]] = []
        self.path_index = 0
        self.need_plan = True
        self.last_plan_time = -math.inf
        self.blocked_updates = 0
        self.no_path_failures = 0
        self.replan_count = 0
        self.last_replan_reason = ""
        self.last_plan_status = ""
        self.minimum_front = math.inf
        self.handled_recovery_events = 0
        self.tentative_from: int | None = None
        self.was_confident = self.localization_mode != "mcl"

    @property
    def mode_label(self) -> str:
        robust = "+robust" if self.scan_filter.enabled else ""
        degradation = f"|degr={self.degrader.settings['name']}" if self.degrader.enabled else ""
        return f"{self.localization_mode}+{self.planning_mode}{robust}|{self.scenario}{degradation}|seed={self.seed}"

    # ------------------------------------------------------------------
    def step(
        self,
        left_encoder: float,
        right_encoder: float,
        scan_points: list[tuple[float, float]],
        free_ray_bearings: list[float] | None = None,
    ) -> StepResult:
        self.step_count += 1
        self.time_s += self.dt
        odometry_pose, delta_motion = self.odometry.update(left_encoder, right_encoder)
        scan_points, free_ray_bearings = self.degrader.apply(scan_points, free_ray_bearings)
        localizer_scan = scan_points
        self.scan_held = False
        if self.scan_filter.enabled:
            scan_points, free_ray_bearings, self.scan_held = self.scan_filter.process(
                scan_points, free_ray_bearings, odometry_pose
            )
            # A held (predicted) scan is for the avoider only, never evidence.
            localizer_scan = [] if self.scan_held else scan_points
            # Median-of-3 leaves about 0.67 of the raw Gaussian noise std.
            filtered_noise = 0.67 * self.scan_filter.noise_estimate
            self.localizer.set_range_noise(filtered_noise)
            if self.planner is not None:
                # Wall returns scattered by noise must not become obstacles.
                self.planner.settings["dynamic_detection_distance_m"] = max(
                    self.base_detection_distance, 3.0 * filtered_noise
                )
        estimate = self.localizer.update(odometry_pose, delta_motion, localizer_scan, self.grid)

        self.free_ray_bearings = free_ray_bearings
        if self.planning_mode == "astar":
            linear, angular, safety_state = self._team_step(estimate, scan_points)
        else:
            linear, angular, safety_state = self._baseline_step(estimate, scan_points)

        left, right = unicycle_to_wheels(
            linear,
            angular,
            self.robot_config["wheel_radius_m"],
            self.robot_config["effective_track_width_m"],
            self.robot_config["max_wheel_speed_rad_s"],
        )
        loc = self.localizer
        telemetry = {
            "time": self.time_s,
            "estimate": list(estimate.as_tuple()),
            "active_waypoint": self.active_waypoint,
            "mission_goal_index": self.mission_goal_index,
            "minimum_lidar_range": self.minimum_front if math.isfinite(self.minimum_front) else None,
            "mission_complete": self.mission_complete,
            "mode": self.mode_label,
            "commanded_linear_speed": linear,
            "commanded_angular_speed": angular,
            "left_wheel_speed": left,
            "right_wheel_speed": right,
            "lidar_point_count": len(scan_points),
            "controller_state": self.controller_state,
            "safety_state": safety_state,
            "escape_turn_direction": (
                self.avoider.turn_direction if self.planning_mode == "astar" else self.safety.turn_direction
            ),
            "distance_to_waypoint": self.follower.last_distance,
            "heading_error_to_waypoint": self.follower.last_heading_error,
            "particle_count": len(loc.particles),
            "particle_position_spread": loc.position_spread,
            "particle_heading_spread": loc.heading_spread,
            "effective_sample_size": loc.effective_sample_size,
            "lidar_inlier_ratio": loc.last_inlier_ratio,
            "localization_converged": loc.converged,
            "localization_recovering": loc.recovering,
            "recovery_events": loc.recovery_events,
            "replan_count": self.replan_count,
            "last_replan_reason": self.last_replan_reason,
            "scan_held": self.scan_held,
            "dropped_scans": self.degrader.dropped_scans,
            "estimated_range_noise": self.scan_filter.noise_estimate,
        }
        return StepResult(left, right, linear, angular, estimate, telemetry)

    # ------------------------------------------------------------------
    # Baseline: the supplied starter logic, unchanged.
    # ------------------------------------------------------------------
    def _baseline_step(self, estimate: Pose2D, scan_points) -> tuple[float, float, str]:
        tolerance = self.controller_config["waypoint_tolerance_m"]
        previous = self.active_waypoint
        while self.active_waypoint < len(self.route) and self.follower.distance_to(
            estimate, self.route[self.active_waypoint]
        ) <= tolerance:
            self.active_waypoint += 1
        if self.active_waypoint != previous:
            self.safety.reset_for_waypoint()
        if self.mission_goal_index < len(self.goals):
            goal = tuple(self.goals[self.mission_goal_index]["position"])
            if self.follower.distance_to(estimate, goal) <= self.config["evaluation"]["position_goal_tolerance_m"]:
                self.mission_goal_index += 1

        if self.active_waypoint >= len(self.route):
            self.mission_complete = True
            self.controller_state = "complete"
            self.safety.state = "inactive"
            self.safety.turn_direction = 0.0
            self.follower.last_distance = 0.0
            self.follower.last_heading_error = 0.0
            self.minimum_front = min((math.hypot(x, y) for x, y in scan_points), default=math.inf)
            return 0.0, 0.0, self.safety.state
        linear, angular = self.follower.command(estimate, self.route[self.active_waypoint])
        linear, angular, self.minimum_front = self.safety.filter_command(linear, angular, scan_points)
        self.controller_state = self.follower.state
        return linear, angular, self.safety.state

    # ------------------------------------------------------------------
    # Team system: MCL + A* + avoidance + replanning
    # ------------------------------------------------------------------
    def _set_state(self, state: str) -> None:
        if state != self.controller_state:
            self.controller_state = state
            self.state_time = 0.0

    def _request_plan(self, reason: str) -> None:
        self.need_plan = True
        self.last_replan_reason = reason

    def _team_step(self, estimate: Pose2D, scan_points) -> tuple[float, float, str]:
        settings = self.mission_settings
        loc = self.localizer
        planner = self.planner
        self.state_time += self.dt
        self.minimum_front = min(
            (math.hypot(x, y) for x, y in scan_points if x > 0.0 and abs(math.atan2(y, x)) < 0.55),
            default=math.inf,
        )

        if self.mission_complete:
            self._set_state("complete")
            return 0.0, 0.0, "inactive"

        confident = self.localization_mode != "mcl" or (loc.converged and not loc.recovering)

        # 1. Localization health: stop and turn in place once per kidnap event.
        if loc.recovery_events > self.handled_recovery_events:
            self.handled_recovery_events = loc.recovery_events
            self._set_state("relocalize")
            self.path = []
            planner.clear_dynamic()
            self.avoider.reset()
        if self.controller_state in ("localize", "relocalize"):
            ready = confident and self.state_time >= settings["localize_min_time_s"]
            timeout_key = "localize_timeout_s" if self.controller_state == "localize" else "relocalize_timeout_s"
            # On timeout the robot drives on the best hypothesis; the filter
            # keeps every hypothesis until the corridor disambiguates them.
            if ready or self.state_time >= settings[timeout_key]:
                self._request_plan("localized" if ready else "localize_timeout")
                self._set_state("plan")
            else:
                return 0.0, float(settings["localize_turn_speed_rad_s"]), "localizing"

        # 2. Large estimate correction: obstacle cells were registered in a
        #    wrong frame, and the current path starts from the wrong place.
        if loc.updated_this_step and loc.last_jump > settings["jump_replan_m"]:
            planner.clear_dynamic()
            self._request_plan("pose_jump")

        # 3. Update the dynamic obstacle layer from LiDAR.
        if loc.updated_this_step and confident and not self.scan_held:
            # Only with a trustworthy pose; otherwise walls become "obstacles".
            planner.update_dynamic(estimate, scan_points, self.lidar_offset, self.free_ray_bearings)

        # 4. Goal bookkeeping.  With several pose hypotheses (congruent rooms)
        #    an arrival is only tentative: the robot continues to the next goal,
        #    which always crosses the corridor where the hypotheses separate.
        #    Once localization is confident again, the mission resumes from the
        #    first tentative goal so that no delivery is skipped.
        if confident and not self.was_confident and self.tentative_from is not None:
            self.mission_goal_index = self.tentative_from
            self.active_waypoint = self.mission_goal_index
            self.tentative_from = None
            self.path = []
            self._request_plan("resume_after_relocalization")
        self.was_confident = confident
        goal = tuple(self.goals[self.mission_goal_index]["position"])
        final_goal = self.mission_goal_index == len(self.goals) - 1
        if self.follower.distance_to(estimate, goal) <= settings["goal_tolerance_m"] and (
            confident or not final_goal
        ):
            if not confident and self.tentative_from is None:
                self.tentative_from = self.mission_goal_index
            self.mission_goal_index += 1
            self.active_waypoint = self.mission_goal_index
            self.path = []
            if self.mission_goal_index >= len(self.goals):
                self.mission_complete = True
                self._set_state("complete")
                self.follower.last_distance = 0.0
                self.follower.last_heading_error = 0.0
                return 0.0, 0.0, "inactive"
            goal = tuple(self.goals[self.mission_goal_index]["position"])
            self._request_plan("next_goal")
            self.last_plan_time = -math.inf

        if not confident and final_goal and self.follower.distance_to(estimate, goal) <= settings["goal_tolerance_m"]:
            # Never finish on an ambiguous pose: re-cross the corridor.
            if self.tentative_from is None:
                self.tentative_from = self.mission_goal_index
            self.mission_goal_index = max(0, self.mission_goal_index - 1)
            self.active_waypoint = self.mission_goal_index
            goal = tuple(self.goals[self.mission_goal_index]["position"])
            self.path = []
            self._request_plan("disambiguate")
            self.last_plan_time = -math.inf

        # 5. (Re)plan.
        if self.need_plan and self.time_s - self.last_plan_time >= settings["replan_min_interval_s"]:
            self._plan(estimate, goal)
        if self.controller_state == "no_path":
            if self.state_time >= settings["no_path_retry_s"]:
                if self.no_path_failures >= settings["no_path_forget_after"]:
                    planner.clear_dynamic()
                self._request_plan("no_path_retry")
                self.state_time = 0.0
            return 0.0, 0.0, "no_path"
        if not self.path:
            return 0.0, 0.0, "waiting_for_plan"

        # 6. Follow the path.
        tolerance = self.controller_config["waypoint_tolerance_m"]
        while self.path_index < len(self.path) - 1 and self.follower.distance_to(
            estimate, self.path[self.path_index]
        ) <= tolerance:
            self.path_index += 1

        if loc.updated_this_step:
            if planner.path_blocked((estimate.x, estimate.y), self.path[self.path_index:]):
                self.blocked_updates += 1
                if self.blocked_updates >= settings["blocked_persist_updates"]:
                    self.blocked_updates = 0
                    self._request_plan("path_blocked")
            else:
                self.blocked_updates = 0

        linear, angular = self.follower.command(estimate, self.path[self.path_index])
        linear, angular, _, replan = self.avoider.filter_command(linear, angular, scan_points)
        if replan:
            planner.add_obstacle_points(estimate, self.avoider.blocking_points, self.lidar_offset)
            self._request_plan("local_blockage")
        self._set_state("follow_" + self.follower.state)
        return linear, angular, self.avoider.state

    def _plan(self, estimate: Pose2D, goal: tuple[float, float]) -> None:
        self.last_plan_time = self.time_s
        result = self.planner.plan((estimate.x, estimate.y), goal)
        self.last_plan_status = result.status
        self.need_plan = False
        self.replan_count += 1
        if result.status == "ok":
            self.path = result.waypoints[1:] if len(result.waypoints) > 1 else list(result.waypoints)
            self.path_index = 0
            self.no_path_failures = 0
            self.blocked_updates = 0
            self._set_state("follow_rotate")
        else:
            self.path = []
            self.no_path_failures += 1
            self._set_state("no_path")
            print(
                f"[planner] no valid path to {goal} from "
                f"({estimate.x:.2f}, {estimate.y:.2f}): {result.status}"
            )

    # Visualization helpers ---------------------------------------------
    def display_path(self) -> list[tuple[float, float]]:
        if self.planning_mode == "astar":
            return list(self.path)
        return self.route

    def display_waypoint_index(self) -> int:
        if self.planning_mode == "astar":
            return self.path_index
        return min(self.active_waypoint, max(0, len(self.route) - 1))
