"""Waypoint following and the deliberately limited starter safety layer.

The nominal baseline rotates before large heading changes, reduces speed near
each waypoint, and uses a calibrated e-puck drive model.  The safety layer can
turn away and move clear of a dangerously close surface, but it deliberately
does not plan around a blocked route.  Students replace it with the full local
avoidance and replanning method taught later in the course.
"""

from __future__ import annotations

from math import atan2, cos, hypot, inf, sqrt

from odometry import Pose2D, wrap_angle


class WaypointFollower:
    """A conservative waypoint controller in unicycle form.

    A small hysteresis band separates ``rotate`` and ``drive`` modes.  The
    robot therefore finishes a corner before translating through a doorway.
    """

    def __init__(
        self,
        cruise_speed: float,
        slowdown_distance: float,
        minimum_drive_speed: float,
        rotate_enter_heading_error: float,
        rotate_exit_heading_error: float,
        gain_heading: float,
        maximum_angular_speed: float,
    ) -> None:
        self.cruise_speed = cruise_speed
        self.slowdown_distance = max(0.01, slowdown_distance)
        self.minimum_drive_speed = max(0.0, min(minimum_drive_speed, cruise_speed))
        self.rotate_enter_heading_error = rotate_enter_heading_error
        self.rotate_exit_heading_error = rotate_exit_heading_error
        self.gain_heading = gain_heading
        self.maximum_angular_speed = maximum_angular_speed
        self.rotating = False
        self.state = "drive"
        self.last_distance = 0.0
        self.last_heading_error = 0.0

    @staticmethod
    def distance_to(pose: Pose2D, target: tuple[float, float]) -> float:
        return hypot(target[0] - pose.x, target[1] - pose.y)

    def command(self, pose: Pose2D, target: tuple[float, float]) -> tuple[float, float]:
        distance = self.distance_to(pose, target)
        desired_heading = atan2(target[1] - pose.y, target[0] - pose.x)
        heading_error = wrap_angle(desired_heading - pose.theta)
        self.last_distance = distance
        self.last_heading_error = heading_error

        if self.rotating:
            if abs(heading_error) <= self.rotate_exit_heading_error:
                self.rotating = False
        elif abs(heading_error) >= self.rotate_enter_heading_error:
            self.rotating = True

        angular_speed = max(
            -self.maximum_angular_speed,
            min(self.maximum_angular_speed, self.gain_heading * heading_error),
        )
        if self.rotating:
            self.state = "rotate"
            return 0.0, angular_speed

        self.state = "drive"
        alignment = max(0.0, cos(heading_error))
        distance_scale = min(1.0, distance / self.slowdown_distance)
        approach_speed = max(
            self.minimum_drive_speed,
            self.cruise_speed * distance_scale,
        )
        linear_speed = approach_speed * alignment
        return linear_speed, angular_speed


class SafetyLayer:
    """Stateful emergency behavior only; students add full avoidance.

    The selected turn direction is latched while emergency mode is active.
    Release requires both a minimum turn duration and several consecutive clear
    scans, preventing noisy LiDAR samples from reversing the turn every step.
    """

    def __init__(
        self,
        emergency_distance: float,
        emergency_release_distance: float,
        avoidance_distance: float,
        emergency_turn_speed: float,
        minimum_turn_steps: int,
        clear_steps_required: int,
        escape_forward_speed: float,
        escape_forward_steps: int,
        maximum_recoveries_per_waypoint: int,
    ) -> None:
        self.emergency_distance = emergency_distance
        self.emergency_release_distance = emergency_release_distance
        self.avoidance_distance = avoidance_distance
        self.emergency_turn_speed = emergency_turn_speed
        self.minimum_turn_steps = max(1, int(minimum_turn_steps))
        self.clear_steps_required = max(1, int(clear_steps_required))
        self.escape_forward_speed = max(0.0, float(escape_forward_speed))
        self.escape_forward_steps = max(0, int(escape_forward_steps))
        self.maximum_recoveries_per_waypoint = max(
            1, int(maximum_recoveries_per_waypoint)
        )
        self.emergency_active = False
        self.turn_direction = 0.0
        self.turn_steps = 0
        self.clear_steps = 0
        self.escape_steps_remaining = 0
        self.recovery_count = 0
        self.blocked = False
        self.state = "clear"

    def reset_for_waypoint(self) -> None:
        """Reset recovery bookkeeping after genuine route progress."""
        self.emergency_active = False
        self.turn_direction = 0.0
        self.turn_steps = 0
        self.clear_steps = 0
        self.escape_steps_remaining = 0
        self.recovery_count = 0
        self.blocked = False
        self.state = "clear"

    @staticmethod
    def _minimum_range(points: list[tuple[float, float]]) -> float:
        return min((hypot(x, y) for x, y in points), default=inf)

    def _choose_turn_direction(self, scan_points_robot: list[tuple[float, float]]) -> float:
        side_points = [
            (x_forward, y_left)
            for x_forward, y_left in scan_points_robot
            if x_forward > -0.05 and abs(atan2(y_left, x_forward)) < 1.20
        ]
        left_clearance = self._minimum_range([(x, y) for x, y in side_points if y > 0.0])
        right_clearance = self._minimum_range([(x, y) for x, y in side_points if y < 0.0])
        if left_clearance > right_clearance + 0.02:
            return 1.0
        if right_clearance > left_clearance + 0.02:
            return -1.0

        # When both sides look similar, turn away from the front-sector
        # centroid.  A centered obstacle uses a deterministic left turn.
        lateral_sum = sum(y for _, y in side_points)
        return -1.0 if lateral_sum > 0.0 else 1.0

    def filter_command(
        self,
        linear_speed: float,
        angular_speed: float,
        scan_points_robot: list[tuple[float, float]],
    ) -> tuple[float, float, float]:
        front_points = [
            (x_forward, y_left)
            for x_forward, y_left in scan_points_robot
            if x_forward > 0.0 and abs(atan2(y_left, x_forward)) < 0.55
        ]
        minimum_front = self._minimum_range(front_points)

        if self.blocked:
            self.state = "blocked"
            return 0.0, 0.0, minimum_front

        if (
            not self.emergency_active
            and self.escape_steps_remaining == 0
            and minimum_front < self.emergency_distance
        ):
            self.recovery_count += 1
            if self.recovery_count > self.maximum_recoveries_per_waypoint:
                self.blocked = True
                self.turn_direction = 0.0
                self.state = "blocked"
                return 0.0, 0.0, minimum_front
            self.emergency_active = True
            self.turn_direction = self._choose_turn_direction(scan_points_robot)
            self.turn_steps = 0
            self.clear_steps = 0

        if self.emergency_active:
            self.turn_steps += 1
            if minimum_front >= self.emergency_release_distance:
                self.clear_steps += 1
            else:
                self.clear_steps = 0

            can_release = (
                self.turn_steps >= self.minimum_turn_steps
                and self.clear_steps >= self.clear_steps_required
            )
            if not can_release:
                self.state = "emergency_turn"
                return 0.0, self.emergency_turn_speed * self.turn_direction, minimum_front

            self.emergency_active = False
            self.turn_steps = 0
            self.clear_steps = 0
            self.escape_steps_remaining = self.escape_forward_steps

        if self.escape_steps_remaining > 0:
            if minimum_front < self.emergency_distance:
                self.recovery_count += 1
                if self.recovery_count > self.maximum_recoveries_per_waypoint:
                    self.blocked = True
                    self.turn_direction = 0.0
                    self.escape_steps_remaining = 0
                    self.state = "blocked"
                    return 0.0, 0.0, minimum_front
                self.emergency_active = True
                self.turn_direction = self._choose_turn_direction(scan_points_robot)
                self.turn_steps = 0
                self.clear_steps = 0
                self.escape_steps_remaining = 0
                self.state = "emergency_turn"
                return 0.0, self.emergency_turn_speed * self.turn_direction, minimum_front

            self.escape_steps_remaining -= 1
            self.state = "escape_forward"
            return self.escape_forward_speed, 0.0, minimum_front

        self.turn_direction = 0.0

        if minimum_front < self.avoidance_distance:
            self.state = "slowdown"
            linear_speed *= 0.30
        else:
            self.state = "clear"

        # STUDENT TODO: replace this bounded safety recovery with the obstacle
        # avoidance method taught in class, and request replanning when the
        # current global path remains blocked.  The starter intentionally
        # enters ``blocked`` instead of pretending that repeated local turns
        # have solved a global path obstruction.
        return linear_speed, angular_speed, minimum_front


class ReactiveAvoider:
    """Local obstacle avoidance used together with the global A* planner.

    The global planner already routes around every obstacle that the LiDAR
    has registered in the dynamic layer, so this layer only handles what the
    plan does not yet know about:

    * forward gap ``g`` is the free distance between the body front and the
      closest LiDAR return inside the swept corridor ``|y| < r + side_margin``;
    * ``g < slow_gap`` scales the forward speed down linearly;
    * ``g < stop_gap`` stops translation and turns toward the more open side
      (direction latched until clear) and, if this persists for
      ``replan_after_steps`` control steps, asks the mission layer to insert
      the blocking returns into the map and replan;
    * returns closer than ``side_margin`` beside the body add a small turn
      away from that side.

    Stopping is temporary: the replan request guarantees a new route, so the
    robot never waits permanently in front of the obstacle.
    """

    def __init__(
        self,
        body_radius: float,
        stop_gap: float = 0.035,
        slow_gap: float = 0.16,
        side_margin: float = 0.025,
        side_gain: float = 4.0,
        turn_speed: float = 0.6,
        replan_after_steps: int = 12,
    ) -> None:
        self.body_radius = body_radius
        self.stop_gap = stop_gap
        self.slow_gap = max(slow_gap, stop_gap + 1e-3)
        self.side_margin = side_margin
        self.side_gain = side_gain
        self.turn_speed = turn_speed
        self.replan_after_steps = max(1, int(replan_after_steps))
        self.turn_direction = 0.0
        self.stopped_steps = 0
        self.state = "clear"
        self.front_gap = inf
        self.blocking_points: list[tuple[float, float]] = []

    def reset(self) -> None:
        self.turn_direction = 0.0
        self.stopped_steps = 0
        self.state = "clear"

    def filter_command(
        self,
        linear_speed: float,
        angular_speed: float,
        scan_points_robot: list[tuple[float, float]],
    ) -> tuple[float, float, float, bool]:
        """Return ``(v, omega, minimum_front_range, replan_requested)``."""
        radius = self.body_radius
        half_width = radius + self.side_margin
        front_gap = inf
        minimum_front = inf
        left_side = inf
        right_side = inf
        left_open = 0.0
        right_open = 0.0
        blocking: list[tuple[float, float]] = []
        for x_forward, y_left in scan_points_robot:
            distance = hypot(x_forward, y_left)
            bearing = atan2(y_left, x_forward)
            if x_forward > 0.0 and abs(bearing) < 0.55:
                minimum_front = min(minimum_front, distance)
            if 0.35 < abs(bearing) < 1.6:
                if bearing > 0.0:
                    left_open += min(distance, 1.0)
                else:
                    right_open += min(distance, 1.0)
            if x_forward > 0.0 and abs(y_left) < half_width:
                surface = sqrt(max(0.0, radius * radius - y_left * y_left)) if abs(y_left) < radius else 0.0
                gap = x_forward - surface
                if gap < front_gap:
                    front_gap = gap
                if gap < self.slow_gap:
                    blocking.append((x_forward, y_left))
            elif -0.03 < x_forward < 0.10 and abs(y_left) < half_width + 0.03:
                side_clearance = abs(y_left) - radius
                if y_left > 0.0:
                    left_side = min(left_side, side_clearance)
                else:
                    right_side = min(right_side, side_clearance)
        self.front_gap = front_gap
        self.blocking_points = blocking

        if linear_speed <= 1e-6:
            # Rotating in place is always safe for the round e-puck body.
            self.turn_direction = 0.0
            self.stopped_steps = 0
            self.state = "clear"
            return linear_speed, angular_speed, minimum_front, False

        if front_gap < self.stop_gap:
            self.stopped_steps += 1
            if self.turn_direction == 0.0:
                self.turn_direction = 1.0 if left_open >= right_open else -1.0
            self.state = "avoid_turn"
            request = self.stopped_steps >= self.replan_after_steps
            if request:
                self.stopped_steps = 0
            return 0.0, self.turn_speed * self.turn_direction, minimum_front, request

        self.turn_direction = 0.0
        self.stopped_steps = 0
        self.state = "clear"
        if front_gap < self.slow_gap:
            scale = (front_gap - self.stop_gap) / (self.slow_gap - self.stop_gap)
            linear_speed *= max(0.2, min(1.0, scale))
            self.state = "slowdown"
        if left_side < self.side_margin:
            angular_speed -= self.side_gain * (self.side_margin - left_side)
            self.state = "side_clearance"
        if right_side < self.side_margin:
            angular_speed += self.side_gain * (self.side_margin - right_side)
            self.state = "side_clearance"
        return linear_speed, angular_speed, minimum_front, False


def unicycle_to_wheels(
    linear_speed: float,
    angular_speed: float,
    wheel_radius: float,
    axle_length: float,
    max_wheel_speed: float,
) -> tuple[float, float]:
    left = (linear_speed - 0.5 * axle_length * angular_speed) / wheel_radius
    right = (linear_speed + 0.5 * axle_length * angular_speed) / wheel_radius
    scale = max(1.0, abs(left) / max_wheel_speed, abs(right) / max_wheel_speed)
    return left / scale, right / scale
