"""Grid planning for the CMPE 185 project.

* ``astar`` is a generic 8-connected A* search on any grid object that offers
  ``in_bounds(row, col)`` and ``occupied(row, col)`` (for example the
  supplied ``OccupancyGrid`` or ``OccupancyGrid.inflated(...)``).  Diagonal
  moves that would cut an occupied corner are rejected.  An optional
  ``traversal_cost(row, col)`` multiplier (>= 1) keeps the octile heuristic
  admissible.
* ``PlanningGrid`` is the inflated grid used on the robot: a 0.05 m grid whose
  cells are free only when their clearance to the static map *and* to
  LiDAR-detected unmapped obstacles is at least robot radius + margin.  Cells
  with small clearance cost more, which pushes paths toward corridor centres.
* ``GlobalPlanner`` plans from the estimated pose to a metric goal, converts
  cells to metric waypoints, simplifies them with a clearance-preserving
  line-of-sight pass, and tests whether the remaining path has become blocked.
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from dataclasses import dataclass, field
from math import hypot

from occupancy_grid import distance_field


SQRT2 = math.sqrt(2.0)
NEIGHBORS = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, SQRT2),
    (-1, 1, SQRT2),
    (1, -1, SQRT2),
    (1, 1, SQRT2),
)


def reconstruct_path(came_from: dict, current: tuple[int, int]) -> list[tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def octile_distance(a: tuple[int, int], b: tuple[int, int]) -> float:
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)


def astar(
    occupancy_grid,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    """Return a collision-free 8-connected cell path, or ``[]`` if none exists."""
    grid = occupancy_grid
    for cell in (start, goal):
        if not grid.in_bounds(*cell) or grid.occupied(*cell):
            return []
    if start == goal:
        return [start]
    cost_fn = getattr(grid, "traversal_cost", None)
    counter = 0
    open_heap: list = [(octile_distance(start, goal), 0.0, counter, start)]
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    closed: set[tuple[int, int]] = set()
    while open_heap:
        _, current_g, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return reconstruct_path(came_from, current)
        closed.add(current)
        row, col = current
        for d_row, d_col, step in NEIGHBORS:
            n_row = row + d_row
            n_col = col + d_col
            neighbor = (n_row, n_col)
            if neighbor in closed:
                continue
            if not grid.in_bounds(n_row, n_col) or grid.occupied(n_row, n_col):
                continue
            if d_row and d_col and (grid.occupied(row + d_row, col) or grid.occupied(row, col + d_col)):
                continue
            tentative = current_g + step * (cost_fn(n_row, n_col) if cost_fn else 1.0)
            if tentative < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative
                came_from[neighbor] = current
                counter += 1
                heapq.heappush(
                    open_heap,
                    (tentative + octile_distance(neighbor, goal), tentative, counter, neighbor),
                )
    return []


def path_length(points: list[tuple[float, float]]) -> float:
    return sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def simplify_world_path(path: list[tuple[float, float]], minimum_spacing: float = 0.15) -> list[tuple[float, float]]:
    """Remove waypoints that are much closer than the controller needs."""
    if not path:
        return []
    simplified = [path[0]]
    for point in path[1:-1]:
        if hypot(point[0] - simplified[-1][0], point[1] - simplified[-1][1]) >= minimum_spacing:
            simplified.append(point)
    if len(path) > 1:
        simplified.append(path[-1])
    return simplified


class PlanningGrid:
    """Clearance-inflated planning grid with a LiDAR dynamic-obstacle layer.

    Rows count from the top and columns from the left, like ``OccupancyGrid``.
    """

    def __init__(
        self,
        occupancy_grid,
        resolution: float = 0.05,
        robot_radius: float = 0.037,
        safety_margin: float = 0.035,
        preferred_clearance: float = 0.20,
        clearance_weight: float = 3.0,
        max_distance: float = 0.6,
        dynamic_extra_margin: float = 0.02,
    ) -> None:
        self.resolution = resolution
        # Unmapped obstacles are only partly observed (their far side is
        # unknown), so they get an additional margin.
        self.dynamic_extra_margin = dynamic_extra_margin
        self.origin_x = occupancy_grid.origin_x
        self.origin_y = occupancy_grid.origin_y
        self.required_clearance = robot_radius + safety_margin
        self.preferred_clearance = max(preferred_clearance, self.required_clearance + 1e-6)
        self.clearance_weight = clearance_weight
        self.max_distance = max_distance
        # Use the full map cell as the obstacle footprint: conservative.
        self.static_clearance, self.nx, self.ny = distance_field(
            occupancy_grid, resolution, 0.5 * occupancy_grid.resolution, max_distance
        )
        self.dynamic_clearance = [max_distance] * (self.nx * self.ny)
        self.dynamic_cells: set[int] = set()

    # Grid interface ----------------------------------------------------
    @property
    def width(self) -> int:
        return self.nx

    @property
    def height(self) -> int:
        return self.ny

    def index(self, row: int, col: int) -> int:
        return (self.ny - 1 - row) * self.nx + col

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.ny and 0 <= col < self.nx

    def clearance(self, row: int, col: int) -> float:
        index = self.index(row, col)
        static = self.static_clearance[index]
        dynamic = self.dynamic_clearance[index] - self.dynamic_extra_margin
        return static if static < dynamic else dynamic

    def occupied(self, row: int, col: int) -> bool:
        return not self.in_bounds(row, col) or self.clearance(row, col) < self.required_clearance

    def traversal_cost(self, row: int, col: int) -> float:
        deficit = self.preferred_clearance - self.clearance(row, col)
        if deficit <= 0.0:
            return 1.0
        return 1.0 + self.clearance_weight * deficit / self.preferred_clearance

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row_from_bottom = int(math.floor((y - self.origin_y) / self.resolution))
        return self.ny - 1 - row_from_bottom, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (self.ny - 1 - row + 0.5) * self.resolution,
        )

    def flat_index_world(self, x: float, y: float) -> int | None:
        fx = (x - self.origin_x) / self.resolution
        fy = (y - self.origin_y) / self.resolution
        if fx < 0.0 or fy < 0.0 or fx >= self.nx or fy >= self.ny:
            return None
        return int(fy) * self.nx + int(fx)

    def clearance_at(self, x: float, y: float) -> float:
        index = self.flat_index_world(x, y)
        if index is None:
            return 0.0
        static = self.static_clearance[index]
        dynamic = self.dynamic_clearance[index] - self.dynamic_extra_margin
        return static if static < dynamic else dynamic

    def dynamic_clearance_at(self, x: float, y: float) -> float:
        index = self.flat_index_world(x, y)
        return self.max_distance if index is None else self.dynamic_clearance[index]

    def set_dynamic_cells(self, cells: set[int]) -> None:
        """Replace the dynamic obstacle cells and recompute their clearance."""
        self.dynamic_cells = set(cells)
        values = [self.max_distance] * (self.nx * self.ny)
        resolution = self.resolution
        window = int(math.ceil(self.max_distance / resolution))
        for index in self.dynamic_cells:
            j0, i0 = divmod(index, self.nx)
            for j in range(max(0, j0 - window), min(self.ny, j0 + window + 1)):
                dy = max(0.0, (abs(j - j0) - 0.5) * resolution)
                base = j * self.nx
                for i in range(max(0, i0 - window), min(self.nx, i0 + window + 1)):
                    dx = max(0.0, (abs(i - i0) - 0.5) * resolution)
                    distance = (dx * dx + dy * dy) ** 0.5
                    if distance < values[base + i]:
                        values[base + i] = distance
        self.dynamic_clearance = values

    def nearest_free(self, cell: tuple[int, int], max_radius_m: float = 0.5) -> tuple[int, int] | None:
        if self.in_bounds(*cell) and not self.occupied(*cell):
            return cell
        limit = int(math.ceil(max_radius_m / self.resolution))
        queue = deque([cell])
        seen = {cell}
        while queue:
            row, col = queue.popleft()
            if abs(row - cell[0]) > limit or abs(col - cell[1]) > limit:
                continue
            if self.in_bounds(row, col) and not self.occupied(row, col):
                return row, col
            for d_row, d_col, _ in NEIGHBORS:
                neighbor = (row + d_row, col + d_col)
                if neighbor not in seen and self.in_bounds(*neighbor):
                    seen.add(neighbor)
                    queue.append(neighbor)
        return None


@dataclass
class PlanResult:
    status: str
    waypoints: list[tuple[float, float]] = field(default_factory=list)
    cells: list[tuple[int, int]] = field(default_factory=list)
    length_m: float = 0.0


DEFAULT_PLANNING_SETTINGS: dict = {
    "resolution_m": 0.05,
    "safety_margin_m": 0.035,
    "dynamic_extra_margin_m": 0.02,
    "preferred_clearance_m": 0.20,
    "clearance_weight": 3.0,
    "line_of_sight_clearance_m": 0.12,
    "start_snap_radius_m": 0.5,
    "dynamic_detection_distance_m": 0.10,
    "dynamic_max_range_m": 2.0,
    "dynamic_hit_increment": 2,
    "dynamic_clear_decrement": 1,
    "dynamic_max_hits": 6,
    "dynamic_active_hits": 3,
    "dynamic_clear_margin_m": 0.08,
    "blocked_lookahead_m": 1.5,
    "blocked_clearance_m": 0.05,
}


class GlobalPlanner:
    """A* planning, waypoint generation, and the dynamic obstacle layer."""

    def __init__(self, occupancy_grid, robot_radius: float, settings: dict | None = None) -> None:
        self.settings = dict(DEFAULT_PLANNING_SETTINGS)
        if settings:
            self.settings.update(settings)
        s = self.settings
        self.grid = PlanningGrid(
            occupancy_grid,
            resolution=float(s["resolution_m"]),
            robot_radius=robot_radius,
            safety_margin=float(s["safety_margin_m"]),
            preferred_clearance=float(s["preferred_clearance_m"]),
            clearance_weight=float(s["clearance_weight"]),
            dynamic_extra_margin=float(s["dynamic_extra_margin_m"]),
        )
        self.robot_radius = robot_radius
        self.hits: dict[int, int] = {}
        self.dynamic_dirty = False

    # Planning ----------------------------------------------------------
    def plan(self, start_xy: tuple[float, float], goal_xy: tuple[float, float]) -> PlanResult:
        self.refresh()
        grid = self.grid
        snap = float(self.settings["start_snap_radius_m"])
        start = grid.nearest_free(grid.world_to_cell(*start_xy), snap)
        goal = grid.nearest_free(grid.world_to_cell(*goal_xy), snap)
        if start is None:
            return PlanResult("start_blocked")
        if goal is None:
            return PlanResult("goal_blocked")
        cells = astar(grid, start, goal)
        if not cells:
            return PlanResult("no_path")
        points = [tuple(start_xy)] + [grid.cell_to_world(*cell) for cell in cells[1:-1]] + [tuple(goal_xy)]
        clearances = [grid.clearance_at(*point) for point in points]
        waypoints = self.simplify_line_of_sight(points, clearances)
        return PlanResult("ok", waypoints, cells, path_length(waypoints))

    def segment_clear(self, a: tuple[float, float], b: tuple[float, float], required: float) -> bool:
        length = hypot(b[0] - a[0], b[1] - a[1])
        samples = max(1, int(math.ceil(length / (0.5 * self.grid.resolution))))
        for k in range(samples + 1):
            t = k / samples
            if self.grid.clearance_at(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])) < required:
                return False
        return True

    def simplify_line_of_sight(self, points, clearances) -> list[tuple[float, float]]:
        """Greedy shortcutting that never reduces the clearance of the A* path.

        A shortcut from point i to j is accepted only if every sample keeps at
        least min(line_of_sight_clearance, smallest clearance of the cells it
        replaces).
        """
        if len(points) <= 2:
            return list(points)
        target = float(self.settings["line_of_sight_clearance_m"])
        output = [points[0]]
        i = 0
        while i < len(points) - 1:
            j = i + 1
            span_min = min(clearances[i], clearances[j])
            while j + 1 < len(points):
                candidate_min = min(span_min, clearances[j + 1])
                required = min(target, candidate_min) - 1e-3
                if not self.segment_clear(points[i], points[j + 1], required):
                    break
                j += 1
                span_min = candidate_min
            output.append(points[j])
            i = j
        return output

    def path_blocked(self, robot_xy: tuple[float, float], remaining: list[tuple[float, float]]) -> bool:
        """True if a newly detected obstacle intrudes on the next part of the path."""
        if not remaining:
            return False
        self.refresh()
        lookahead = float(self.settings["blocked_lookahead_m"])
        threshold = float(self.settings["blocked_clearance_m"]) + self.robot_radius
        travelled = 0.0
        previous = robot_xy
        for point in remaining:
            length = hypot(point[0] - previous[0], point[1] - previous[1])
            samples = max(1, int(math.ceil(length / (0.5 * self.grid.resolution))))
            for k in range(1, samples + 1):
                t = k / samples
                x = previous[0] + t * (point[0] - previous[0])
                y = previous[1] + t * (point[1] - previous[1])
                if self.grid.dynamic_clearance_at(x, y) < threshold:
                    return True
                if travelled + t * length >= lookahead:
                    return False
            travelled += length
            previous = point
        return False

    # Dynamic obstacle layer --------------------------------------------
    def refresh(self) -> None:
        if self.dynamic_dirty:
            active = int(self.settings["dynamic_active_hits"])
            self.grid.set_dynamic_cells({index for index, count in self.hits.items() if count >= active})
            self.dynamic_dirty = False

    def clear_dynamic(self) -> None:
        if self.hits:
            self.hits = {}
            self.dynamic_dirty = True

    def _active(self) -> set[int]:
        active = int(self.settings["dynamic_active_hits"])
        return {index for index, count in self.hits.items() if count >= active}

    def update_dynamic(
        self,
        pose,
        scan_points_robot,
        sensor_offset_x: float = -0.0095,
        free_ray_bearings: list[float] | None = None,
    ) -> bool:
        """Insert unmapped LiDAR returns and clear cells that beams pass through.

        ``free_ray_bearings`` are robot-frame bearings of beams that returned
        nothing within range; they clear cells up to ``dynamic_max_range_m``,
        which removes obstacles that have moved away even when the space
        behind them is open.  Returns True when the active cell set changed.
        """
        s = self.settings
        before = self._active()
        c = math.cos(pose.theta)
        sn = math.sin(pose.theta)
        sensor_x = pose.x + c * sensor_offset_x
        sensor_y = pose.y + sn * sensor_offset_x
        max_range = float(s["dynamic_max_range_m"])
        margin = float(s["dynamic_clear_margin_m"])
        decrement = int(s["dynamic_clear_decrement"])
        step = 0.5 * self.grid.resolution
        grid = self.grid

        if self.hits:
            rays = []
            for x_forward, y_left in scan_points_robot[::2]:
                beam_range = hypot(x_forward, y_left)
                rays.append((x_forward / beam_range, y_left / beam_range, min(beam_range, max_range) - margin))
            for bearing in (free_ray_bearings or [])[::2]:
                rays.append((math.cos(bearing), math.sin(bearing), max_range))
            for fx, fy, free_length in rays:
                if free_length <= 0.0:
                    continue
                ux = c * fx - sn * fy
                uy = sn * fx + c * fy
                distance = step
                while distance < free_length:
                    index = grid.flat_index_world(sensor_x + ux * distance, sensor_y + uy * distance)
                    if index is not None and index in self.hits:
                        self.hits[index] -= decrement
                        if self.hits[index] <= 0:
                            del self.hits[index]
                    distance += step

        detection = float(s["dynamic_detection_distance_m"])
        increment = int(s["dynamic_hit_increment"])
        maximum = int(s["dynamic_max_hits"])
        for x_forward, y_left in scan_points_robot:
            if x_forward * x_forward + y_left * y_left > max_range * max_range:
                continue
            world_x = sensor_x + c * x_forward - sn * y_left
            world_y = sensor_y + sn * x_forward + c * y_left
            index = grid.flat_index_world(world_x, world_y)
            if index is None or grid.static_clearance[index] < detection:
                continue
            self.hits[index] = min(maximum, self.hits.get(index, 0) + increment)

        changed = self._active() != before
        if changed:
            self.dynamic_dirty = True
        return changed

    def add_obstacle_points(self, pose, points_robot, sensor_offset_x: float = -0.0095) -> None:
        """Force LiDAR points (robot frame) into the obstacle layer."""
        c = math.cos(pose.theta)
        sn = math.sin(pose.theta)
        sensor_x = pose.x + c * sensor_offset_x
        sensor_y = pose.y + sn * sensor_offset_x
        maximum = int(self.settings["dynamic_max_hits"])
        for x_forward, y_left in points_robot:
            index = self.grid.flat_index_world(
                sensor_x + c * x_forward - sn * y_left, sensor_y + sn * x_forward + c * y_left
            )
            if index is not None:
                self.hits[index] = maximum
        self.dynamic_dirty = True

    def dynamic_points(self) -> list[tuple[float, float]]:
        points = []
        for index in self._active():
            j, i = divmod(index, self.grid.nx)
            points.append(
                (
                    self.grid.origin_x + (i + 0.5) * self.grid.resolution,
                    self.grid.origin_y + (j + 0.5) * self.grid.resolution,
                )
            )
        return points
