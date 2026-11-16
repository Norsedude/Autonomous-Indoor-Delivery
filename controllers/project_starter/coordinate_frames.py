"""Coordinate conversions shared by the robot controller and Supervisor.

The world uses Webots ENU: global +x is east/right, +y is north/up on the
map, and +z is vertical.  Webots R2022a and later use FLU for robot devices:
local +x is forward, +y is left, and +z is up.  The project planar frame is
therefore the Webots global x-y plane with no sign or axis swap.
"""

from __future__ import annotations

import math
from collections.abc import Sequence


def lidar_point_to_robot_xy(point_x: float, point_y: float, point_z: float) -> tuple[float, float] | None:
    """Return a finite LiDAR point as ``(forward, left)`` in robot FLU.

    ``point_z`` is accepted to make the input convention explicit; a planar
    controller intentionally discards it.
    """
    _ = point_z
    if not math.isfinite(point_x) or not math.isfinite(point_y):
        return None
    return float(point_x), float(point_y)


def project_pose_from_webots(
    position: Sequence[float],
    orientation: Sequence[float],
) -> tuple[float, float, float]:
    """Convert a Webots global pose to project ``(x, y, theta)``.

    Webots returns the orientation as a row-major 3 x 3 rotation matrix.  The
    first column is the robot's local +x (forward) axis expressed globally.
    """
    if len(position) < 2 or len(orientation) != 9:
        raise ValueError("Expected a 3-vector position and 3 x 3 orientation matrix")
    theta = math.atan2(float(orientation[3]), float(orientation[0]))
    return float(position[0]), float(position[1]), theta


def webots_pose_from_project(
    x: float,
    y: float,
    theta: float,
    z: float = 0.0,
) -> tuple[list[float], list[float]]:
    """Return Webots translation and z-axis rotation for a planar pose."""
    return [float(x), float(y), float(z)], [0.0, 0.0, 1.0, float(theta)]


def is_non_floor_contact(
    contacted_node_id: int,
    floor_node_id: int,
    contact_z: float,
    floor_height_tolerance: float,
) -> bool:
    """Classify a contact while robustly excluding the horizontal floor."""
    return (
        int(contacted_node_id) != int(floor_node_id)
        and abs(float(contact_z)) > float(floor_height_tolerance)
    )


def free_ray_bearings(cloud_xy: list[tuple[float, float] | None]) -> list[float]:
    """Bearings (robot FLU, radians) of LiDAR samples that returned nothing.

    ``cloud_xy`` is the full point cloud in scan order with ``None`` for
    samples without a finite return.  The bearing of a missing sample is
    extrapolated from the nearest valid sample using the constant angular
    step of the scan, so the result does not depend on whether the device
    scans clockwise or counterclockwise.
    """
    count = len(cloud_xy)
    valid = [(index, math.atan2(point[1], point[0])) for index, point in enumerate(cloud_xy) if point is not None]
    if count == 0 or len(valid) < 2 or len(valid) == count:
        return []
    step = 2.0 * math.pi / count
    # Determine the scan direction from consecutive valid samples.
    votes = 0.0
    for (i0, a0), (i1, a1) in zip(valid, valid[1:]):
        if i1 - i0 == 1:
            votes += math.remainder(a1 - a0, 2.0 * math.pi)
    if votes < 0.0:
        step = -step
    bearings = []
    pointer = 0
    for index, point in enumerate(cloud_xy):
        if point is not None:
            continue
        while pointer + 1 < len(valid) and abs(valid[pointer + 1][0] - index) <= abs(valid[pointer][0] - index):
            pointer += 1
        reference_index, reference_angle = valid[pointer]
        bearings.append(math.remainder(reference_angle + (index - reference_index) * step, 2.0 * math.pi))
    return bearings
