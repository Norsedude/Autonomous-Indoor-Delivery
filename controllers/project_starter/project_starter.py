"""Webots controller for the CMPE 185 autonomous-delivery project.

This file only handles Webots devices: it reads the wheel encoders and the
LiDAR point cloud, passes them to ``MissionController`` (``mission.py``), sets
the wheel speeds, draws the Display, and sends telemetry to the evaluation
Supervisor.  It never accesses Webots ground truth (no GPS, no Supervisor API).

Which system runs is selected in ``config/project_config.json`` and can be
overridden per run with ``config/run_overrides.json`` (written by
``tools/set_run_config.py``):

* baseline  - encoder odometry + the 23 provided waypoints (the starter)
* team      - LiDAR Monte Carlo Localization + A* + avoidance + replanning
"""

from __future__ import annotations

import json
from pathlib import Path

from controller import Robot

from coordinate_frames import free_ray_bearings, lidar_point_to_robot_xy
from mission import MissionController, load_project_config
from occupancy_grid import OccupancyGrid
from visualization import DisplayVisualizer


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def lidar_points_in_robot_frame(lidar) -> tuple[list[tuple[float, float]], list[float]]:
    """Return FLU ``(forward, left)`` returns and the bearings of empty beams."""
    cloud = [lidar_point_to_robot_xy(point.x, point.y, point.z) for point in lidar.getPointCloud()]
    converted = [point for point in cloud if point is not None]
    return converted, free_ray_bearings(cloud)


def main() -> None:
    config = load_project_config()
    waypoints_data = load_json(PROJECT_ROOT / "maps" / "waypoints.json")
    grid = OccupancyGrid.from_files(PROJECT_ROOT / "maps" / "building_map.yaml")

    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_encoder = robot.getDevice("left wheel sensor")
    right_encoder = robot.getDevice("right wheel sensor")
    lidar = robot.getDevice("lidar")
    display = robot.getDevice("student display")
    emitter = robot.getDevice("emitter")

    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    motor_acceleration = float(config["robot"].get("motor_acceleration_rad_s2", -1.0))
    if motor_acceleration > 0.0:
        left_motor.setAcceleration(motor_acceleration)
        right_motor.setAcceleration(motor_acceleration)
    left_encoder.enable(time_step)
    right_encoder.enable(time_step)
    lidar.enable(time_step)
    lidar.enablePointCloud()

    robot_config = config["robot"]
    controller_config = config["controller"]
    mission = MissionController(config, waypoints_data, grid, time_step / 1000.0)
    visualizer = DisplayVisualizer(display, grid)

    print("CMPE 185 autonomous indoor delivery, release 4.0.0 + Group 5 (Tron) navigation stack")
    print(f"Run configuration: {mission.mode_label}")
    print(
        "Drive parameters: "
        f"wheel radius={robot_config['wheel_radius_m']:.4f} m, "
        f"effective track={robot_config['effective_track_width_m']:.4f} m"
    )
    print("Frames: Webots ENU world (x-y floor, z up); robot/LiDAR FLU (+x forward, +y left)")
    print("Ground truth is recorded only by the evaluation Supervisor.")

    step_count = 0
    while robot.step(time_step) != -1:
        step_count += 1
        left_encoder_value = left_encoder.getValue()
        right_encoder_value = right_encoder.getValue()
        scan_points, empty_bearings = lidar_points_in_robot_frame(lidar)
        result = mission.step(left_encoder_value, right_encoder_value, scan_points, empty_bearings)

        left_motor.setVelocity(result.left_speed)
        right_motor.setVelocity(result.right_speed)

        if step_count % controller_config["display_update_steps"] == 0:
            visualizer.draw(
                result.estimate,
                scan_points,
                mission.display_path(),
                mission.goals,
                mission.display_waypoint_index(),
                mission.mode_label.split("|")[0],
                mission.controller_state,
                result.telemetry["safety_state"],
                particles=mission.localizer.sample_particles(150),
                obstacles=mission.planner.dynamic_points() if mission.planner else None,
                status=(
                    f"goal {min(mission.mission_goal_index + 1, len(mission.goals))}/{len(mission.goals)}"
                    f"  particles {len(mission.localizer.particles)}"
                    f"  inliers {mission.localizer.last_inlier_ratio:.2f}"
                ),
            )

        if step_count % controller_config["telemetry_period_steps"] == 0:
            telemetry = dict(result.telemetry)
            telemetry["time"] = robot.getTime()
            telemetry["left_encoder_rad"] = left_encoder_value
            telemetry["right_encoder_rad"] = right_encoder_value
            emitter.send(json.dumps(telemetry).encode("utf-8"))


if __name__ == "__main__":
    main()
