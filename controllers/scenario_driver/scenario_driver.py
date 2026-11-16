"""Scripted scenario driver for unattended (batch) Webots experiments.

This is NOT the evaluation Supervisor and does not log anything.  It runs in
a separate Supervisor robot that only exists in the generated batch world
(``worlds/_batch.wbt``) and reproduces the evaluation Supervisor's keyboard
perturbations at fixed times from ``config/experiments.json``:

* ``start_offset``   - relocate the robot before it moves (uncertain start)
* ``blocker_events`` - same as pressing 1/2/3 (moves DYNAMIC_BLOCK)
* ``kidnap_time_s``  - same as pressing K (same pose, same Webots calls)

It listens to the robot telemetry (channel 7) only to end the run: it quits
Webots a few seconds after the robot reports mission completion, or at
``max_time_s``.  It never sends anything to the robot controller.

controllerArgs: [scenario name, max simulated seconds]
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from controller import Supervisor

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "controllers" / "project_starter"))

from coordinate_frames import webots_pose_from_project  # noqa: E402


def main() -> None:
    scenario = sys.argv[1] if len(sys.argv) > 1 else "nominal"
    max_time = float(sys.argv[2]) if len(sys.argv) > 2 else 900.0
    experiments = json.loads((PROJECT_ROOT / "config" / "experiments.json").read_text(encoding="utf-8"))
    simulation = experiments["scenarios"][scenario].get("simulation", {})
    config = json.loads((PROJECT_ROOT / "config" / "project_config.json").read_text(encoding="utf-8"))

    supervisor = Supervisor()
    step = int(supervisor.getBasicTimeStep())
    robot = supervisor.getFromDef("STUDENT_ROBOT")
    block = supervisor.getFromDef("DYNAMIC_BLOCK")
    receiver = supervisor.getDevice("driver receiver")
    receiver.enable(step)

    def place_robot(x: float, y: float, theta: float) -> None:
        translation, rotation = webots_pose_from_project(x, y, theta)
        robot.getField("translation").setSFVec3f(translation)
        robot.getField("rotation").setSFRotation(rotation)
        robot.resetPhysics()

    offset = simulation.get("start_offset")
    if offset:
        x0, y0, theta0 = config["coordinate_frame"]["initial_pose"]
        place_robot(x0 + offset[0], y0 + offset[1], theta0 + offset[2])
        print(f"[driver] uncertain start: robot placed with offset {offset}")

    events = sorted(simulation.get("blocker_events", []), key=lambda event: event[0])
    kidnap_time = simulation.get("kidnap_time_s")
    kidnap_pose = simulation.get("kidnap_pose", [0.9, -2.1, math.pi])
    kidnapped = False
    complete_since = None

    while supervisor.step(step) != -1:
        now = supervisor.getTime()
        while events and now >= events[0][0]:
            _, position = events.pop(0)
            block.getField("translation").setSFVec3f([position[0], position[1], 0.18])
            print(f"[driver] t={now:.1f}s moved blocker to {position}")
        if kidnap_time is not None and not kidnapped and now >= kidnap_time:
            place_robot(*kidnap_pose)
            kidnapped = True
            print(f"[driver] t={now:.1f}s kidnapped robot to {kidnap_pose}")
        while receiver.getQueueLength() > 0:
            try:
                telemetry = json.loads(receiver.getString())
                if telemetry.get("mission_complete") and complete_since is None:
                    complete_since = now
            except (ValueError, TypeError):
                pass
            receiver.nextPacket()
        if (complete_since is not None and now - complete_since >= 3.0) or now >= max_time:
            print(f"[driver] t={now:.1f}s ending run (complete={complete_since is not None})")
            supervisor.simulationQuit(0)
            break


if __name__ == "__main__":
    main()
