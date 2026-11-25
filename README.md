# CMPE 185 Autonomous Indoor Delivery Project

Group 5 (Tron): Mateo Ortega, Zihao He, Devin Roberson.

The team system (LiDAR Monte Carlo Localization, A* planning, reactive
avoidance, and replanning) is implemented on top of the provided starter. See
[docs/team_progress.md](docs/team_progress.md) for methods, status, and the
symmetry finding. The provided baseline is still available for the required
comparison. The degraded-sensor extension (`sensing.py`, method `team_robust`) and all
Webots results are summarized in `results/webots/summary.md`.
To record the demonstration video, follow [docs/demo_video.md](docs/demo_video.md).

## Windows setup (team machines)

1. Install Webots R2025a and Python 3.10 or newer (python.org installer, with
   "Add python.exe to PATH" ticked). Check with `python --version`.
2. In Webots: Tools > Preferences > General > Python command = `python` (or
   the full path, for example `C:\Python314\python.exe`).
3. Optional, for plots: `python -m pip install matplotlib`.
4. From the project root in PowerShell (use `python`, not `python3`):

   ```powershell
   python tools/validate_project.py
   python -m unittest discover -s tests -v
   python tools/simulate_baseline.py
   ```

5. Do not save the world from Webots (Ctrl+S) unless you mean to change it.
   Saving rewrites default-valued fields and the camera pose, and the
   validator then fails on text it no longer finds.

## Choosing what runs in Webots

`config/project_config.json` runs the team system in the nominal scenario by
default. To select a method, scenario, and seed for the next run:

```powershell
python tools/set_run_config.py --method baseline --scenario nominal --seed 1
python tools/set_run_config.py --method team --scenario kidnapped --seed 3
python tools/set_run_config.py --method team_robust --scenario nominal --degradation severe   # extension
python tools/set_run_config.py --clear     # back to project_config.json
```

Then reload or reset the world in Webots. The tool prints the keyboard action
each scenario needs (for example `2` or `K`). The Supervisor logs the method,
scenario, and seed in the `mode` column of every CSV.

Offline closed-loop simulation of the same controller code (fast, no Webots):

```powershell
python tools/simulate_navigation.py --method team --scenario nominal --seed 1 --verbose
python tools/simulate_navigation.py --method team --all-scenarios --seeds 1 2 3 4 5 --output-dir results/offline_sim/team
python tools/summarize_offline.py
```

## Reproducing the reported results

Every number in `results/webots/summary.md` comes from an evaluation-Supervisor
CSV committed under `results/webots/` (one per method, scenario, degradation
level, and seed; seeds 1-5). The run settings are in `config/experiments.json`
and each run's metrics are appended to `results/webots/runs.jsonl` (the
latest record per combination is used).

```bash
# 1. Run the experiments headless (about 1.5-5 min of wall time per run)
python tools/run_webots_batch.py --method baseline --method team --method team_robust --all-scenarios --seeds 1 2 3 4 5
python tools/run_webots_batch.py --method team --method team_robust --scenario nominal --scenario blocked_route --degradation moderate --degradation severe --seeds 1 2 3 4 5
# 2. Tables (success, time, path, collisions, RMSE, convergence and recovery time)
python tools/summarize_webots.py
# 3. Report figures -> results/webots/figures/ (needs matplotlib)
python tools/plot_webots_summary.py
```

On Windows, `run_webots_batch.py` finds Webots in `C:\Program Files\Webots`;
elsewhere pass `--webots PATH`. On a Linux machine without a display, wrap the
command in `xvfb-run -a` (for example
`xvfb-run -a python3 tools/run_webots_batch.py --webots /opt/webots/webots ...`).
The scripted `scenario_driver` performs the `2` and `K` actions at fixed
times in a generated copy of the world (`worlds/_batch.wbt`, not committed);
the official world and the evaluation Supervisor are not modified.

Recovery metrics (`tools/check_run_health.py`): a kidnap is detected as a
ground-truth jump of more than 0.25 m between log rows. Recovery time is the
time from the jump until the estimate is within 0.20 m and 0.35 rad of ground
truth and stays there for 5 s. Convergence time is the same measure from
the start of the run.

## Software

- Webots R2025a
- Python 3 using the interpreter configured by Webots
- No third-party package is needed to run the controllers
- Matplotlib is optional and used only by the offline plotting tools

## First run

1. Extract the complete ZIP; 
2. From the project root, run the three offline checks:

   ```bash
   python3 tools/validate_project.py
   python3 -m unittest discover -s tests -v
   python3 tools/simulate_baseline.py
   ```

3. For the untouched starter baseline described in steps 4-6 (Checkpoint 1),
   first run `python tools/set_run_config.py --method baseline --scenario nominal`.
   Without it, the team system runs.
   Open `worlds/cmpe185_indoor_delivery.wbt` in Webots R2025a. On first load,
   allow Webots network access so it can download the version-pinned official
   e-puck PROTO and its assets.
4. Reset the simulation, leave the blocker at position 1, and run without
   pressing `1`, `2`, `3`, `K`, or `N`.
5. The robot should follow HOME → A → B → C → HOME and stop. The evaluation
   Supervisor writes a CSV under `results/`.
6. Check the completed run:

   ```bash
   python3 tools/check_run_health.py results/run_YYYYMMDD_HHMMSS.csv --require-complete
   python3 tools/analyze_run.py results/run_YYYYMMDD_HHMMSS.csv
   ```

The deterministic controller-in-loop reference completes 23/23 waypoints in
531.9 s, travels 33.37 m, reaches all four ordered ground-truth goals, returns
within 0.074 m of HOME, and keeps at least 0.084 m body clearance. A Webots run
may differ slightly because of contact dynamics, but it must complete the same
route without a collision or permanent emergency turn. See
`docs/baseline_validation.md` and `reference/expected_baseline_route.png`.

The starter controller uses encoder odometry and the provided waypoints. It
does not use GPS, Supervisor pose, or the evaluation log. Its initial pose is
configured only to make the starter demonstration reproducible; teams replace
this assumption for uncertain-start and kidnapped-robot tests.

## Correct drive model

The supplied Webots R2025a e-puck2 model uses:

- wheel radius: `0.0200 m`
- geometric wheel-center spacing: `0.0520 m`
- calibrated effective track width for odometry/control: `0.0574 m`
- collision-body radius: `0.037 m`
- command limit: `6.28 rad/s` (below the model's motor limit)
- baseline cruise speed: `0.09 m/s`

Both encoder odometry and inverse kinematics use the same effective track
width. The larger effective value accounts for the Webots contact model during
turns; substituting the geometric axle length causes heading drift. Details and
equations are in `docs/drive_model_and_expected_behavior.md`.

## Coordinate convention

The project and Webots world share the ENU `x-y` floor plane:

- `x` points right on the supplied map.
- `y` points upward on the supplied map.
- `z` is vertical.
- `theta = 0` points along `+x`; positive heading is counterclockwise about `+z`.
- Robot and LiDAR data use FLU: local `+x` forward, `+y` left, `+z` up.

LiDAR point-cloud `(x, y)` is already `(x_forward, y_left)`. Do not swap or
negate these components.

## Package contents

- `worlds/`: corrected Webots world
- `controllers/project_starter/`: sensing, odometry, visualization, conservative
  waypoint baseline, recovery state machine, and student implementation hooks
- `controllers/evaluation_supervisor/`: public ground-truth logger
- `maps/`: occupancy grid, metadata, mission goals, and nominal waypoints
- `config/`: robot, controller, and evaluation parameters
- `reference/`: canonical expected trace, summary, and trajectory figures
- `docs/`: assignment, baseline validation, drive model, and project guidance
- `tools/`: validation, simulation, health checking, plotting, and drive calibration
- `tests/`: frame, map, odometry, controller, safety, and full-route checks

Required student work lives in `localization.py`, `planning.py`, and
`navigation.py` (now implemented), integrated by `mission.py`. `mapping.py`
remains available for a mapping/SLAM extension. Do not modify the public
Supervisor for evaluated runs.

For A* runs, `active_waypoint` in the log counts completed goals (0-4).
`tools/check_run_health.py` detects this from the `mode` column and then
requires 4 instead of 23 (override with `--expected-waypoints`).

## Evaluation controls

With the Webots 3D window focused:

- `1`, `2`, `3`: move the unexpected obstacle
- `K`: apply the kidnapped-robot pose change
- `N`: begin a new CSV log
- `H`: print the controls

The supplied starter is expected to complete only the untouched nominal setup
(blocker position 1). Positions 2 and 3 and `K` are deliberate student tests.


