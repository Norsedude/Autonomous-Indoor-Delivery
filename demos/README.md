# Demonstration clips

Recorded in Webots R2025a on 25 Nov 2026 with the evaluation Supervisor logging
every run. Times are simulation time (the Webots clock); the videos are sped
up. The matching evaluation logs are `results/run_20261125_*.csv` on the
recording machine (not committed; the committed Webots experiment logs are in
`results/webots/`). Full recording procedure: `docs/demo_video.md`.

`Test 1 Normal Speed.mp4` (238 MB, over GitHub's 100 MB file limit) is kept
locally only; `Test 1 Increased Speed.mp4` is the same first nominal run sped up.

| File | Scenario | Run configuration | What happened in the recorded run |
|---|---|---|---|
| `Clip A Blocked Move 2.mp4` | Blocked route | `mcl+astar, blocked_route, seed 1` | Blocker moved at 0:20; goals at 1:08, 2:58, 4:03; HOME at about 5:52; 0 collisions |
| `Clip B Robot Kidnapped.mp4` | Kidnapped robot | `mcl+astar, kidnapped, seed 2` | Kidnapped at 2:30; relocalized 58 s later; goals at 1:11, 5:05, 6:11; HOME at 8:03; 0 collisions |
| `Clip C Unseen Blocker.mp4` | Untested blocker placement (position 3) | `mcl+astar, blocker_position_3, seed 1` | Goals at 1:08, 2:52, 3:57; HOME at 5:52; 0 collisions |
| `Clip D Baseline.mp4` | Provided odometry-only baseline | `odometry_baseline, provided waypoints, seed 1` | Blocker was not moved in this take. Reached A (1:30) and B (4:22), finished its 23-waypoint route at 8:47, but drift made it pass 0.23 m from C (tolerance 0.20 m), so 2/4 goals |
| `Clip E Extension.mp4` | Extension, severe LiDAR degradation | `mcl+astar+robust, nominal, severe, seed 2` | Goals at 1:09, 3:24, 4:30; HOME at 6:47; 0 collisions |
| `Test 1 Increased Speed.mp4` | First nominal run | `mcl+astar, nominal, seed 1` | Goals at 1:08, 3:27, 4:33; HOME at 6:25; 0 collisions |

## Speaker notes, one per slide

### Clip A: Full Delivery Mission with a Blocked Route

What it is: the full mission, HOME to A to B to C and back to HOME, with an
obstacle moved onto the route after the robot has already planned.

What it shows: the whole team pipeline working together. Monte Carlo
Localization (300 particles, LiDAR likelihood field) keeps the pose estimate
on the map; A* plans on an inflated grid; when the LiDAR sees the unmapped
block, it is added to the planning map (red squares on the display) and A*
replans around it.

Accomplishes: Checkpoint 3 (path following, avoidance, replanning) in one run.
Recorded run: goals at 1:08, 2:58, 4:03, HOME at about 5:52, 0 collisions.
Over 5 seeds: 5/5 success, 0 collisions; the baseline gets 0/5.

How to run:
1. VS Code terminal: `python tools/set_run_config.py --method team --scenario blocked_route --seed 1`
2. Webots: File menu, Reload World; console shows `mcl+astar|blocked_route|seed=1`
3. Press Run; at 0:20 click the 3D view and press 2
4. Expect: the block jumps into the west corridor, the blue path reroutes,
   mission complete at about 5:52

### Clip B: Kidnapped Robot Recovery

What it is: while driving from A toward B, the Supervisor teleports the
robot to the south-east room without telling it.

What it shows: kidnap detection and global relocalization. The fraction of
LiDAR beams matching the map collapses, so the filter runs a global
scan-matching search, keeps each good match as its own particle-filter
hypothesis, and accepts one once it is tight and consistent. On the display,
the purple particles spread out, the robot turns in place (`relocalize`),
then the cyan scan lines up with the walls again.

Accomplishes: the "recover after a pose disturbance" requirement and the
recovery-time metric. Recorded run: kidnapped at 2:30, relocalized 58 s later,
then B at 5:05, C at 6:11, HOME at 8:03. Over 5 seeds: 2/5. The map is
180-degree rotationally symmetric, so two poses can give identical scans;
in 3 trials the filter committed to the mirror pose (failure analysis).

How to run:
1. VS Code terminal: `python tools/set_run_config.py --method team --scenario kidnapped --seed 2`
2. Webots: File menu, Reload World; console shows `mcl+astar|kidnapped|seed=2`
3. Press Run; after goal A, once the robot has crossed the middle corridor
   into the right half of the map (about 2:28 to 2:30), click the 3D view
   and press K
4. Expect: `relocalize`, about 1 minute to recover, mission complete at about 8:05.
   If it heads to the wrong room it chose the mirror pose: reload and retake

### Clip C: Untested Obstacle Placement

What it is: the obstacle moved to Supervisor test position 3, on the route
from B to C. No earlier experiment used this placement and nothing was tuned
for it.

What it shows: the system generalizes. The robot detects the block with
LiDAR after leaving B and steers around it with the same code and parameters.

Accomplishes: readiness for the "unreleased obstacle placement" in the final
demonstration. Recorded run: goals at 1:08, 2:52, 3:57, HOME at 5:52,
0 collisions. Over 5 seeds: 5/5; a second untested placement on the
HOME to A corridor also gave 5/5. Baseline: 0/5 on both.

How to run:
1. VS Code terminal: `python tools/set_run_config.py --method team --scenario blocker_position_3 --seed 1`
2. Webots: File menu, Reload World; console shows `mcl+astar|blocker_position_3|seed=1`
3. Press Run; at 0:20 click the 3D view and press 3
4. Expect: avoidance after B (around 4:00), mission complete at about 5:52

### Clip D: Provided Baseline (Odometry Only)

What it is: the supplied starter controller: wheel-encoder odometry, a fixed
list of 23 waypoints, and a simple safety stop. No LiDAR localization and no
planner.

What it shows: why localization matters. With no correction, odometry error
grows along the route. The robot reaches A and B and finishes its route, but
passes 0.23 m from C (goal tolerance 0.20 m), so only 2 of 4 goals count.

Accomplishes: the required comparison, encoder odometry baseline versus
LiDAR-corrected localization and navigation. Baseline over 5 seeds: nominal
0/5 (always misses C), uncertain start 0/5, blocked route 0/5, kidnapped 0/5.
Team: 5/5, 5/5, 5/5, 2/5. Position RMSE in nominal: 0.107 m baseline versus
0.030 m team.

How to run:
1. VS Code terminal: `python tools/set_run_config.py --method baseline --scenario nominal --seed 1`
2. Webots: File menu, Reload World; console shows `odometry_baseline+provided_waypoints|nominal|seed=1`
3. Press Run, no keys
4. Expect: route finished at about 8:47 with goals A and B only.
   Optional blocked version: use `--scenario blocked_route` and press 2 at
   0:20 (click the 3D view first); the baseline stalls at the block

### Clip E: Extension, Robustness under Degraded LiDAR

What it is: our team extension. Severe degradation is injected into the LiDAR
before any processing: about 38% of scans lost in bursts, 30% of beams
dropped, extra range noise, 5% false short returns. Seeded and identical for
every method.

What it shows: the robust sensing layer. It predicts the scan during dropouts
by moving the last good scan with odometry, median-filters the ranges,
estimates the noise online and widens the localization model to match, and
only clears free space when several neighbouring beams agree. On the display
the cyan scan flickers and disappears, yet the mission completes.

Accomplishes: tests the proposal hypothesis that prediction and denoising
improve performance with degraded sensors. Recorded run: goals at 1:09,
3:24, 4:30, HOME at 6:47, 0 collisions. Over 5 seeds, severe, nominal route:
team without the extension 0/5 (stalls after A), with it 4/5, 0 collisions;
blocked route severe: 4/5 versus 5/5. No cost without degradation.

How to run:
1. VS Code terminal: `python tools/set_run_config.py --method team_robust --scenario nominal --seed 2 --degradation severe`
2. Webots: File menu, Reload World; console shows `mcl+astar+robust|nominal|degr=severe|seed=2`
3. Press Run, no keys
4. Expect: mission complete at about 6:47. For contrast, the same command
   with `--method team` stalls after goal A

When finished with any clip: `python tools/set_run_config.py --clear`
