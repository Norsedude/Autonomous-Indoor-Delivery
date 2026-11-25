# Group 5 (Tron): implementation notes and progress

Team: Mateo Ortega, Zihao He, Devin Roberson. Status as of 25 November 2026.

## Where the project stands against our proposal schedule

| Proposal date | Milestone | Status |
|---|---|---|
| Oct 25 | World runs, odometry, LiDAR, display, first CSV log (Checkpoint 1) | Done in Webots. The untouched baseline completes the route but misses C by 0.23 m (see Webots results); confirm with the instructor. |
| Oct 25 | Research on Dijkstra/A* and robustness testing | A* implemented (`planning.py`). |
| Nov 8 | Particle belief and LiDAR measurement update (Checkpoint 2) | Done: `localization.py`. |
| Nov 15 | Planned path, path following, avoidance, replanning (Checkpoint 3) | Done: `planning.py`, `navigation.py`, `mission.py`. Verified in Webots (40 runs). |
| Nov 22 | Robustness extension, data sets, analysis | Done: `sensing.py`, 50 Webots runs, `results/webots/summary.md`. |
| Nov 29 | Report and slides | In progress. Figures in `results/webots/figures/`; demo video steps in `docs/demo_video.md`. |

## Code map

| File | Role |
|---|---|
| `controllers/project_starter/project_starter.py` | Webots device glue only: encoders, LiDAR, motors, Display, telemetry. |
| `controllers/project_starter/mission.py` | `MissionController`: all decision logic for both systems. Shared by Webots and the offline simulator. |
| `controllers/project_starter/localization.py` | Monte Carlo Localization, global scan-matching search, multi-hypothesis recovery. |
| `controllers/project_starter/planning.py` | A*, the clearance-inflated planning grid, the LiDAR dynamic-obstacle layer, line-of-sight simplification. |
| `controllers/project_starter/navigation.py` | Provided waypoint follower and safety layer (unchanged), plus `ReactiveAvoider`. |
| `controllers/project_starter/sensing.py` | Extension: `SensorDegrader` (experimental condition) and `RobustScanFilter` (mitigation). |
| `config/project_config.json` | Default run: team system, nominal scenario. All tuning parameters. |
| `config/experiments.json` | Methods (`baseline`, `team`, `team_robust`), the four required scenarios (plus one stress test), degradation levels, and seeds 1-5. |
| `tools/set_run_config.py` | Picks method/scenario/seed/degradation for the next Webots run (`config/run_overrides.json`). |
| `tools/run_webots_batch.py` | Runs Webots headless for many method/scenario/degradation/seed combinations and collects the logs. |
| `tools/summarize_webots.py` | Builds `results/webots/summary.md` (aggregate and per-trial tables). |
| `tools/simulate_navigation.py` | Offline closed-loop simulator of the full controller with noise, blocker motion and kidnapping. |
| `tests/test_navigation_stack.py` | A* required cases, planner, MCL, kidnap detection, avoidance, closed-loop smoke test. |

## Methods (for the report)

### Localization: Monte Carlo Localization (`localization.py`)

- **Initial belief.** Nominal: 300 particles, Gaussian around the configured start, sigma 0.45 m / 0.50 rad, rejecting occupied cells. Uncertain start: uniform over the whole HOME room with unknown heading, realized by the global scan-matching search restricted to that room.
- **Motion model.** Particles are moved by the odometry increment since the last update, expressed in the robot frame, plus Gaussian noise. Translation std = 0.08 |d| + 0.01 |dtheta| + 2 mm; rotation std = 0.08 |dtheta| + 0.25 |d| + 0.004 rad.
- **Measurement model.** Likelihood field on a 5 cm grid: `p = z_hit exp(-d^2 / 2 sigma^2) + z_rand`, with sigma = 0.05 m and z_rand = 0.05. Uses 40 of the 360 beams, includes the 9.5 mm LiDAR offset, and tempers the summed log-likelihood by 10/40 because neighbouring beams are correlated. Map walls are fattened by up to half a cell during rasterization, so the field treats each occupied cell as a quarter-cell square to balance that bias.
- **Weights and resampling.** Log-space normalization. Effective sample size `N_eff = 1 / sum w^2`. Low-variance resampling only when N_eff < N/2.
- **Estimate.** Weighted mean of the dominant cluster. Heading is a circular mean. The estimate sticks to the current cluster unless another is 1.5x heavier.
- **Updates.** A filter update runs after 2 cm or 0.05 rad of motion (or every 12 steps). Between updates the estimate is propagated with odometry.
- **Kidnap detection.** The fraction of beams within 8 cm of a mapped wall (the inlier ratio) is tracked. Correct tracking has a median of 0.98 and dips to about 0.62 next to the unmapped blocker. Recovery starts after 2 updates below 0.25, 4 updates below 0.45, or a jump of more than 0.5 m in one update.
- **Recovery (global localization).** A brute-force scan-matching search scores every free 10 cm cell at 36 headings (about 0.4 s). Each distinct good match becomes its own 100-particle filter. Hypotheses compete through decaying evidence `e_k = 0.85 e_(k-1) + T (L_best - L_h)` and are pruned at e > 6. The decay stops tiny constant biases (map rasterization) from accumulating into a wrong decision, while real disambiguating evidence prunes a wrong hypothesis within a few updates. A single surviving hypothesis is accepted when it is tight (std < 0.10 m / 0.12 rad) and its 10-update mean inlier ratio is at least 0.90.

### Planning (`planning.py`)

- A* on a 0.05 m grid. 8-connected, octile heuristic. Diagonal moves are rejected when either adjacent cardinal cell is occupied (no corner cutting). Returns `[]` when no path exists; the mission reports `no_path`, waits, retries, and after 3 failures forgets the dynamic obstacles.
- **Inflation.** A cell is free when its clearance to the static map is at least robot radius 0.037 m + margin 0.035 m. LiDAR obstacles get another 0.02 m. A cost term `1 + 3 max(0, (0.20 - clearance) / 0.20)` pushes paths toward corridor centres.
- **Cells to waypoints.** Cell centres become metric points. A line-of-sight pass then shortcuts them, accepting a shortcut only if it keeps at least min(0.12 m, the clearance of the cells it replaces).
- **Dynamic layer.** LiDAR returns more than 0.10 m from any mapped wall (within 2 m) add hits to 5 cm cells. Beams passing through a cell remove hits, including beams with no return (their bearings are recovered from the point-cloud order). Cells with at least 3 hits are obstacles.
- **Replanning triggers.** A new goal; the remaining path (next 1.5 m) blocked by a dynamic obstacle for 3 consecutive filter updates; a pose correction above 0.25 m; a local-avoidance request; recovery after a kidnap; and periodic retries after `no_path`.

### Path following and avoidance (`navigation.py`, `mission.py`)

- The provided rotate-then-drive `WaypointFollower` follows the A* waypoints.
- `ReactiveAvoider` replaces the starter safety turn in the team system:
  - slows down when the swept corridor ahead has a return closer than 0.16 m;
  - stops translating and turns toward the more open side below 0.035 m;
  - pushes away from returns closer than 2.5 cm beside the body;
  - if a stop lasts 12 steps, inserts the blocking returns into the map and requests a replan.

  Stopping is therefore never permanent.
- **Local/global interaction.** The global planner owns the route. The local layer only reacts to what the planner does not know yet, and every persistent local event is pushed back to the planner as map information.

### Mission logic (`mission.py`)

States: `localize` → `plan` → `follow_*` → next goal ... → `complete`, plus `relocalize` (turn in place 6 s after a kidnap), `no_path`, and `waiting_for_plan`.

- Goals count only when localization is confident.
- While several hypotheses remain, an arrival counts as tentative and the robot continues; every leg of this mission crosses the corridor, which separates the hypotheses.
- If localization becomes confident again after a tentative arrival, the mission resumes from the first tentative goal.

## Important finding: the map is symmetric

All four rooms are congruent, including the door positions. With the blocker at position 1 (the origin), the whole world is 180° rotationally symmetric about the origin. Consequences:

- From inside a room, a scan matches four poses equally well. The corridor views reduce this to two: pose P and its 180° rotation about the origin.
- With the blocker at position 1, **those two poses produce identical LiDAR scans everywhere**. After a kidnap, no sensor-based method can tell them apart. Our filter keeps both hypotheses, follows the one closer to the last confident estimate (a disturbance is assumed to be local), and, after 4 m of driving with no further evidence, commits to it so the mission does not stall.
- In the offline kidnapped trials (kidnap to (0.9, -2.1, pi) at t = 150 s, robot between A and B), the closer hypothesis is the true one, so recovery succeeds. Kidnapping at t ≈ 60–100 s (robot on the west side) makes the twin the closer one, and the run fails. This is a fundamental observability limit, not a tuning problem. It belongs in the failure analysis.
- An unmapped obstacle cannot break the tie either, because the measurement model scores scans against the static map only.

## Webots results (15 Nov 2026, headless batch, evaluation Supervisor logs)

Produced by `python tools/run_webots_batch.py --method baseline --method team --all-scenarios --seeds 1 2 3 4 5`. CSVs, console output, and `runs.jsonl` (health metrics) are in `results/webots/`. The scripted `scenario_driver` performs the `2`/`K` key actions at fixed times in a generated copy of the world; the official world and the evaluation Supervisor are untouched.

| Scenario | Baseline (health PASS / goals) | Team (health PASS / goals) |
|---|---|---|
| Nominal | 0/5, 2 of 4 goals (misses C by 0.23 m, odometry drift) | 5/5, 4 goals, 363-396 s, RMSE 0.025-0.037 m |
| Uncertain start | 0/5, 0 goals | 5/5, 4 goals |
| Blocked route (blocker to position 2 at 20 s) | 0/5, 0 goals (safety layer latches "blocked") | 5/5, 4 goals |
| Kidnapped (K at 150 s) | 0/5, 1 goal | 2/5 (seeds 2, 4); seeds 1, 3, 5 committed to the 180° twin |

No collisions in any of the 40 runs. Baseline runs are identical across seeds because Webots physics and the baseline are deterministic.

**Checkpoint 1 note.** The untouched baseline completes all 23 waypoints and returns within 0.13 m of HOME. Its odometry error grows to about 0.17 m, so the ground truth passes 0.23 m from C (tolerance 0.20 m), and `check_run_health.py --require-complete` reports FAIL: "ground truth recorded 2/4 ordered goals". See `results/webots/baseline/nominal/seed1_plots.png`. This is the drift the assignment asks us to explain; confirm with the instructor whether a near-miss baseline is acceptable for Checkpoint 1.

## Untested blocker placements (25 Nov 2026, Webots, 5 seeds each)

The final demonstration may use an unreleased obstacle placement, so we ran two placements that no earlier experiment used. No code or parameter was changed for them.

| Scenario | Placement | Baseline | Team | Team robust |
|---|---|---|---|---|
| `blocker_position_3` | Supervisor test position 3, (2.0, -1.3), on the B -> C leg, moved at 20 s | 0/5 (2 goals, then stalls) | 5/5, 359 s, 0 collisions | 5/5, 359 s, 0 collisions |
| `blocked_corridor` | (-2.3, 0.0), directly on the team's HOME -> A path, moved at 15 s | 0/5 (0 goals) | 5/5, 363 s, 0 collisions | 5/5, 364 s, 0 collisions |

Evidence that the robot actually avoided the obstacle (seed 1, closest approach of the robot to the blocker centre): the nominal team path passes 0.13 m from the corridor spot and 0.26 m from position 3; with the blocker there, the robot detected it from LiDAR, replanned, and kept 0.39 m and 0.53 m. Figure: `results/webots/figures/success_unseen_placements.png`.

Rerun: `python tools/run_webots_batch.py --method baseline --method team --method team_robust --scenario blocker_position_3 --scenario blocked_corridor --seeds 1 2 3 4 5`.

## Offline results so far

`python tools/simulate_navigation.py --method team --all-scenarios --seeds 1 2 3 4 5 --output-dir results/offline_sim/team` (same with `--method baseline`). See `results/offline_sim/summary.md`, generated by `python tools/summarize_offline.py`. These are simulator results, **not** Webots results; the report must use Webots runs logged by the evaluation Supervisor.

## How to run each scenario in Webots

1. `python tools/set_run_config.py --method team --scenario nominal --seed 1` (or `--method baseline` / `team_robust`, optionally `--degradation moderate|severe`). The tool prints what to do in Webots for that scenario.
2. In Webots: File > Reload World (or Ctrl+Shift+T to reset), then run. Keep the 3D view focused for the Supervisor keys (`2` moves the blocker, `K` kidnaps, `N` starts a new log).
3. When the robot stops at HOME: `python tools/check_run_health.py results/run_<stamp>.csv --require-complete` (A* runs need 4 goals instead of 23 waypoints; the tool detects this from the log), then `python tools/analyze_run.py results/run_<stamp>.csv`.
4. Repeat for seeds 1-5 and each scenario. Keep every CSV, including failures.

The baseline is expected to fail the uncertain-start, blocked-route and kidnapped scenarios. That contrast is the required comparison.

## Extension: robustness under degraded sensors (proposal hypothesis)

**Hypothesis (proposal):** a prediction and denoising algorithm improves the robot's performance with degraded sensors.

**Degradation (experimental condition, `SensorDegrader`).** Injected in software in the robot controller, before any processing. It is seeded from the trial seed and identical for every method. Webots' LiDAR has no dropout option, so state this in the report.

| Level | Scan dropout | Beam dropout | Extra range noise | Spurious short returns |
|---|---|---|---|---|
| moderate | 2 % chance per step, 8-step bursts (about 14 % of scans) | 10 % | 0.02 m | 2 % |
| severe | 4 % chance per step, 15-step bursts (about 38 % of scans) | 30 % | 0.05 m | 5 % |

**Mitigation (`team_robust`, `RobustScanFilter`):**
1. *Prediction:* during a dropout, the last good scan is re-projected with odometry and given to the local avoider for up to 20 steps; the localizer never sees it.
2. *Denoising:* median of three adjacent ranges.
3. *Noise estimation:* `sigma = 1.4826 MAD(e) / sqrt(1.5)` from second differences of adjacent ranges. The localizer widens its likelihood field to `sqrt(sigma_hit^2 + (0.67 sigma)^2)` and its inlier gate to `3 sigma`, and the obstacle-detection gate becomes `max(0.10, 3 sigma)` so scattered wall returns are not inserted as obstacles.
4. *Free-space evidence:* only runs of at least 3 adjacent no-return beams clear obstacle cells.

**Webots results (5 seeds each; full table in `results/webots/summary.md`):**

| Scenario | Degradation | team | team_robust |
|---|---|---|---|
| Nominal | none | 5/5, 377 s | 5/5, 367 s |
| Nominal | moderate | 5/5, 418 ± 82 s | 5/5, 369 ± 15 s |
| Nominal | severe | **0/5** (stalls after A; 21 collisions) | **4/5**, 448 s, 0 collisions |
| Blocked route | none | 5/5, 359 s | 5/5, 359 s |
| Blocked route | moderate | 5/5, 379 s | 5/5, 360 s |
| Blocked route | severe | 4/5 (7 collisions in one run) | **5/5**, 364 s, 0 collisions |

The hypothesis is supported: the mitigation does not cost anything without degradation, makes moderate-degradation runs faster and more consistent, and turns severe degradation from 4/10 into 9/10 successes with zero collisions. Failure case: `team_robust` nominal severe seed 1 stalls after goal A (localization is fine, RMSE 0.047 m; the planner is blocked, most likely by obstacle cells inserted from noisy returns in the A room's doorway). Success is scored on the mission criteria; the health checker's "LiDAR telemetry >= 80 %" sanity rule is excluded because the degraded runs drop scans on purpose.

Rerun: `python tools/run_webots_batch.py --method team --method team_robust --scenario nominal --scenario blocked_route --degradation none --degradation moderate --degradation severe --seeds 1 2 3 4 5`, then `python tools/summarize_webots.py`.

## Things to verify in Webots (not testable offline)

- Real contact dynamics and the LiDAR noise model versus the simulator.
- The free-ray bearing recovery assumes the point cloud is in scan order (left to right). `tests/test_navigation_stack.py` checks both scan directions.
- Controller speed: the global search takes about 0.4 s and slows the simulation briefly after a kidnap. That is expected.
