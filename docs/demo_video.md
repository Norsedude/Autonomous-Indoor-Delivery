# Recording the two-minute demonstration video

The final package needs a two-minute demonstration video. This guide records
it from the Webots GUI with the same controller code and settings as the
logged experiments. Timings below come from our Webots logs
(`results/webots/team/...`), so you know when to act and what to expect.

## 0. Before recording (once)

From the project root in PowerShell:

```powershell
git pull
python tools/validate_project.py
python -m unittest discover -s tests
python tools/set_run_config.py --clear
```

All three must pass. In Webots, Tools > Preferences > General > Python command
must be `python` (or the full path to Python 3.10+).

Do not save the world (Ctrl+S) at any point. Moving the camera is fine, but
saving rewrites the `.wbt` file and the validator then fails.

## 1. Set up the Webots window

1. Open `worlds/cmpe185_indoor_delivery.wbt`.
2. Set a top-down camera so all four rooms are visible (use the viewpoint
   menu or drag the view); zoom until the outer walls fill the frame.
3. The robot's **student display** shows the map, the A* path, the live
   LiDAR points, the estimated pose (arrow), the goals, and a status block
   (`mode`, `waypoint`, `control`, `safety`). Enlarge it and place it in a
   corner of the 3D view so the 3D world and the display are both visible.
4. Keep the simulation clock (top toolbar) visible: some steps are timed.

## 2. How to record

Either option works:

- **Webots built-in:** File > Make Movie..., choose 1920x1080 and an
  acceleration of 4-8x, start it, then press Run (real-time, not
  fast-forward; fast mode does not render). Stop recording from the same
  menu when the clip ends. Acceleration keeps a 6-minute mission short.
- **Screen recorder:** Windows Game Bar (Win+Alt+R) or OBS, then speed the
  clip up in Clipchamp (built into Windows 11) or any editor.

Record each clip separately, then cut them together.

## 3. Clips

Before each clip: run the `set_run_config.py` line, then in Webots use
File > Reload World (or Reset Simulation) so the controller reloads the
setting. The console prints the effective run line; check it says the method
and scenario you expect.

### Clip A: full mission with a blocked route (about 45 s at 8x)

Shows localization, A* planning, obstacle avoidance, replanning, and all four
goals in one run.

```powershell
python tools/set_run_config.py --method team --scenario blocked_route --seed 1
```

1. Start recording, press Run.
2. At about **0:20** on the simulation clock, click the 3D view and press `2`
   (moves the red blocker into the west corridor).
3. Expected (seed 1 log): A at about 68 s, B at about 178 s, C at about
   243 s, back HOME and `complete` at about 356 s. No collisions.
4. Stop recording when the display shows `complete`.

### Clip B: kidnapped robot recovery (about 20-30 s at 8x)

```powershell
python tools/set_run_config.py --method team --scenario kidnapped --seed 2
```

1. Start recording, press Run.
2. Watch the robot leave A and cross the central corridor toward B. Press `K`
   (3D view focused) **only after it has crossed into the right half of the
   map**, heading up toward B's doorway: about **2:28-2:30** on the clock.
   In the logged successes (seeds 2 and 4) the robot was at x = +1.4 m when
   kidnapped.
3. Expected: the robot jumps to the south-east room; the display shows
   `relocalize` and the robot turns in place for about 6 s, keeps both
   mirror hypotheses for a while, then the estimate locks back on (about
   57 s after the kidnap in our logs) and it continues to B, C, and HOME.
4. If it drives toward the wrong room, the filter picked the 180° mirror
   pose (the map is rotationally symmetric; see `docs/team_progress.md`).
   That is a known limit, not a crash. Reset and press `K` a little later,
   further into the right half.

### Clip C (optional, pick one)

- **Untested blocker placement:** `--scenario blocker_position_3 --seed 1`,
  press `3` at about 0:20. The blocker sits on the B -> C leg; the robot
  detects it with LiDAR and replans around it.
- **Extension, severe LiDAR degradation:**
  `--method team_robust --scenario nominal --seed 2 --degradation severe`.
  Scans drop out and get noisy, and the robust filter still completes the
  mission. Contrast: `--method team` with the same settings stalls after A
  (0/5 in our trials).

## 4. Suggested two-minute cut

| Time | Content |
|---|---|
| 0:00-0:10 | Title card: project title, Group 5 (Tron), Mateo Ortega, Zihao He, Devin Roberson |
| 0:10-0:20 | Map and mission: HOME -> A -> B -> C -> HOME, encoders + LiDAR only, no GPS or Supervisor pose |
| 0:20-1:05 | Clip A: point out the display (particle-filter pose, A* path), the blocker moved at 0:20, the replanned path |
| 1:05-1:35 | Clip B: kidnap, relocalize, recovery |
| 1:35-1:50 | Clip C or `results/webots/figures/success_by_scenario.png` |
| 1:50-2:00 | Results summary: team 5/5 vs baseline 0/5 in nominal, uncertain start, and blocked route; kidnap 2/5 (symmetry limit) |

## 5. After each clip

The evaluation Supervisor writes `results/run_YYYYMMDD_HHMMSS.csv`. Check it:

```powershell
python tools/check_run_health.py results/run_YYYYMMDD_HHMMSS.csv --require-complete
python tools/analyze_run.py results/run_YYYYMMDD_HHMMSS.csv
```

When finished, return Webots to the default (team, nominal):

```powershell
python tools/set_run_config.py --clear
```
