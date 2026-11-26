# Recording the two-minute demonstration video

You work in two windows:

- **VS Code terminal (PowerShell)**: type the `python ...` commands here.
  Open it with **Ctrl + `** (or Terminal > New Terminal). The prompt must end
  in `Autonomous-Indoor-Delivery>`. If not, run
  `cd "<path to>\Autonomous-Indoor-Delivery"` first.
  The Webots console cannot take commands; it only shows output.
- **Webots**: the simulation and the recording.

Timings below come from our logged Webots runs (`results/webots/team/...`).

## Part 1: One-time setup

**In the VS Code terminal:**

```powershell
git pull
python tools/validate_project.py
python -m unittest discover -s tests
python tools/set_run_config.py --clear
```

The first two checks must pass.

**In Webots:**

1. File > Open World > `worlds/cmpe185_indoor_delivery.wbt`.
2. Close the text-editor pane on the right (the x on its tab) so the 3D view
   is wider.
3. Drag/zoom the 3D view to a top-down view of all four rooms.
4. Turn on the **student display**: open **Overlays**, untick **Hide All
   Display Overlays** (or press **Shift+F12**), then make sure
   Overlays > 'CMPE185 e-puck' Overlays > Display Devices >
   **Show 'student display' Overlay** is ticked.
5. Drag the display panel into a corner of the 3D view and drag its
   bottom-right corner to enlarge it.
6. Click empty floor so the robot is not selected (hides the coloured move
   handles).
7. Never press **Ctrl+S** in Webots. Moving the camera is fine; saving
   rewrites the world file and the validator then fails.

**Display legend (useful for narration):**

| Colour | Meaning |
|---|---|
| dark grey | walls from the static occupancy map |
| blue line | remaining A* path (after line-of-sight simplification) |
| purple dots | MCL particles (150 of 300 drawn); stacked under the robot when confident, spread out after a kidnap |
| cyan dots | current LiDAR scan placed at the estimated pose; they sit on the walls when localization is right |
| red dot + line | estimated pose and heading |
| small red squares | obstacles detected by LiDAR that are not in the map (the blocker) |
| blue / green / orange / grey dots | goals A, B, C and HOME |
| text (top left) | method, waypoint, controller state, safety state, goal n/4, particle count, LiDAR inlier ratio |

## Part 2: The recording loop (same for every clip)

1. **VS Code terminal:** paste the clip's command and press Enter. It prints
   `Effective run: method=... scenario=... seed=...`.
2. **Webots:** File > **Reload World**.
3. Check the Webots console shows the matching line, for example
   `Run configuration: mcl+astar|blocked_route|seed=1`. If not, reload again.
4. If the display panel disappeared, press **Shift+F12** (or re-tick it under
   Overlays). Click empty floor to deselect the robot.
5. **Start recording:** click the **film-strip icon** in the toolbar (next to
   the camera icon), or File > Make Movie.... Choose **1920x1080**, high
   quality, **acceleration 8**, and start.
   Alternative: Windows Game Bar (**Win+Alt+R**) and speed the clip up later.
6. Press **Run** (the single play arrow, normal speed; not fast-forward,
   which does not render).
7. Do the clip's key press at the listed clock time. **Click inside the 3D
   view first** so Webots receives the key.
8. When the display shows `control: complete` (or you have the part you
   need), press **Pause**, click the film-strip icon again to stop, and save
   the file with the clip's name.
9. Optional check of the run you just recorded (VS Code terminal):

   ```powershell
   $log = Get-ChildItem results\run_*.csv | Sort-Object LastWriteTime | Select-Object -Last 1
   python tools/check_run_health.py $log.FullName --require-complete
   ```

   `PASS` = all 4 goals, no collision, back within 0.20 m of HOME.

## Part 3: The clips

### Clip A: full mission with a blocked route (required, about 45 s at 8x)

```powershell
python tools/set_run_config.py --method team --scenario blocked_route --seed 1
```

- At **0:20** on the clock: click the 3D view, press **2**. The red block
  jumps into the west corridor.
- Watch for red squares on the display where the LiDAR sees the block and
  the blue path rerouting around it.
- Expected: A at about 1:08, B at about 2:58, C at about 4:03, HOME and
  `complete` at about 5:56. No collisions.
- Save as `clipA_blocked_route.mp4`.

### Clip B: kidnapped-robot recovery (required, about 60 s at 8x, trim later)

```powershell
python tools/set_run_config.py --method team --scenario kidnapped --seed 2
```

- After goal A, the robot crosses the middle corridor toward B.
- At about **2:28-2:30**, once it is in the **right half** of the map, click
  the 3D view and press **K**.
- Expected: the robot jumps to the south-east room; the purple particles
  spread out; the status shows `relocalize` and the robot turns in place for
  about 6 s; about a minute later the particles collapse, the cyan dots line
  up with the walls, and it continues to B, C and HOME (about 8:10).
- If it drives toward the wrong room, the filter chose the 180° mirror pose
  (the map is rotationally symmetric; see `docs/team_progress.md`). Stop,
  reload, retake, and press K a little later in the right half. In our logs
  both successes were kidnapped at x = +1.4 m; the failures mostly on the
  left half.
- Save as `clipB_kidnap.mp4`.

### Clip C: untested blocker placement (recommended, about 45 s at 8x)

```powershell
python tools/set_run_config.py --method team --scenario blocker_position_3 --seed 1
```

- At **0:20**: click the 3D view, press **3**. The block moves to the
  south-east room, on the B -> C route.
- Watch around **4:00** (after B): red squares appear and the robot steers
  around the block. `complete` at about 6:00.
- Save as `clipC_unseen_blocker.mp4`.

### Clip D: baseline on the same blocked route (optional, about 10 s)

```powershell
python tools/set_run_config.py --method baseline --scenario blocked_route --seed 1
```

- At **0:20** press **2**. The odometry-only starter drives into the blocked
  corridor and stalls. The "before" shot for Clip A.
- Save as `clipD_baseline_blocked.mp4`.

### Clip E: extension, severe LiDAR degradation (optional)

```powershell
python tools/set_run_config.py --method team_robust --scenario nominal --seed 2 --degradation severe
```

- No keys. The cyan scan flickers and drops out; the robust filter still
  completes the mission (about 6:54).
- Contrast: same command with `--method team` stalls after goal A.
- Save as `clipE_extension_severe.mp4` (and `clipE_team_severe.mp4`).

## Part 4: Finish

```powershell
python tools/set_run_config.py --clear
```

Suggested two-minute cut (Clipchamp is built into Windows 11):

| Time | Content |
|---|---|
| 0:00-0:10 | Title card: project title, Group 5 (Tron), Mateo Ortega, Zihao He, Devin Roberson |
| 0:10-0:20 | Map and mission: HOME -> A -> B -> C -> HOME, encoders + LiDAR only, no GPS or Supervisor pose |
| 0:20-1:05 | Clip A (optionally 5 s of Clip D first): MCL pose, A* path, blocker moved, replanned path |
| 1:05-1:35 | Clip B: kidnap, particles spread, relocalize, recovery |
| 1:35-1:50 | Clip C or Clip E, or `results/webots/figures/success_by_scenario.png` |
| 1:50-2:00 | Results: team 5/5 vs baseline 0/5 in nominal, uncertain start, blocked route and both untested placements; kidnap 2/5 (symmetry limit) |
