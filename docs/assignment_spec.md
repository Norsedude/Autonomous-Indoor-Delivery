# CMPE 185 Group Project: Autonomous Indoor Delivery

Markdown copy of `docs/CMPE185_Group_Project_Assignment.docx` (Fall 2026, release 4.0.0). The .docx is the authoritative version.

Fall 2026 | Release 4.0.0

## Team structure

The class will form seven teams of three students. Each team submits one proposal, one code repository, one presentation, one video, and one report. Every student must understand the integrated system and document an identifiable technical contribution.

## Mission

Program the supplied e-puck to complete an indoor delivery mission in Webots. The robot begins with an uncertain pose, travels to delivery locations A, B, and C in that order, and returns HOME. It must estimate its pose from wheel encoders and a 2D LiDAR, plan collision-free motion on an occupancy grid, and respond when an unexpected obstacle blocks the original route.
The evaluation Supervisor may record ground truth, move an obstacle, or change the robot pose. Submitted robot code may not access Supervisor functions, GPS, the ground-truth pose, or the Supervisor log during a run.

## Provided materials

- Webots building world and e-puck with encoders, LiDAR, and a Display
- Static occupancy-grid map and map metadata
- Mission destinations and a waypoint route for the initial baseline
- Sensor initialization and LiDAR coordinate conversion
- Differential-drive odometry baseline
- Conservative rotate-then-drive waypoint controller and wheel-speed conversion
- Latched emergency safety turn with clearance hysteresis
- Map, scan, path, and estimated-pose visualization
- Independent evaluation Supervisor and CSV ground-truth logger
- Deterministic nominal-run simulator, reference trajectory, log health checker,
and result-summary/plotting tools
- Scaffolds for localization, A*, obstacle avoidance, and optional mapping
The project frame is the Webots ENU x-y floor plane (z up). Heading is counterclockwise about +z. Robot and LiDAR data use FLU: +x forward, +y left, +z up. Do not swap axes or negate LiDAR x or y values.

## Required technical work


### 1 Baseline validation

Run and explain the supplied odometry and waypoint baseline. Confirm the wheel parameters and coordinate convention. Show how odometry error develops during a mission and identify at least one cause. Before modifying the controller:
1. run tools/validate_project.py, the unit tests, and
tools/simulate_baseline.py; and
2. record one untouched full Webots mission for which
tools/check_run_health.py LOG.csv --require-complete reports PASS.
The nominal starter must reach A, B, and C in order, return HOME, complete all 23 waypoints, and record no collision episode. The supplied safety behavior is a bounded last-resort recovery mechanism; it does not satisfy the obstacle-avoidance requirement below.

### 2 LiDAR localization

Implement Monte Carlo Localization or obtain prior approval for another probabilistic method. The implementation must include:
- an explicit initial belief
- an odometry-based motion update with noise
- a LiDAR measurement likelihood tied to the supplied map
- weight normalization and effective sample size
- conditional resampling
- a pose estimate that handles circular heading
- recovery after the evaluation Supervisor changes the robot pose
Students may implement a beam model, likelihood field, scan matching, or another well-justified measurement model.

### 3 Global path planning

Implement A* or Dijkstra on an inflated occupancy grid. The planner must:
- reject occupied cells and diagonal corner cutting
- produce a path from the current estimated pose to the active destination
- convert grid cells to metric waypoints
- report when no valid path exists
- replan after a persistent blockage or major localization correction

### 4 Path following and obstacle avoidance

Integrate the supplied waypoint follower with the method taught in class. The robot must avoid a LiDAR-detected obstacle that does not appear on the static map. Stopping permanently before the obstacle does not count as successful avoidance. The robot must resume progress toward the delivery goal.

### 5 Quantitative evaluation

Compare at least these two systems:
1. encoder odometry with the provided waypoint baseline
2. the team's LiDAR-corrected localization and navigation system
Run at least five trials for each required scenario. Keep the random seeds and scenario settings in the repository.

## Required scenarios

Scenario
Test condition
Main question
Nominal
Known start region, standard noise, blocker position 1
Does the integrated system complete the mission?
Uncertain start
Initial belief covers a larger region
Can the system localize before navigation fails?
Blocked route
Blocker position changes after planning
Does the robot avoid or replan?
Kidnapped robot
Instructor presses K during navigation
Can the belief recover after a large pose error?
The instructor may use an unreleased obstacle placement or starting pose in the final demonstration.

## Metrics

Report the following for every method and scenario:
- position RMSE in meters
- heading RMSE in radians or degrees
- mission success rate
- number of ordered destinations reached
- completion time
- ground-truth path length
- collision episodes
- recovery time after the kidnapped-robot event
State how success and recovery time are defined. Report individual trials as well as aggregate values. Discuss failure cases rather than removing them.

## Team selected extension

Choose one extension worth approximately 20% of the technical work:
- occupancy-grid mapping or full SLAM
- adaptive particle count
- comparison of two measurement models or localization methods
- D* Lite or another incremental replanner
- dynamic-obstacle prediction
- autonomous exploration and frontier selection
- camera landmark recognition fused with LiDAR localization
- systematic robustness study under sensor degradation
The required system takes priority. A partially implemented extension does not replace a missing required component.

## Milestones


### Proposal due October 8 2026

Submit one single-spaced page containing:
- title and team members
- problem statement and mission interpretation
- block diagram and planned data flow
- localization approach and initial belief
- evaluation hypothesis, baseline, metrics, and trial plan
- selected extension
- schedule, risks, and initial division of responsibility
- three to five technical references, including at least three archival papers
Path-planning and obstacle-avoidance choices may remain preliminary because those topics occur later in the course.

### Checkpoint 1 motion and sensing

- world runs on every team member's computer
- encoder odometry and LiDAR visualization work
- an untouched full-mission baseline CSV, health-check output, and plot are submitted

### Checkpoint 2 localization

- particle belief updates during motion
- LiDAR changes particle weights
- estimated and ground-truth trajectories are compared

### Checkpoint 3 integrated navigation

- planner produces a collision-free path
- robot follows the path and reaches at least two destinations
- unexpected obstacle triggers avoidance or replanning

### Final presentation December 1 or 3 2026

The presentation is 12 minutes including three minutes for questions. It must show the mission, approach, live or recorded demonstration, quantitative comparison, failure analysis, and each member's contribution.

### Final submission due December 11 2026

Submit:
- IEEE-style report
- two-minute demonstration video or unlisted link
- repository or ZIP with source code, configuration, and instructions
- all experiment CSV files and analysis scripts
- contribution statement and confidential peer evaluation

## Report organization

Use the IEEE conference paper format unless the instructor specifies another template. Include:
1. abstract
2. introduction and problem definition
3. related work with at least three archival sources outside class readings
4. system architecture and coordinate conventions
5. localization, planning, and control methods with equations
6. experimental design and metrics
7. quantitative results and failure analysis
8. extension
9. conclusion
10. individual contributions
11. references in IEEE style

## Grading

Component
Weight within project
Proposal
10%
Presentation and demonstration
30%
Final report and reproducible submission
60%

### Proposal 10 points

- clear and feasible problem definition: 3
- technical approach: 3
- experiment and evaluation plan: 2
- schedule and team responsibilities: 2

### Presentation and demonstration 30 points

- mission demonstration: 10
- technical explanation: 8
- quantitative results and interpretation: 8
- organization and questions: 4

### Final report and submission 60 points

- technical correctness: 15
- localization and navigation integration: 10
- experimental design and quantitative evaluation: 15
- team-selected extension: 8
- reproducibility of code and instructions: 7
- writing and contribution statement: 5
The instructor may adjust individual grades when repository history, contribution statements, demonstration questions, and peer evaluations show a substantial difference in participation.
