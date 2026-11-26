# Presentation work log — Mateo Ortega

This log records Mateo's presentation work for the CMPE 185 Group 5 (Tron)
project. It is a working contribution record, not a claim about code ownership.

Presentation: https://docs.google.com/presentation/d/1zKQ-C0b8SaJn00IhIOfR8yzC2oi_I6Zf-XY3cT02Ik4/edit

## 2026-11-25 — opening and demonstration slides

Mateo identified himself as the presenter preparing the opening section and
asked that his work be tracked. The slide content below was observed in the
linked deck on 2026-11-25. Authorship is based on Mateo's statement that these
are the slides he worked on; it has not been verified from Google Slides
revision history.

| Slide | Current content and work | Status / follow-up |
| --- | --- | --- |
| 1 — Title | Group 5 (Tron), CMPE 185, December 3, 2026, project title, methods, and all three team members. | Present. Consider shortening the main title to `Autonomous Indoor Delivery` and removing the semicolon after the names. |
| 2 — Agenda | Three sections: mission/system, localization/planning/demo, and evaluation/robustness/lessons. Includes previews of the pipeline, robot/map, and results chart. | Present. |
| 3 — Mission | HOME → A → B → C → HOME; uncertain start, blocked route, and relocation challenges; labeled supplied nominal-route map. | Present. Caption correctly identifies the route as provided baseline material. |
| 4 — How Our Robot Navigates | Six-stage sense, localize, plan, follow, avoid, replan timeline with descriptions. | Present. A second boxed pipeline remains above the timeline in the latest rendered slide; decide whether to retain it or give the timeline more space. |
| 5 — Robot in Action | Four visual phases using Webots screenshots: start at HOME, localize/plan, detect/replan, and complete the mission. | Present. Speaker script drafted below. |

## Slides reserved for later demonstration use

Mateo asked to retain slides 6–13. Their current titles/roles are:

| Slide | Current title or role |
| --- | --- |
| 6 | Normal Test |
| 7 | Normal Test (Sped Up) |
| 8 | Demonstration Clips overview |
| 9 | Full Delivery with Blocked Route |
| 10 | Kidnapped Robot Recovery |
| 11 | Untested Obstacle Placement |
| 12 | Provided Baseline (Odometry Only) |
| 13 | Extension: Robustness Under Degraded LiDAR |

These slides are retained for future use; this log does not mark their videos,
speaker notes, or final visual QA as complete.

## Slide 5 speaker script — draft

"These screenshots show the delivery process in Webots. The robot starts at
HOME and uses its encoders and LiDAR to estimate where it is on the map. The
display shows its estimated pose, the next goal, and the route it plans to
follow. When a new blocker appears on that route, the robot detects it,
avoids it, and replans so it can keep moving. It then visits A, B, and C in
order and returns HOME. In the next section, we'll show the runs and explain
the methods behind them."

Target delivery: approximately 40–50 seconds. The screenshots are a visual
overview; the recorded demonstration and Webots logs provide the run evidence.

## Ongoing entries

Add a dated entry whenever Mateo changes slide content, records or edits demo
footage, writes speaker notes, rehearses, or submits presentation materials.
Record the slide or artifact, the concrete change, and the evidence or source.
Keep actual code and experiment contributions separate until verified.
