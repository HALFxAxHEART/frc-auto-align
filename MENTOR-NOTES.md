# Mentor notes: teaching auto-align

Companion to [AUTO-ALIGN-GUIDE.md](AUTO-ALIGN-GUIDE.md) (the student guide). This page is for whoever runs the sessions.

## What's in this repo

This repo is the team's 2026 Robot Template (CTRE swerve + PathPlanner + Limelight, Hephaestus's drivetrain), plus auto-align.
The git history is split on purpose:
1. **First commit** = the plain template.
2. **The auto-align commit** = *everything* that was added. Show students that diff (GitHub → the commit → "Files changed"):
   it's the whole feature in one readable change.

**Deploying this to Hephaestus replaces the competition code** (the `goonettes` branch of Hephaestus). Only the
drivetrain and vision run. The intake, turret, hood and shooter get no commands, so they sit still. When you're done, redeploy
Hephaestus's own code if the robot needs to shoot again.

## What has and hasn't been tested

| Tested | How |
|---|---|
| Compiles, boots, drives to the hub (both faces) and the outpost on **both alliances** | Simulation with the real code, more than a dozen runs. All arrived within 2-3 cm and 0.1°. |
| Picks the closer hub face | Simulation (start from the middle of the field → middle face; from the alliance zone → wall face) |
| Close + lined up → skips pathfinding; close but beside the hub → still pathfinds | Simulation |
| Final-approach timeout | Simulation with `kFinalTranslationP = 1` (challenge 5): gave up at 3 s as designed |
| Pressing during PathPlanner's boot warm-up | Simulation: found a real bug (robot sat forever in "Pathfinding"), fixed with the startup check |

| **Not yet tested** | Why / how to test |
|---|---|
| Anything on the **real robot** | Follow the guide's section 10 step by step, slow speeds first |
| Stick override, stuck check | Simulation can't push a stick mid-run or put a wall in the way. Test on the robot (guide step 6) |
| Paddle mapping | Depends on your controller model; verify in the DS USB tab (buttons 9 and 10) |
| Pose accuracy on **your half field** | See the next section; this is the most likely thing to need work |

## The half field: the one thing that will bite you

The Limelight works out the robot's position from **where the field map says each tag is**. The code works out the
targets from **the same official map**. So the tags on your half field have to be where the official map says,
relative to each other, or the robot's position and the targets won't agree.

- If the half field was built from the official field drawings: you're fine. Set the Limelight's field map to the 2026
  map (welded or AndyMark, whichever matches the drawings you built from) and make `kFieldLayout` match.
- If the elements are just roughly placed: measure where the tags really are (tape measure from the corner you call (0, 0)),
  make a custom field map, upload it to the Limelight (it accepts custom field maps; see Limelight's docs on custom maps), and load
  the same JSON in code: in `AutoAlign`, replace `AprilTagFieldLayout.loadField(kFieldLayout)` with
  `new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("half-field.json"))` (it throws an
  `IOException`, so wrap it). Good student project.
- **Driver Station alliance color = the side you built.** The code goes to *our* alliance's hub/outpost. If you built
  the blue half, run as Blue.
- The robot needs to have seen a tag in the last 2 s when the paddle is pressed. On a half field with few tags, the
  camera may lose sight of them. If presses keep getting refused, raise `kMaxSecondsSinceTag` a bit, or add tags.

## Suggested sessions (about 90 min each)

| # | Topic | Guide sections | Activity |
|---|---|---|---|
| 1 | Where am I? Field coordinates and pose | 1-3 | Put the robot at taped spots, compare the AdvantageScope pose with a tape measure. Challenge 1 on paper. |
| 2 | Targets from tags (transforms) | 4 | Challenges 1 (check in sim), 2 and 3 |
| 3 | Pathfinding and obstacles | 5 (stage 1), 6 | Sim. Challenges 6 and 8. Discuss challenge 7. |
| 4 | P controllers and tuning | 5 (stage 2), 11 | Sim: challenge 5 with a graph on screen. Then the robot, guide section 10, slow speeds. |
| 5 | Next year | 12 | Challenge 4 (reef-style left/right). Plan what to build once the 2027 game is out. |

Tips:
- Have students predict before they run. "Where will it stop?" "Which face will it pick?" Then run it.
- Project AdvantageScope's 2D field big: watching the planned path appear is the moment it clicks.
- One laptop per 2-3 students for sim; only one group on the robot at a time.

## Robot safety during testing

- Everyone off the field. One person on the laptop with a finger near **Enter** (disable); **Space** = emergency stop.
- First runs at `kPathfindMaxSpeed = 1.0`, `kFinalMaxSpeed = 0.5`. Raise one step at a time.
- The robot only moves while the paddle is held. Teach drivers: **when in doubt, let go.**
- It does **not** see people, robots or game pieces. It will drive into them.

## Challenge answers

1. **By hand.** Out = 0.45 + 0.10 = 0.55 m along the tag's +X. Tag 26 faces 180° (toward the blue wall), so "out" is −X:
   X = 4.022 − 0.55 = **3.472**, Y = **4.035**, heading = 180° + 180° = **0°** (facing the hub, toward red).
2. **Back up to it.** `kFaceTargetWithFront = false`. The added rotation becomes 0° instead of 180°, so the robot
   faces the same way as the tag: at the outpost (tag faces 0°) the heading becomes 0° with the back to the wall. It changes
   *every* target. To do it per target, add a field to the `FieldTarget` record (good follow-up).
3. **Tower button.** In `AlignConstants`:
   ```java
   public static final TargetGroup kTower = new TargetGroup("Tower",
       new FieldTarget[] { new FieldTarget("Blue tower", 31, -0.216) },
       new FieldTarget[] { new FieldTarget("Red tower", 15, -0.216) });
   ```
   and in `RobotContainer`: `driver.a().whileTrue(autoAlign.alignTo(AlignConstants.kTower));`.
   **Catch:** the tower's base sticks out from the wall, so 0.10 m from the *tag* would drive into it. Look at the navgrid
   (blocked out to ~1.8 m there) and measure the real tower. The right fix is a per-target "extra distance out" field
   in `FieldTarget`, used in `AutoAlign.targetPose`. Good discussion of why defaults don't fit every target.
4. **Reef-style.** Two groups with the same tags and `sideOffsetM` of `+0.3` (left) and `-0.3` (right), each on its own
   button. Side offset is "as seen looking at the tag", which is also the robot's left/right when it faces the tag front-first.
5. **Feel the P.** kP = 1: speed = distance, so from 0.75 m the distance halves about every 0.7 s. Getting to 3 cm takes
   about ln(0.75/0.03) ≈ 3.2 s, which is *longer than the 3 s timeout*, so it gives up. (Great lesson: the parts interact.)
   kP = 10: it hits the 1.5 m/s cap until ~15 cm out, then brakes hard. In sim it's fine. On a real robot, delays
   (motors, sensor lag) make high kP overshoot and wiggle.
6. **Fake obstacle.** The path goes around the new blocked area. If it's blocking the only way, there's no path and the
   robot won't move. Discuss what the driver sees then (`Align/State` stays on Pathfinding: a good improvement project
   is a timeout for stage 1 too).
7. **Avoiding robots.** You'd need to *detect* robots: a camera running an object-detection model trained on bumpers
   (e.g. a Limelight neural detector pipeline), or a LIDAR. Turn each detection into a field position using our pose,
   make a small box around it, and pass the boxes to `Pathfinding.setDynamicObstacles(boxes, robotPosition)` every
   loop. PathPlanner re-plans around them. The final approach (DriveToPose) would still need its own check.
8. **Skipping stage 1.** Already close and lined up = nothing to avoid. Pathfinding there would drive back out to the
   approach pose and then come back in, wasting time, and our start might be inside a blocked square near the hub.
   It also checks we're *lined up*, because from beside the hub a straight line to the spot would clip the hub's corner.

## Ideas for advanced students

- A timeout for stage 1 (pathfinding), like the final approach has.
- LEDs that show the align state (aligning / arrived / gave up).
- Per-target settings (distance out, front/back), see challenges 2 and 3.
- A custom half-field layout (see above).
- Robot avoidance (challenge 7).
- Replace DriveToPose's P controller with WPILib's `ProfiledPIDController` and compare the graphs.
