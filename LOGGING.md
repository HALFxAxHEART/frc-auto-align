# Year-over-year logging template

`RobotLog.java` is the reusable part: copy it into next year's project, call `RobotLog.start()` first thing in
`Robot()` and `RobotLog.periodic()` in `robotPeriodic()`. Open the logs in **AdvantageScope** (`.wpilog` from WPILib,
`.hoot` from CTRE).

This is what to log every season and why. Groups 1–5 stay the same every year. Group 6 changes with the game.
The **"In this template"** line says where each item is logged now.

## 1. Match context
- Code version (branch + commit)
- Alliance, match number, match phase (auto/teleop)
- Which auto was selected

**Why:** so every log is matched to the code and match it came from.

**In this template:** `Code Version` on the dashboard (made by `writeVersionFile` in build.gradle). An `AUTO START` / `TELEOP START`
event line with match type, number, alliance and auto name (`Robot.logMatchStart`). FMS info, DS mode and
`Auton Selector` are recorded automatically from NetworkTables.

## 2. Power and health
- Battery voltage and brownout events
- Total current, and each breaker channel
- Per-motor stator and supply current
- Per-motor temperature
- Faults and sticky faults on every CAN device
- CAN bus utilization and loop-time overruns

**Why:** most "the robot acted weird" problems turn out to be power or CAN, like this year's 120 A drive limit and the gyro ID.

**In this template:** `Power/Battery (V)`, a `BROWNOUT` event, and `Power/PDH` (total current + every channel) in `RobotLog.periodic`.
Per-motor current, temperature and faults are recorded for every CTRE device in the `.hoot` files (SignalLogger).
`CAN/rio/*` and `CAN/Jarvis/*` show utilization and errors. `Logging/Loop time (ms)` plus a `LOOP OVERRUN` event.
A `CAN DISCONNECTED` event the moment the Pigeon or any swerve motor/CANcoder drops off (`CommandSwerveDrivetrain.logDisconnects`).

## 3. Drivetrain and position
- The robot's position estimate (fused pose)
- The camera's pose, tag count, and how much it corrected the position
- Gyro yaw, and whether the gyro is connected
- Swerve module states, commanded vs. actual

**Why:** this catches drift, a bad gyro, and wheels slipping.

**In this template:** `DriveState/Pose`, `ModuleStates` vs `ModuleTargets` (`Telemetry.java`). `Poses/Vision/<camera>`,
`Vision/<camera>/Tags seen`, `Correction (m)`, `Heading disagreement (deg)` (`Vision.java`). `Gyro/Yaw (deg)`, `Gyro/Connected`.

## 4. Driver and commands
- All joystick axes and buttons
- Every command start, end and interrupt

**Why:** it answers "did the driver press it, and did something interrupt it?"

**In this template:** joysticks via `DriverStation.startDataLog`. The `Commands` entry gets `START` / `END` / `INTERRUPTED ... by ...` lines.

## 5. Autonomous
- The selected auto, the active path, and the target pose vs. the actual pose
- Every named-command event

**Why:** it shows where an auto falls behind, drifts, or hits something.

**In this template:** `Poses/Auto path` and `Poses/Auto target` (PathPlannerLogging in `AutonContainer`). An `AUTO named command: <name>`
event each time one runs (register named commands with `register(...)`, not `NamedCommands.registerCommand` directly).

## 6. Game-specific (rewrite each season)
- Every scoring attempt: inputs (distance, setpoints vs. actual, robot speed, pose) and, if you can, the result (made or missed)
- Every closed-loop mechanism: setpoint vs. actual, and when it reports "ready"
- Game-piece state: sensors, counts, and whether the robot has a piece
- Safety interlocks: when they trip and why (2026: hood near trench, turret out of range)

**Why:** this is what you tune from, and what explains a miss.

**How:** one `RobotLog.event(...)` line per scoring attempt, plus `RobotLog.pose("Shot", pose)` so attempts show on the field.
Use `RobotLog.eventOnRise(...)` for interlocks. Put setpoint/actual/ready on the dashboard (it's logged automatically).
2026 example (Hephaestus `RobotContainer.logShot`):
```java
RobotLog.event(String.format(
    "SHOT #%d %s %s | dist %.2fm | shooter %.1f/%.1f RPS | hood %.2f/%.2f | speed %.2f m/s | pose (%.2f, %.2f, %.0f deg)",
    count, kind, passing ? "PASS" : "HUB", distance, shooterRps, targetRps, hood, hoodTarget, speed, x, y, deg));
RobotLog.pose("Shot", pose);
RobotLog.eventOnRise("hoodForced", hood.isForcedDown(), "Hood held down near a trench");
```

## Rules
1. **USB stick in the roboRIO, always.** It keeps logs off the roboRIO's small internal storage. (`Logging/USB stick` on the dashboard goes false and the DS shows a warning if it's missing.)
2. **Log events once when they start, not every loop.** Use `eventOnRise`.
3. **Use the log names the dashboard uses**, so live debugging and log review match. Anything on the dashboard is already in the log.
4. **After every event, pull the logs and save them as `Season/Event/Match#`.** (`/u/logs/*.wpilog` and `/u/ctre-logs/*.hoot` on the stick.)
5. **The first time something weird happens, check the log before changing code.**
