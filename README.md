# TorqueNados 5090 Robot Template — 2026

Starting point for a 2026 robot: **CTRE swerve + PathPlanner + Limelight vision**, nothing game-specific.
**New here? Read [STUDENT-GUIDE.md](STUDENT-GUIDE.md) first.** Names in this code must be professional and descriptive (guide section 8): judges, alumni and other teams read it. Every Java file also starts with a "WHAT THIS FILE IS" note.

Built from the drivetrain and vision code of our 2026 robot, Hephaestus, with the game-specific mechanisms removed.

## What's in it
| Part | Where | Notes |
|---|---|---|
| Swerve drive (Phoenix 6) | `generated/TunerConstants.java`, `subsystems/CommandSwerveDrivetrain.java` | Currently Hephaestus's drivetrain (SDS MK5n R3, CANivore "Jarvis", Pigeon ID 0). **For a new robot, regenerate TunerConstants with Tuner X.** |
| Vision (Limelight) | `subsystems/Vision.java`, `LimelightHelpers.java` (v1.14, needs LLOS 2026+) | MegaTag2 while enabled (X/Y only, heading from the Pigeon). MegaTag1 with 2+ tags while disabled to fix the starting heading. Any number of cameras, listed in `Constants.VisionConstants.kCameras` with their mount position + pipeline (set in code). See STUDENT-GUIDE "Add another Limelight". |
| Autos (PathPlanner) | `commands/AutonContainer.java`, `deploy/pathplanner/` | Example "Drive Forward" auto. Add autos to the chooser in `buildAutonChooser()`, register robot actions in `registerNamedCommands()`. |
| Logging | `RobotLog.java`, [LOGGING.md](LOGGING.md) | Everything to the USB stick: dashboard values, joysticks, commands, events, power, CAN, CTRE devices. LOGGING.md has what to log each season and why. |
| Constants | `Constants.java` | Every number in one place: CAN buses, CAN ID map + numbering scheme, DIO ports, controller, drive speeds, PathPlanner, vision, example mechanism. |
| Example mechanism | `subsystems/ExampleRoller.java` | A commented one-motor CTRE subsystem to copy for real mechanisms. Not turned on. |
| Wrappers | `wrappers/` | LimitSwitch, LaserDetector, DigitalLED (plain WPILib). |

## Controls (driver, port 0)
- **Left stick** drive, **right stick** turn (field-centric)
- **Right trigger** slow mode (40%)
- **X** lock wheels in an X
- **Start** reset field-centric forward to the way the robot faces
- Controller rumbles as the battery nears brownout (starts at 7.5 V)

## Dashboard
- `Vision/Use camera` — turn off to ignore the Limelights (pure odometry)
- `Vision/<camera>/Correction (m)` — how far vision moved the pose; consistently large = odometry problem
- `Gyro/Connected`, `Field`, `Auton Selector`, `Code Version` (branch + commit on the robot)

## Setting up a new robot
1. Tuner X → Swerve Project Generator → replace `generated/TunerConstants.java` (keep `kSlipCurrent` sensible, ~60 A).
2. PathPlanner app → Settings → Robot Config: mass, MOI, module locations, gearing.
3. Each Limelight's web page: unique hostname (`limelight-front`...) + static IP + AprilTag pipeline. Then add it to `kCameras` with its measured mount position (STUDENT-GUIDE → "Add another Limelight").
4. Vendordeps: Phoenix 6 26.3.0, PathPlannerLib 2026.1.2 (update via WPILib "Manage Vendor Libraries").
