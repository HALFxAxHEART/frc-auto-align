package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generated.TunerConstants;

/**
 * WHAT THIS FILE IS: every number you might want to look up or change, in one place.
 * <p>
 * Rules:
 * <ul>
 *   <li>No "magic numbers" in other files. If you type a number into a subsystem or command, it belongs here
 *       with a comment saying what it is and what UNITS it's in.</li>
 *   <li>Each mechanism gets its own section ({@code public static final class XxxConstants}).</li>
 *   <li>Names start with {@code k} (for "konstant"), e.g. {@code kMaxSpeed}. That's how you can tell a constant apart.</li>
 *   <li>The swerve drive's own numbers (module CAN IDs, encoder offsets, gear ratios, PID) are in
 *       {@code generated/TunerConstants.java} because Phoenix Tuner X writes that file for us.</li>
 *   <li>Names must say what the thing is, with units: {@code kIntakeDeployedRotations}, not {@code kThing2}.
 *       Our code is read by judges, alumni and other teams; keep every name professional (STUDENT-GUIDE section 8).</li>
 * </ul>
 * Use them with e.g. {@code Constants.DriveConstants.kMaxSpeed}, or add
 * {@code import static frc.robot.Constants.DriveConstants.*;} at the top of a file to just write {@code kMaxSpeed}.
 */
public final class Constants {
    private Constants() {}

/* -------------- CAN BUSES -------------- */

    /**
     * The robot has two CAN buses:
     * <ul>
     *   <li>"rio": the CAN port built into the roboRIO. Slower (CAN 2.0).</li>
     *   <li>The CANivore (a USB-to-CAN-FD adapter, ours is named "Jarvis"). Faster (CAN FD). The drivetrain is on it.</li>
     * </ul>
     * Every CTRE device is created with the bus it's wired to. Wrong bus = the device is "not found".
     */
    public static final class CANBuses {
        /** The roboRIO's built-in CAN bus */
        public static final String kRio = "rio";
        /** The CANivore the drivetrain is on. Comes from TunerConstants so there's only one place to change it. */
        public static final String kCanivore = TunerConstants.kCANBus.getName();
    }

/* -------------- CAN IDS -------------- */

    /**
     * Every CAN device's ID number. Two devices of the SAME TYPE on the same bus can't share an ID.
     * Set/check IDs in Phoenix Tuner X. If the code's ID doesn't match the real device, that part just won't work
     * (that's what happened to the 2026 gyro: the code said 24, the real Pigeon was 0).
     * <p>
     * Team numbering scheme:
     * <pre>
     *   0       Pigeon 2 gyro
     *   1       PDH (power distribution hub)
     *   10-29   mechanisms (intake, shooter, arm, ...)
     *   30-33   swerve STEER motors  (FL, FR, BL, BR)
     *   40-43   swerve DRIVE motors  (FL, FR, BL, BR)
     *   50-53   swerve CANcoders     (FL, FR, BL, BR)
     * </pre>
     * The swerve IDs (0, 30s, 40s, 50s) are set in {@code generated/TunerConstants.java}, not here.
     */
    public static final class CANIds {
        /** Example mechanism motor (see subsystems/ExampleRoller.java). Change to your real ID. */
        public static final int kExampleRollerMotor = 20;
    }

/* -------------- DIO PORTS -------------- */

    /**
     * Ports 0-9 on the roboRIO's "DIO" (digital input/output) header: limit switches, beam-break/laser sensors, LEDs.
     * See the wrappers/ folder for easy classes to use them. Each port can only be used once.
     */
    public static final class DIOPorts {
        // Example: public static final int kIntakeBeamBreak = 0;
    }

/* -------------- CONTROLLER CONSTANTS -------------- */

    public static final class OperatorConstants {
        /** USB port of the driver's Xbox controller in the Driver Station (USB tab) */
        public static final int kDriverPort = 0;
        /** Stick inputs smaller than this (as a fraction of full stick, 0-1) are ignored, so a resting stick doesn't creep */
        public static final double kDeadband = 0.1;
        /** Slow mode turns on when the right trigger is pulled past this much (0-1) */
        public static final double kSlowModeTrigger = 0.5;
        /** Speed multiplier while slow mode is on (0.4 = 40% speed) */
        public static final double kSlowModeMultiplier = 0.4;
        /** Battery voltage where the controller starts to rumble. Full rumble at the brownout voltage (6.75 V). */
        public static final double kRumbleStartVolts = 7.5;
        /** How fast the rumble fades after a dip (per 20ms loop). Dips last milliseconds, so this lets the driver feel them. */
        public static final double kRumbleDecayPerLoop = 0.03;
    }

/* -------------- DRIVETRAIN CONSTANTS -------------- */

    public static final class DriveConstants {
        /** Top driving speed, meters per second. Comes from TunerConstants so there is only one number to keep correct. */
        public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        /** Top spinning speed, radians per second (set here as 0.75 rotations per second). Raise it for faster turning. */
        public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        /** CAN devices get this many seconds after boot to report in before they're logged as disconnected */
        public static final double kConnectGraceSeconds = 5.0;
    }

/* -------------- PATHPLANNER (AUTO) CONSTANTS -------------- */

    public static final class AutoConstants {
        /** How hard PathPlanner corrects position error (meters of error -> m/s of correction). Too high = wobbly. */
        public static final PIDConstants kTranslationPID = new PIDConstants(5.0, 0.0, 0.0);
        /** How hard PathPlanner corrects heading error (radians of error -> rad/s of correction) */
        public static final PIDConstants kRotationPID = new PIDConstants(5.0, 0.0, 0.0);
        /** How long the "X Brake" named command locks the wheels, seconds */
        public static final double kXBrakeSeconds = 0.5;

        /** Robot mass with bumpers and battery, in kg. Keep in sync with the PathPlanner app's Robot Config. */
        private static final double kRobotMassKg = 74.088;
        /** Robot moment of inertia (how hard it is to spin), in kg*m^2 */
        private static final double kRobotMOI = 6.883;
        /** Wheel coefficient of friction (1.2 is typical for new tread) */
        private static final double kWheelCOF = 1.2;
        /** Drive motor current limit, in amps. Matches kSlipCurrent in TunerConstants. */
        private static final double kDriveCurrentLimit = 60;

        /**
         * Backup PathPlanner robot config built from TunerConstants. AutonContainer normally loads the config
         * saved by the PathPlanner app (deploy/pathplanner/settings.json); this is only used if that fails.
         */
        public static final RobotConfig kFallbackConfig = new RobotConfig(
            kRobotMassKg, kRobotMOI,
            new ModuleConfig(
                TunerConstants.FrontLeft.WheelRadius,
                DriveConstants.kMaxSpeed,
                kWheelCOF,
                DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                kDriveCurrentLimit,
                1),
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY));
    }

/* -------------- AUTO-ALIGN CONSTANTS -------------- */

    /**
     * Settings for commands/AutoAlign.java and commands/DriveToPose.java: hold a paddle and the robot drives itself
     * to a scoring/loading spot. Read AUTO-ALIGN-GUIDE.md for how it works.
     * <p>
     * Every spot is described by an AprilTag: "stand in front of tag N, this far out, this far to the side".
     * The tag's position comes from the official field layout, so the code never has a hand-typed field coordinate.
     * Next season: change {@link #kFieldLayout} and the tag IDs below, and the same code drives to the new game's spots.
     */
    public static final class AlignConstants {
        /**
         * Which field's AprilTag positions to use. MUST match the field map loaded on the Limelight's web page.
         * 2026 has two versions (welded steel field and AndyMark field); ask the event which one they have.
         */
        public static final AprilTagFields kFieldLayout = AprilTagFields.k2026RebuiltWelded;

        /**
         * One spot the robot can drive to.
         *
         * @param name       Shown on the dashboard, e.g. "Hub front"
         * @param tagId      The AprilTag the spot is in front of
         * @param sideOffsetM Meters to slide sideways along the tag's face, as seen by someone standing in front of
         *                   the tag LOOKING AT it: + = to their left, - = to their right. 0 = centered on the tag.
         *                   (In 2025 this is how teams picked the left or right reef pole.)
         */
        public record FieldTarget(String name, int tagId, double sideOffsetM) {}

        /**
         * A group of spots for one button. The robot goes to whichever spot in the group is CLOSEST when the
         * button is pressed. Spots are listed separately for each alliance because each alliance has its own hub/outpost.
         */
        public record TargetGroup(String name, FieldTarget[] blue, FieldTarget[] red) {}

        /**
         * HUB. Each hub face has its own tags. Only the face toward our driver station and the face toward the
         * middle of the field can be reached (the bumps block the other two), so those two are listed.
         * Blue hub: tag 26 = face toward the blue wall, tag 20 = face toward the middle. Red hub: tags 10 and 4.
         */
        public static final TargetGroup kHub = new TargetGroup("Hub",
            new FieldTarget[] { new FieldTarget("Blue hub, wall side", 26, 0.0), new FieldTarget("Blue hub, middle side", 20, 0.0) },
            new FieldTarget[] { new FieldTarget("Red hub, wall side", 10, 0.0), new FieldTarget("Red hub, middle side", 4, 0.0) });

        /**
         * OUTPOST (human player station in the corner of our driver station wall). It has two tags side by side,
         * 0.432 m apart; the opening is centered between them, so we use the first tag and slide half the gap
     * (0.216 m) to the right, toward the second tag.
         * Blue: tags 29 + 30. Red: tags 13 + 14.
         */
        public static final TargetGroup kOutpost = new TargetGroup("Outpost",
            new FieldTarget[] { new FieldTarget("Blue outpost", 29, -0.216) },
            new FieldTarget[] { new FieldTarget("Red outpost", 13, -0.216) });

        /** Half the robot's length WITH bumpers, meters (bumper edge to robot center). Hephaestus is 0.9 m square. */
        public static final double kRobotHalfLengthM = 0.45;
        /** Gap left between our bumper and the tag's face when we arrive, meters. 0 = bumper touching. */
        public static final double kBumperGapM = 0.10;
        /**
         * Which side of the robot points AT the target. true = the front (intake) faces it, false = the back.
         * Pick-and-place robots usually score off the front.
         */
        public static final boolean kFaceTargetWithFront = true;

        /**
         * The APPROACH spot is this much farther out than the final spot, straight back from the tag, in meters.
         * PathPlanner drives to the approach spot (avoiding obstacles), then DriveToPose drives straight in.
         * Bigger = straighter, safer final approach, but slower.
         */
        public static final double kApproachDistanceM = 0.75;
        /** If we start closer than this to the final spot, skip PathPlanner and just drive straight in, meters... */
        public static final double kSkipPathfindingDistanceM = 1.0;
        /** ...and we're no more than this far off to the side of the straight-in lane, meters (else we might clip a corner) */
        public static final double kMaxSidewaysToSkipPathfindingM = 0.3;

        /** PathPlanner speed limits while pathfinding: m/s, m/s^2, deg/s, deg/s^2. Kept below full speed for safety. */
        public static final double kPathfindMaxSpeed = 3.0;
        public static final double kPathfindMaxAccel = 3.0;
        public static final double kPathfindMaxTurnDegPerSec = 360;
        public static final double kPathfindMaxTurnAccelDegPerSec2 = 540;

        /** Final approach: speed = kP x distance left (m/s per meter of error), capped at the max below */
        public static final double kFinalTranslationP = 3.0;
        /** Final approach turning: turn rate = kP x heading error (rad/s per radian of error) */
        public static final double kFinalRotationP = 4.0;
        /** Final approach top speed (m/s), acceleration (m/s^2) and turn rate (rad/s) */
        public static final double kFinalMaxSpeed = 1.5;
        public static final double kFinalMaxAccel = 3.0;
        public static final double kFinalMaxTurnRadPerSec = 3.0;
        /** "Arrived" when within this many meters and degrees of the spot... */
        public static final double kPositionToleranceM = 0.03;
        public static final double kHeadingToleranceDeg = 2.0;
        /** ...for this long, seconds (so we don't call it done while still swinging through the spot) */
        public static final double kSettleSeconds = 0.15;
        /** Give up if the final approach hasn't arrived after this long, seconds (something is probably in the way) */
        public static final double kFinalApproachTimeoutSeconds = 3.0;

        /**
         * Only start if a Limelight saw a tag within this many seconds. If the robot doesn't know where it is,
         * it would confidently drive to the wrong place. (Skipped in simulation, which has no camera.)
         */
        public static final double kMaxSecondsSinceTag = 2.0;
        /** Moving either drive stick past this (0-1) cancels the align and gives the driver control back */
        public static final double kDriverOverrideStick = 0.2;
        /**
         * STUCK check: if the wheels are told to go faster than kStallCommandedSpeed (m/s) but the robot moves slower
         * than kStallActualSpeed (m/s) for kStallSeconds, we've hit something (robot, wall, game piece). Give up.
         */
        public static final double kStallCommandedSpeed = 0.5;
        public static final double kStallActualSpeed = 0.1;
        public static final double kStallSeconds = 0.5;
        /** Controller rumble length when we arrive (one short buzz) or give up (a longer buzz), seconds */
        public static final double kArrivedRumbleSeconds = 0.25;
        public static final double kFailedRumbleSeconds = 0.75;
    }

/* -------------- VISION CONSTANTS -------------- */

    public static final class VisionConstants {
        /**
         * Where a Limelight is mounted, measured from the CENTER of the robot on the floor.
         * Same numbers as "Robot Space Pose" on the Limelight's web page.
         *
         * @param forwardM Meters forward (+) or backward (-) of the robot center
         * @param sideM    Meters left (+) or right (-) of the robot center
         * @param upM      Meters up from the floor
         * @param rollDeg  Tilt sideways, degrees (usually 0)
         * @param pitchDeg Tilt up (+) or down (-), degrees. A camera aimed slightly up at the tags is usually +10 to +30.
         * @param yawDeg   Which way it faces, degrees: 0 = front, 180 = back, +90 = left, -90 = right
         */
        public record CameraPose(double forwardM, double sideM, double upM, double rollDeg, double pitchDeg, double yawDeg) {}

        /**
         * One Limelight on the robot.
         *
         * @param name     The Limelight's hostname, exactly as set on its web page (e.g. "limelight-front")
         * @param pipeline Pipeline to run (0-9), or null to leave whatever the Limelight is set to
         * @param pose     Where it's mounted, or null to use the Robot Space Pose saved on the Limelight's web page
         */
        public record Camera(String name, Integer pipeline, CameraPose pose) {
            /** Mount position and pipeline set on the Limelight's web page. Code doesn't override anything. */
            public static Camera configuredOnLimelight(String name) {
                return new Camera(name, null, null);
            }

            /** Mount position and pipeline set here in code. Overrides what's on the Limelight's web page. */
            public static Camera configuredInCode(String name, int pipeline, CameraPose pose) {
                return new Camera(name, pipeline, pose);
            }
        }

        /**
         * Every Limelight on the robot. Each one feeds the robot's position. See STUDENT-GUIDE.md "Add another Limelight".
         * <p>
         * Team standard: set the mount position + pipeline HERE (configuredInCode), so it's saved in git and comes back
         * automatically if a camera is swapped or reflashed. Image settings (exposure, resolution, tag family, field
         * map) stay on the Limelight's web page.
         */
        public static final Camera[] kCameras = {
            // Hephaestus's camera. Its mount position hasn't been measured into code yet, so it uses the one saved on
            // the Limelight. Once measured, switch to configuredInCode like the example below.
            Camera.configuredOnLimelight("limelight"),

            // EXAMPLE second camera on the back of the robot, 30 cm behind center, 25 cm up, tilted up 20 deg, facing backward:
            // Camera.configuredInCode("limelight-back", 0, new CameraPose(-0.30, 0.0, 0.25, 0, 20, 180)),
        };

        /** A Limelight can take ~30 s to boot; don't log it as disconnected before then */
        public static final double kBootGraceSeconds = 30.0;
        /** A Limelight is "disconnected" if its heartbeat hasn't changed for this long, seconds */
        public static final double kHeartbeatTimeoutSeconds = 0.5;

        /** Ignore vision while spinning faster than this (rotations per second); the images blur and lag. */
        public static final double kMaxSpinRps = 2.0;

        /**
         * "Std dev" = how much to trust a measurement. Smaller = trust it more. In meters (or radians for heading).
         * How much to trust MegaTag2 X/Y. Heading always comes from the Pigeon.
         */
        public static final double kMegaTag2XYStdDev = 0.7;
        /** How much to trust MegaTag1 while disabled, in meters/radians. Used to fix the starting heading. */
        public static final double kMegaTag1StdDev = 0.5;
        /** MegaTag1 needs at least this many tags before it is allowed to fix the heading */
        public static final int kMegaTag1MinTags = 2;
    }

/* -------------- EXAMPLE MECHANISM -------------- */

    /** Settings for subsystems/ExampleRoller.java. Copy this section when you add a real mechanism. */
    public static final class ExampleRollerConstants {
        /** Which CAN bus the motor is wired to */
        public static final String kCANBus = CANBuses.kRio;
        /** Speed the roller runs at for intaking, rotations per second of the MOTOR (not the roller) */
        public static final double kIntakeRps = 40;
        /** Stator current limit, amps. Limits torque/heat so the motor and mechanism don't get hurt. */
        public static final double kStatorCurrentLimit = 40;
        /** Supply current limit, amps. Limits what it pulls from the battery so the robot doesn't brown out. */
        public static final double kSupplyCurrentLimit = 30;
        /** Velocity PID + feedforward gains. kV = volts per RPS (about 0.12 for a Kraken X60). kP = extra volts per RPS of error. */
        public static final double kP = 0.1;
        public static final double kV = 0.12;
        /** "At speed" if within this many RPS of the target */
        public static final double kToleranceRps = 2;
    }
}
