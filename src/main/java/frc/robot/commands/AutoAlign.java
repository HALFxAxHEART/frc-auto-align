package frc.robot.commands;

import static frc.robot.Constants.AlignConstants.*;

import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlignConstants.FieldTarget;
import frc.robot.Constants.AlignConstants.TargetGroup;
import frc.robot.RobotLog;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * WHAT THIS FILE IS: "hold a button and the robot drives itself to the hub / outpost". The same idea teams used in
 * 2025 to line up on the reef. Read AUTO-ALIGN-GUIDE.md for the full explanation with pictures.
 * <p>
 * When the button is pressed:
 * <ol>
 *   <li><b>Check</b> we can trust our position (a Limelight saw a tag recently). If not, buzz and do nothing.</li>
 *   <li><b>Pick</b> the closest spot in the group (e.g. the closer of the two reachable hub faces).</li>
 *   <li><b>Work out two poses</b> from that spot's AprilTag: the FINAL pose (bumper just short of the tag, facing it)
 *       and an APPROACH pose straight out from it.</li>
 *   <li><b>Stage 1, PathPlanner pathfinding</b> to the approach pose. PathPlanner plans a route around the field
 *       elements marked in deploy/pathplanner/navgrid.json and re-plans if needed. Skipped if we're already close.</li>
 *   <li><b>Stage 2, DriveToPose</b>: straight in to the final pose with a simple P controller, for accuracy.</li>
 * </ol>
 * Letting go of the button or moving a drive stick stops it right away (the driver is back in control).
 * It gives up on its own (and buzzes the controller) if the robot is stuck or the final approach takes too long.
 * When it arrives, the controller gives one short buzz.
 * <p>
 * Dashboard: Align/State, Align/Target, Align/Distance left (m). AdvantageScope: Poses/Align/Goal and
 * Poses/Align/Approach drawn on the field, plus PathPlanner's planned path (Poses/Auto path).
 */
public class AutoAlign {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Vision m_vision;
    private final BooleanSupplier m_onRedAlliance;
    private final BooleanSupplier m_driverIsDriving;
    private final DoubleConsumer m_rumbleSeconds;

    /** Where every AprilTag is on the field (official layout, built into WPILib) */
    private final AprilTagFieldLayout m_field = AprilTagFieldLayout.loadField(kFieldLayout);

    /** Speed limits for PathPlanner while pathfinding */
    private final PathConstraints m_constraints = new PathConstraints(
        kPathfindMaxSpeed, kPathfindMaxAccel,
        Units.degreesToRadians(kPathfindMaxTurnDegPerSec), Units.degreesToRadians(kPathfindMaxTurnAccelDegPerSec2));

    /**
     * The first pathfind after boot is slow (Java loads and optimizes the code the first time it runs), so PathPlanner
     * gives us a practice run to do at boot. It shares the pathfinder with real aligns, so an align pressed while it's
     * still running would get stuck: aligns are refused until it's done. Scheduled from RobotContainer.
     */
    private final Command m_warmup = PathfindingCommand.warmupCommand();

    /** True once the "stuck" condition has lasted kStallSeconds in a row */
    private final Debouncer m_stuck = new Debouncer(kStallSeconds);
    /** Stop reason when the driver moves a stick. The driver meant to do that, so it's treated like letting go. */
    private static final String kDriverTookOver = "driver moved a stick";
    /** Why the current align stopped early, or null if it hasn't */
    private String m_stopReason = null;
    /** When the final approach started (for its timeout), or NaN while still pathfinding */
    private double m_finalApproachStartTime = Double.NaN;

    /**
     * @param drivetrain      The swerve drive
     * @param vision          The vision subsystem (to check a tag was seen recently)
     * @param onRedAlliance   Returns true on the red alliance (picks which hub/outpost is ours)
     * @param driverIsDriving Returns true while the driver is moving a stick (cancels the align)
     * @param rumbleSeconds   Buzzes the driver's controller for this many seconds
     */
    public AutoAlign(CommandSwerveDrivetrain drivetrain, Vision vision, BooleanSupplier onRedAlliance,
                     BooleanSupplier driverIsDriving, DoubleConsumer rumbleSeconds) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_onRedAlliance = onRedAlliance;
        m_driverIsDriving = driverIsDriving;
        m_rumbleSeconds = rumbleSeconds;
        SmartDashboard.putString("Align/State", "Idle");
        SmartDashboard.putString("Align/Target", "none");
    }

    /** The pathfinder's practice run. Schedule once at boot. */
    public Command warmup() {
        return m_warmup;
    }

    /**
     * The command to bind to a button with {@code whileTrue}: drives to the closest spot in {@code group}.
     * <p>
     * {@code Commands.defer} means the plan is made at the moment the button is PRESSED (where are we now, which spot
     * is closest), not once when the robot boots, when we didn't know where we'd be yet.
     */
    public Command alignTo(TargetGroup group) {
        return Commands.defer(() -> buildAlign(group), Set.of(m_drivetrain)).withName("Align to " + group.name());
    }

    /** Makes the plan for one button press. Runs once, when the button is pressed. */
    private Command buildAlign(TargetGroup group) {
        // Stick clicked by accident while driving (the paddles share the stick-click buttons): quietly do nothing
        if (m_driverIsDriving.getAsBoolean()) {
            return Commands.none();
        }

        if (m_warmup.isScheduled()) {
            return giveUp(group.name(), "pathfinder still warming up after boot, try again in a few seconds");
        }

        // 1. CHECK: driving to a spot is only as good as knowing where we are. (Simulation has no camera, so skip it there.)
        if (!RobotBase.isSimulation() && !m_vision.seenTagRecently(kMaxSecondsSinceTag)) {
            return giveUp(group.name(), "no AprilTag seen in the last " + kMaxSecondsSinceTag + " s, position not trusted");
        }

        // 2. PICK the closest spot in the group for our alliance
        Pose2d robot = m_drivetrain.getPose();
        FieldTarget[] choices = m_onRedAlliance.getAsBoolean() ? group.red() : group.blue();
        FieldTarget target = null;
        double closestDistance = Double.POSITIVE_INFINITY;
        for (FieldTarget choice : choices) {
            Optional<Pose2d> pose = targetPose(choice, 0.0);
            if (pose.isEmpty()) {
                continue;
            }
            double distance = robot.getTranslation().getDistance(pose.get().getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                target = choice;
            }
        }
        if (target == null) {
            return giveUp(group.name(), "none of its tag IDs are on the field layout " + kFieldLayout);
        }

        // 3. TWO POSES from the tag: final (just short of the tag) and approach (straight back from it)
        Pose2d goal = targetPose(target, 0.0).get();
        Pose2d approach = targetPose(target, kApproachDistanceM).get();
        RobotLog.pose("Align/Goal", goal);
        RobotLog.pose("Align/Approach", approach);
        String targetName = target.name();
        SmartDashboard.putString("Align/Target", targetName + " (tag " + target.tagId() + ")");

        // 4. STAGE 1: PathPlanner finds a way around obstacles to the approach pose. Skipped if we're already close
        // AND roughly lined up in front, where driving straight in can't clip a corner of the target.
        // End speed 0 = arrive stopped, so stage 2 starts cleanly from the approach pose.
        boolean alreadyClose = closestDistance < kSkipPathfindingDistanceM && isLinedUpInFront(robot, goal);
        Command pathfind = alreadyClose ? Commands.none() : AutoBuilder.pathfindToPose(approach, m_constraints, 0.0);

        // 5. STAGE 2: straight in for accuracy
        Command finalApproach = new DriveToPose(m_drivetrain, goal);

        return Commands.sequence(
                Commands.runOnce(() -> start(targetName, alreadyClose)),
                pathfind,
                Commands.runOnce(() -> {
                    m_finalApproachStartTime = Timer.getFPGATimestamp();
                    setState("Final approach");
                }),
                finalApproach)
            // Checked every loop. If it returns true, everything above stops.
            .until(this::shouldStopEarly)
            .finallyDo(interrupted -> finish(targetName, interrupted));
    }

    /**
     * Where the robot should be to line up with a target.
     * <p>
     * An AprilTag's pose points OUT of its face: its +X arrow sticks straight out toward the field. So "stand in front
     * of the tag" = start at the tag, go out along its +X by half our robot length + a gap (+ the approach distance),
     * slide sideways by the side offset, and turn around to face back at it.
     *
     * @param extraOutM Extra distance out from the final spot (0 = the final spot, kApproachDistanceM = approach spot)
     * @return The pose in field coordinates, or empty if the tag isn't on this field
     */
    private Optional<Pose2d> targetPose(FieldTarget target, double extraOutM) {
        Optional<Pose3d> tag = m_field.getTagPose(target.tagId());
        if (tag.isEmpty()) {
            DriverStation.reportWarning("Auto-align: tag " + target.tagId() + " (" + target.name() + ") is not on " + kFieldLayout, false);
            return Optional.empty();
        }
        double outM = kRobotHalfLengthM + kBumperGapM + extraOutM;
        // Facing the tag with our FRONT = turned 180 deg from the way the tag faces. With our BACK = same way as the tag.
        Rotation2d facing = kFaceTargetWithFront ? Rotation2d.k180deg : Rotation2d.kZero;
        // The tag's +Y is to the LEFT of the tag's own view, which is to the RIGHT of someone looking at it, hence the minus.
        return Optional.of(tag.get().toPose2d().transformBy(new Transform2d(outM, -target.sideOffsetM(), facing)));
    }

    /**
     * Whether the robot is in the lane straight out in front of the goal: not off to the side by more than
     * kMaxSidewaysToSkipPathfindingM, and not already past the goal toward the tag.
     * {@code relativeTo} gives our position in the GOAL's own directions: +X = toward the tag, +Y = sideways.
     */
    private static boolean isLinedUpInFront(Pose2d robot, Pose2d goal) {
        Pose2d fromGoal = robot.relativeTo(goal);
        return fromGoal.getX() < kPositionToleranceM && Math.abs(fromGoal.getY()) < kMaxSidewaysToSkipPathfindingM;
    }

    /** Runs when an align starts: resets the early-stop checks */
    private void start(String targetName, boolean alreadyClose) {
        m_stopReason = null;
        m_finalApproachStartTime = Double.NaN;
        m_stuck.calculate(false);
        setState(alreadyClose ? "Final approach" : "Pathfinding");
        RobotLog.event("ALIGN start: " + targetName + (alreadyClose ? " (already close, straight in)" : " (pathfinding)"));
    }

    /** The early-stop checks, every loop. Sets m_stopReason and returns true to stop. */
    private boolean shouldStopEarly() {
        // Final approach started (NaN = not yet) and is taking too long: probably something is in the way
        if (!Double.isNaN(m_finalApproachStartTime)
                && Timer.getFPGATimestamp() - m_finalApproachStartTime > kFinalApproachTimeoutSeconds) {
            m_stopReason = "final approach took over " + kFinalApproachTimeoutSeconds + " s";
            return true;
        }
        if (m_driverIsDriving.getAsBoolean()) {
            m_stopReason = kDriverTookOver;
            return true;
        }
        if (m_stuck.calculate(isStuck())) {
            m_stopReason = "stuck (wheels told to move but not moving)";
            return true;
        }
        return false;
    }

    /** Stuck = the wheels are being told to go somewhere fast, but they're barely turning. Uses the average wheel. */
    private boolean isStuck() {
        var state = m_drivetrain.getState();
        double commanded = averageSpeed(state.ModuleTargets);
        double actual = averageSpeed(state.ModuleStates);
        return commanded > kStallCommandedSpeed && actual < kStallActualSpeed;
    }

    private static double averageSpeed(SwerveModuleState[] modules) {
        double sum = 0.0;
        for (SwerveModuleState module : modules) {
            sum += Math.abs(module.speedMetersPerSecond);
        }
        return modules.length > 0 ? sum / modules.length : 0.0;
    }

    /** Runs when an align ends, however it ended: tell the driver (rumble) and the logs what happened */
    private void finish(String targetName, boolean interrupted) {
        m_finalApproachStartTime = Double.NaN;
        if (interrupted || kDriverTookOver.equals(m_stopReason)) {
            // Button let go, stick moved (or something else took the drivetrain). The driver knows, no rumble needed.
            String why = interrupted ? "button let go" : kDriverTookOver;
            setState("Cancelled: " + why);
            RobotLog.event("ALIGN cancelled: " + targetName + ": " + why);
        } else if (m_stopReason != null) {
            setState("Gave up: " + m_stopReason);
            RobotLog.event("ALIGN gave up: " + targetName + ": " + m_stopReason);
            m_rumbleSeconds.accept(kFailedRumbleSeconds);
        } else {
            setState("Arrived: " + targetName);
            RobotLog.event("ALIGN arrived: " + targetName);
            m_rumbleSeconds.accept(kArrivedRumbleSeconds);
        }
    }

    /** A command that just reports why we can't align, and buzzes the controller */
    private Command giveUp(String groupName, String reason) {
        return Commands.runOnce(() -> {
            setState("Can't align: " + reason);
            RobotLog.event("ALIGN refused: " + groupName + ": " + reason);
            m_rumbleSeconds.accept(kFailedRumbleSeconds);
        });
    }

    private static void setState(String state) {
        SmartDashboard.putString("Align/State", state);
    }
}
