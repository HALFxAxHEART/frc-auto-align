package frc.robot.commands;

import static frc.robot.Constants.AlignConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * WHAT THIS FILE IS: drives the robot in a straight line to an exact spot on the field, then stops.
 * Used for the LAST short stretch of an auto-align, after PathPlanner has brought us close (see AutoAlign.java).
 * <p>
 * How it works, every 20 ms:
 * <ol>
 *   <li>Where are we? ({@code drivetrain.getPose()}, kept correct by wheel odometry + the Limelight)</li>
 *   <li>How far and which way is the goal? That's the ERROR.</li>
 *   <li>Speed = kP x distance left, capped at a max speed. Far away = fast, close = slow, there = stop.
 *       That's a "P controller", the P in PID. Speed is also only allowed to grow a little each loop
 *       (an acceleration limit) so the wheels don't slip when it starts, and never drops below a small minimum
 *       until we've arrived (or friction would stop it a few cm short).</li>
 *   <li>Same idea for turning: turn rate = kP x how many degrees we're off.</li>
 * </ol>
 * It finishes once we've been inside the tolerance (a few cm and degrees) for a short moment.
 * <p>
 * It does NOT avoid obstacles: it drives straight at the goal. That's why AutoAlign uses it only for the last
 * ~0.75 m, from an approach spot straight out in front of the target.
 */
public class DriveToPose extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Pose2d m_goal;

    // Field-centric request where +X ALWAYS means "toward the red wall", whatever alliance we're on.
    // (The drivers' version flips for red so "stick forward" is away from them. Field positions don't flip.)
    // Velocity = closed-loop wheel speed, which is more exact than open-loop voltage at the low speeds we use here.
    private final SwerveRequest.FieldCentric m_request = new SwerveRequest.FieldCentric()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity);

    /** Waits until we've been in tolerance for kSettleSeconds in a row */
    private final Debouncer m_settled = new Debouncer(kSettleSeconds);
    private boolean m_arrived = false;
    private double m_lastSpeed = 0.0;

    /**
     * @param drivetrain The swerve drive
     * @param goal       Where to end up, in field coordinates (blue-origin, meters), including which way to face
     */
    public DriveToPose(CommandSwerveDrivetrain drivetrain, Pose2d goal) {
        m_drivetrain = drivetrain;
        m_goal = goal;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_settled.calculate(false);
        m_arrived = false;
        // Start the acceleration limit from however fast we're already going (we may arrive from PathPlanner moving)
        var speeds = m_drivetrain.getChassisSpeeds();
        m_lastSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    @Override
    public void execute() {
        Pose2d pose = m_drivetrain.getPose();

        // --- DRIVING: error = arrow from us to the goal ---
        Translation2d error = m_goal.getTranslation().minus(pose.getTranslation());
        double distance = error.getNorm();
        double speed = Math.min(kFinalTranslationP * distance, kFinalMaxSpeed);
        // Close to the goal, kP x distance gets so small the robot can't overcome friction and stops just short.
        // So until we're inside the tolerance, always ask for at least a small minimum speed.
        if (distance > kPositionToleranceM) {
            speed = Math.max(speed, kFinalMinSpeed);
        }
        speed = Math.min(speed, m_lastSpeed + kFinalMaxAccel * 0.020); // accel limit, 0.020 s per loop
        m_lastSpeed = speed;
        // Point that speed along the error arrow (divide by length to get a 1 m "direction" arrow first)
        double vx = distance > 1e-6 ? error.getX() / distance * speed : 0.0;
        double vy = distance > 1e-6 ? error.getY() / distance * speed : 0.0;

        // --- TURNING: error = how far our heading is from the goal heading, -180..180 deg ---
        double headingError = m_goal.getRotation().minus(pose.getRotation()).getRadians();
        double turnRate = MathUtil.clamp(kFinalRotationP * headingError, -kFinalMaxTurnRadPerSec, kFinalMaxTurnRadPerSec);
        // Same minimum idea for turning (copysign = the minimum, with the same + or - direction as turnRate)
        if (Math.abs(Units.radiansToDegrees(headingError)) > kHeadingToleranceDeg) {
            turnRate = Math.copySign(Math.max(Math.abs(turnRate), kFinalMinTurnRadPerSec), turnRate);
        }

        m_drivetrain.setControl(m_request.withVelocityX(vx).withVelocityY(vy).withRotationalRate(turnRate));

        boolean inTolerance = distance < kPositionToleranceM
            && Math.abs(Units.radiansToDegrees(headingError)) < kHeadingToleranceDeg;
        m_arrived = m_settled.calculate(inTolerance);

        SmartDashboard.putNumber("Align/Distance left (m)", distance);
        SmartDashboard.putNumber("Align/Heading error (deg)", Units.radiansToDegrees(headingError));
    }

    @Override
    public boolean isFinished() {
        return m_arrived;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop. (If the button is still held, nothing else drives until the driver's default command takes over.)
        m_drivetrain.setControl(m_request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}
