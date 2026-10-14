// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TorqueNados - FRC 5090

package frc.robot;

import static frc.robot.Constants.DriveConstants.kMaxAngularRate;
import static frc.robot.Constants.DriveConstants.kMaxSpeed;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutonContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * WHAT THIS FILE IS: where the robot is put together.
 * <ul>
 *   <li>Makes every subsystem (drivetrain, vision, and later your mechanisms). Each one exists ONCE, here.</li>
 *   <li>Binds controller buttons to commands in {@link #configureBindings()}. Want to know what a button does? Look there.</li>
 *   <li>Sets each subsystem's DEFAULT command (what it does when nothing else is using it, e.g. drive with the sticks).</li>
 *   <li>Builds the auto chooser shown on the dashboard.</li>
 * </ul>
 * Button cheat sheet: {@code whileTrue} = runs while held, stops on release. {@code onTrue} = starts once on press.
 * {@code toggleOnTrue} = press to start, press again to stop.
 */
public class RobotContainer {
    // --- CONTROLLERS ---
    private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverPort);

    // --- SUBSYSTEMS ---
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision vision = new Vision(drivetrain);

    // --- SWERVE REQUESTS ---
    // Field-centric: pushing the stick away from you always drives away from you, whichever way the robot faces.
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxSpeed * OperatorConstants.kDeadband)
        .withRotationalDeadband(kMaxAngularRate * OperatorConstants.kDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final Telemetry logger = new Telemetry(kMaxSpeed);

    // --- AUTON ---
    // Must be made AFTER the subsystems, since it hands them to PathPlanner
    private final AutonContainer auton = new AutonContainer(this);
    private final SendableChooser<Command> autonChooser = auton.buildAutonChooser();

    // --- AUTO-ALIGN (hold a paddle to drive to the hub / outpost) ---
    private final AutoAlign autoAlign = new AutoAlign(
        drivetrain, vision, this::onRedAlliance, this::driverIsDriving, this::rumbleDriver);

    private double m_brownoutRumble = 0.0;
    /** Time until which the controller buzzes for auto-align feedback (see rumbleDriver) */
    private double m_feedbackRumbleUntil = 0.0;

    // This is the Constructor. It runs once when the robot boots up.
    public RobotContainer() {
        SmartDashboard.putData("Auton Selector", autonChooser);
        configureBindings();
    }

    /** @return Whether the robot is on the red alliance or not. */
    public boolean onRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }

    // This wires the physical Xbox controller to the robot's commands.
    private void configureBindings() {
        // DRIVE: left stick moves, right stick turns. Hold the right trigger for slow mode.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double multiplier = driver.getRightTriggerAxis() > OperatorConstants.kSlowModeTrigger ? OperatorConstants.kSlowModeMultiplier : 1.0;
                return drive.withVelocityX(-driver.getLeftY() * kMaxSpeed * multiplier)
                            .withVelocityY(-driver.getLeftX() * kMaxSpeed * multiplier)
                            .withRotationalRate(-driver.getRightX() * kMaxAngularRate * multiplier);
            })
        );

        // Let the wheels coast to a stop while disabled
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // X: lock the wheels in an X so the robot is hard to push
        driver.x().whileTrue(drivetrain.applyRequest(() -> brake));
        // Start (menu): reset field-centric "forward" to the way the robot is facing right now
        driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // AUTO-ALIGN: HOLD to drive to the spot, let go to stop. Moving a stick also stops it.
        // The back buttons (paddles) on our 8BitDo controller aren't buttons of their own: the controller copies them
        // onto other buttons. We set them to copy the STICK CLICKS (see AUTO-ALIGN-GUIDE.md "Set up the paddles"), so:
        //   left back button  = left stick click  -> OUTPOST
        //   right back button = right stick click -> HUB
        // Each one goes to OUR alliance's spot: AutoAlign checks the Driver Station's alliance color on every press.
        // Pressing a stick in by accident while driving is harmless: the stick is moving, which cancels it at once.
        driver.leftStick().whileTrue(autoAlign.alignTo(AlignConstants.kOutpost));
        driver.rightStick().whileTrue(autoAlign.alignTo(AlignConstants.kHub));
        // A: the depot, an example of a spot given by X/Y coordinates instead of an AprilTag
        driver.a().whileTrue(autoAlign.alignTo(AlignConstants.kDepot));
        // Practice pathfind at boot so the first real press responds right away (runs while disabled too)
        CommandScheduler.getInstance().schedule(autoAlign.warmup());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // The FMS calls this right before Auto starts to ask what sequence to run.
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    /** @return Whether the driver is pushing either drive stick past kDriverOverrideStick (used to cancel auto-align) */
    public boolean driverIsDriving() {
        double threshold = AlignConstants.kDriverOverrideStick;
        return Math.hypot(driver.getLeftX(), driver.getLeftY()) > threshold || Math.abs(driver.getRightX()) > threshold;
    }

    /** Buzzes the driver's controller at full strength for {@code seconds} (feedback that doesn't need eyes on a screen) */
    public void rumbleDriver(double seconds) {
        m_feedbackRumbleUntil = Math.max(m_feedbackRumbleUntil, Timer.getFPGATimestamp() + seconds);
    }

    /** Rumbles the driver controller harder the closer the battery gets to browning out. Call every loop. */
    public void updateBrownoutRumble() {
        double target = 0.0;
        if (DriverStation.isEnabled()) {
            if (RobotController.isBrownedOut()) {
                target = 1.0;
            } else {
                double volts = RobotController.getBatteryVoltage();
                double brownoutVolts = RobotController.getBrownoutVoltage();
                target = MathUtil.clamp(
                    (OperatorConstants.kRumbleStartVolts - volts) / (OperatorConstants.kRumbleStartVolts - brownoutVolts),
                    0.0, 1.0);
            }
        }
        m_brownoutRumble = Math.max(target, m_brownoutRumble - OperatorConstants.kRumbleDecayPerLoop);
        // Auto-align feedback buzz wins over a weaker brownout rumble
        double feedback = Timer.getFPGATimestamp() < m_feedbackRumbleUntil ? 1.0 : 0.0;
        driver.getHID().setRumble(RumbleType.kBothRumble, Math.max(m_brownoutRumble, feedback));
        SmartDashboard.putNumber("Brownout Rumble", m_brownoutRumble);
    }
}
