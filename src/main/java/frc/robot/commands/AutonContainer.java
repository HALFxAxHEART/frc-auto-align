package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotLog;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * WHAT THIS FILE IS: everything for autonomous (the first 15-20 seconds when the robot drives itself).
 * <ul>
 *   <li>Connects PathPlanner to the drivetrain ({@code AutoBuilder.configure}).</li>
 *   <li>Registers "named commands": robot actions (intake, shoot...) that PathPlanner autos can call by name.</li>
 *   <li>Builds the auto chooser (the dropdown on the dashboard the drive team picks before a match).</li>
 * </ul>
 * Paths and autos themselves are drawn in the PathPlanner app and saved in src/main/deploy/pathplanner/.
 * To add an auto: draw it in PathPlanner, then add one {@code chooser.addOption(...)} line in buildAutonChooser().
 */
public class AutonContainer {
    private final RobotContainer robotContainer;
    private final CommandSwerveDrivetrain drivetrain;

    /** Constructs an AutonContainer object */
    public AutonContainer(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.drivetrain;
        // Named commands must be registered BEFORE any auto is built, or PathPlanner can't find them
        registerNamedCommands();

        // Load the robot config saved by the PathPlanner app (Settings -> Robot Config).
        // Fall back onto the config in Constants because it's better than crashing.
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner settings.json failed to load, using fallback config: " + e.getMessage(), e.getStackTrace());
            config = AutoConstants.kFallbackConfig;
        }

        // Logged so AdvantageScope can overlay the planned path / target pose on the actual pose
        PathPlannerLogging.setLogTargetPoseCallback(pose -> RobotLog.pose("Auto target", pose));
        PathPlannerLogging.setLogActivePathCallback(poses -> RobotLog.poses("Auto path", poses));

        AutoBuilder.configure(
            drivetrain::getPose,
            drivetrain::resetPose,
            drivetrain::getChassisSpeeds,
            drivetrain::driveRobotRelative,
            new PPHolonomicDriveController(AutoConstants.kTranslationPID, AutoConstants.kRotationPID),
            config,
            // Paths are drawn on the blue side; flip them when we're red
            robotContainer::onRedAlliance,
            drivetrain
        );
    }

    /**
     * Named commands are how PathPlanner autos run robot code (intake, shoot, etc.).
     * The name here must match the name typed into the PathPlanner app exactly.
     */
    private void registerNamedCommands() {
        register("Do Nothing", doNothing());
        // Lock the wheels in an X at the end of an auto so we don't get pushed
        register("X Brake", drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()).withTimeout(AutoConstants.kXBrakeSeconds));
    }

    /** Registers a named command and logs an event each time an auto runs it */
    private void register(String name, Command command) {
        NamedCommands.registerCommand(name, command.beforeStarting(() -> RobotLog.event("AUTO named command: " + name)));
    }

    public SendableChooser<Command> buildAutonChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();
        chooser.setDefaultOption("Do Nothing", doNothing());
        // The string must match the .auto file name in deploy/pathplanner/autos (without ".auto")
        chooser.addOption("Drive Forward", AutoBuilder.buildAuto("Drive Forward"));
        return chooser;
    }

    /** Auton that does nothing */
    public Command doNothing() {
        return Commands.none().withName("Do Nothing");
    }
}
