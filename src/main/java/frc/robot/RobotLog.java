package frc.robot;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * WHAT THIS FILE IS: everything the robot records to the USB stick, so you can find out what happened after a match.
 * Season-independent logging. Copy this file into next year's project, call {@link #start} first thing in
 * Robot() and {@link #periodic} every loop. See LOGGING.md for what to log each season and why.
 *
 * <ul>
 *   <li>WPILib DataLog: every NetworkTables value (all dashboard values) + joysticks, buttons, enable state,
 *       match info and console output, to a .wpilog on the USB stick in the roboRIO (falls back to the RIO).</li>
 *   <li>CTRE SignalLogger: every CTRE device (current, voltage, velocity, temps, faults) to .hoot files.</li>
 *   <li>Power: battery, brownouts, total current and every PDH channel.</li>
 *   <li>CAN bus utilization (RIO bus + CANivores) and loop-time overruns.</li>
 *   <li>Command start / end / interrupt, as text in "Commands".</li>
 *   <li>Short text events (rising edges only) in "Events".</li>
 *   <li>Poses as structs so AdvantageScope draws them on the field.</li>
 * </ul>
 * Open everything in AdvantageScope (only as a viewer, no AdvantageKit needed).
 */
public final class RobotLog {
    private RobotLog() {}

    /** A loop slower than this counts as an overrun (TimedRobot runs every 20 ms) */
    private static final double kLoopOverrunSeconds = 0.025;

    private static StringLogEntry s_commands;
    private static StringLogEntry s_events;
    private static PowerDistribution s_pdh;
    private static CANBus[] s_canivores = new CANBus[0];
    private static double s_lastLoopTime = 0.0;
    private static final Map<String, StructPublisher<Pose2d>> s_poses = new HashMap<>();
    private static final Map<String, StructArrayPublisher<Pose2d>> s_poseArrays = new HashMap<>();
    private static final Map<String, Boolean> s_lastState = new HashMap<>();

    /**
     * Call once, first thing in Robot() (before any subsystem is made).
     * @param canivoreNames Names of any CANivore buses to report utilization for, e.g. "Jarvis"
     */
    public static void start(String... canivoreNames) {
        DataLogManager.start();   // picks the USB stick (/u) automatically when one is plugged in
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog());

        // CTRE logs: USB stick if present, otherwise CTRE's default location on the RIO
        boolean usbStick = new File("/u").canWrite();
        if (usbStick) {
            SignalLogger.setPath("/u/ctre-logs/");
        } else if (RobotBase.isReal()) {
            DriverStation.reportWarning("No USB stick in the roboRIO: logs are going to the RIO's small internal storage", false);
        }
        SmartDashboard.putBoolean("Logging/USB stick", usbStick);
        SignalLogger.start();

        s_commands = new StringLogEntry(DataLogManager.getLog(), "Commands");
        s_events = new StringLogEntry(DataLogManager.getLog(), "Events");

        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(cmd -> s_commands.append("START " + cmd.getName()));
        scheduler.onCommandFinish(cmd -> s_commands.append("END " + cmd.getName()));
        scheduler.onCommandInterrupt((cmd, interruptor) -> s_commands.append("INTERRUPTED " + cmd.getName()
            + interruptor.map(by -> " by " + by.getName()).orElse("")));

        // Power: putData publishes voltage, total current and every channel, so they all end up in the log
        try {
            s_pdh = new PowerDistribution();
            SmartDashboard.putData("Power/PDH", s_pdh);
        } catch (RuntimeException e) {
            DriverStation.reportError("No PDH/PDP found, breaker currents won't be logged: " + e.getMessage(), false);
        }

        s_canivores = new CANBus[canivoreNames.length];
        for (int i = 0; i < canivoreNames.length; i++) {
            s_canivores[i] = new CANBus(canivoreNames[i]);
        }
    }

    /** Call every loop, in robotPeriodic(). Logs power, CAN usage and loop time, plus events when they go bad. */
    public static void periodic() {
        double now = Timer.getFPGATimestamp();
        double loopSeconds = s_lastLoopTime == 0.0 ? 0.0 : now - s_lastLoopTime;
        s_lastLoopTime = now;
        SmartDashboard.putNumber("Logging/Loop time (ms)", loopSeconds * 1000);
        eventOnRise("loopOverrun", loopSeconds > kLoopOverrunSeconds,
            String.format("LOOP OVERRUN %.0f ms", loopSeconds * 1000));

        double volts = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("Power/Battery (V)", volts);
        eventOnRise("brownout", RobotController.isBrownedOut(), String.format("BROWNOUT (battery %.2f V)", volts));

        CANStatus rioCan = RobotController.getCANStatus();
        SmartDashboard.putNumber("CAN/rio/Utilization (%)", rioCan.percentBusUtilization * 100);
        SmartDashboard.putNumber("CAN/rio/TX errors", rioCan.transmitErrorCount);
        SmartDashboard.putNumber("CAN/rio/RX errors", rioCan.receiveErrorCount);
        for (CANBus bus : s_canivores) {
            var status = bus.getStatus();
            SmartDashboard.putNumber("CAN/" + bus.getName() + "/Utilization (%)", status.BusUtilization * 100);
            SmartDashboard.putNumber("CAN/" + bus.getName() + "/TX errors", status.TEC);
            SmartDashboard.putNumber("CAN/" + bus.getName() + "/RX errors", status.REC);
            eventOnRise("canError " + bus.getName(), !status.Status.isOK(),
                "CAN bus " + bus.getName() + " problem: " + status.Status);
        }
    }

    /** A one-off text event, e.g. "Brownout" or "Intake homed" */
    public static void event(String message) {
        if (s_events != null) {
            s_events.append(message);
        }
        DataLogManager.log(message);
    }

    /** Logs an event only when the condition turns true (not every loop it stays true) */
    public static void eventOnRise(String key, boolean condition, String message) {
        boolean last = s_lastState.getOrDefault(key, false);
        if (condition && !last) {
            event(message);
        }
        s_lastState.put(key, condition);
    }

    /** Publishes a pose that AdvantageScope can draw on the field (also recorded in the log) */
    public static void pose(String name, Pose2d pose) {
        s_poses.computeIfAbsent(name, n ->
            NetworkTableInstance.getDefault().getStructTopic("Poses/" + n, Pose2d.struct).publish()).set(pose);
    }

    /** Publishes a list of poses (e.g. the active auto path) */
    public static void poses(String name, List<Pose2d> poses) {
        s_poseArrays.computeIfAbsent(name, n ->
            NetworkTableInstance.getDefault().getStructArrayTopic("Poses/" + n, Pose2d.struct).publish())
            .set(poses.toArray(new Pose2d[0]));
    }
}
