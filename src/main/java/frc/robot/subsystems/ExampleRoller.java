package frc.robot.subsystems;

import static frc.robot.Constants.ExampleRollerConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

/**
 * WHAT THIS FILE IS: an example mechanism (one roller on one Kraken/Falcon) showing how we write a CTRE subsystem.
 * It is NOT created in RobotContainer, so it does nothing until you add it. Copy this file to start a real
 * mechanism (intake, shooter, conveyor...).
 * <p>
 * The pattern:
 * <ol>
 *   <li>Make the motor and give it a config (current limits, brake/coast, direction, PID gains).</li>
 *   <li>Make the control request ONCE and reuse it (making a new one every loop wastes memory).</li>
 *   <li>Write small methods that do one thing ({@link #setSpeed}, {@link #stop}).</li>
 *   <li>Write methods that return Commands so RobotContainer can bind them to buttons ({@link #runCommand}).</li>
 *   <li>Put useful numbers on the dashboard in {@link #periodic()} (they get logged automatically too).</li>
 * </ol>
 * To use it, add this line in RobotContainer: {@code public final ExampleRoller roller = new ExampleRoller();}
 * and bind it, e.g. {@code driver.leftBumper().whileTrue(roller.runCommand(ExampleRollerConstants.kIntakeRps));}
 */
public class ExampleRoller extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(CANIds.kExampleRollerMotor, new CANBus(kCANBus));

    // Velocity control: the motor holds a speed (rotations per second) using PID + feedforward.
    // FOC = "field oriented control", a smoother, stronger way to drive the motor. Needs Phoenix Pro (we have it).
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    // Neutral = stop driving the motor (it brakes or coasts depending on NeutralMode)
    private final NeutralOut m_stopRequest = new NeutralOut();

    private double m_targetRps = 0;

    public ExampleRoller() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Current limits protect the motor, the mechanism and the battery. ALWAYS set them.
        config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // Coast = spins freely when stopped. Use Brake for arms/elevators that must hold still.
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // Flip this if positive speed spins the wrong way
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // Slot 0 gains, used by m_velocityRequest (.withSlot(0))
        config.Slot0.kP = kP;
        config.Slot0.kV = kV;
        // apply() sends the config to the motor. It returns an error code if the motor isn't found.
        var status = m_motor.getConfigurator().apply(config);
        if (!status.isOK()) {
            System.err.println("ExampleRoller: motor config failed: " + status);
        }
    }

    /** Spin at a speed, in motor rotations per second */
    public void setSpeed(double rps) {
        m_targetRps = rps;
        m_motor.setControl(m_velocityRequest.withVelocity(rps));
    }

    /** Stop the motor */
    public void stop() {
        m_targetRps = 0;
        m_motor.setControl(m_stopRequest);
    }

    /** @return Motor speed right now, rotations per second */
    public double getSpeed() {
        return m_motor.getVelocity().getValueAsDouble();
    }

    /** @return Whether the roller is within tolerance of its target speed */
    public boolean atSpeed() {
        return Math.abs(getSpeed() - m_targetRps) < kToleranceRps;
    }

    /** A command that spins the roller while it runs and stops it when it ends (e.g. button released) */
    public Command runCommand(double rps) {
        return startEnd(() -> setSpeed(rps), this::stop).withName("ExampleRoller run " + rps);
    }

    @Override
    public void periodic() {
        // Runs every 20 ms. Dashboard values are also saved to the log.
        SmartDashboard.putNumber("ExampleRoller/Target RPS", m_targetRps);
        SmartDashboard.putNumber("ExampleRoller/Actual RPS", getSpeed());
        SmartDashboard.putBoolean("ExampleRoller/At speed", atSpeed());
    }
}
