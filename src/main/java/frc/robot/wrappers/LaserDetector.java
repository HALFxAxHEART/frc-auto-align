package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalInput;

/** WHAT THIS FILE IS: a laser/beam-break sensor on a roboRIO DIO port (e.g. "is there a game piece in the intake?"). Wraps {@link DigitalInput}. */
public class LaserDetector {
    private DigitalInput laser;

    /** Constructs a LaserDetector object 
     * @param port The DIO port being used by the laser
     */
    public LaserDetector(int port) {
        laser = new DigitalInput(port);
    }

    /** @return True if something is blocking the laser */
    public boolean isBlocked() { return !laser.get(); }

    /** @return True if nothing is blocking the laser */
    public boolean isOpen() { return laser.get(); }
}
