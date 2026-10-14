package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalInput;

/** WHAT THIS FILE IS: a limit switch on a roboRIO DIO port (e.g. "has the elevator reached the bottom?"). Wraps {@link DigitalInput}. */
public class LimitSwitch {
    private DigitalInput limSwitch;
    private boolean inverted;

     /** Constructs LimitSwitch object 
     * @param port The number of the DIO port being used
     */
    public LimitSwitch(int port) {
        limSwitch = new DigitalInput(port);
    }

    /** @return true if the switch is being pressed */
    public boolean isPressed() { return limSwitch.get() ^ inverted; }

    /** @return true if the switch is not being pressed */
    public boolean isNotPressed() { return !limSwitch.get() ^ inverted; }

    /** Inverts the outputs from the limit switch */
    public void setInverted(boolean invert) { inverted = invert; }
}