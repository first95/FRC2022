/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Like {@link JoystickButton}, but it checks for an analog axis being held
 * rather than a button press. This is primarily meant for triggers but you
 * could use it for a joystick axis just as easily.
 */
public class JoystickAxisButton extends JoystickButton {
    private final GenericHID m_joystick;
    private final int m_axisNum;
    private final double m_threshold;
    private static final double DEFAULT_THRESHOLD = 0.5;

    /**
     * Set up a joystick axis to act like a button.
     * 
     * @param joystick the joystick to use
     * @param axisNum the number of the analog axis
     * @param threshold if positive, this "button" will be considered "pressed" when the axis value is greater than this number. 
     *      If negative, this "button" will be considered "pressed" when the axis value is less than this number.
     */
    public JoystickAxisButton(GenericHID joystick, int axisNum, double threshold) {
        super(joystick, axisNum);
        m_joystick = joystick;
        m_axisNum = axisNum;
        m_threshold = threshold;
    }

    /**
     * Set up a joystick axis to act like a button.
     * This "button" will be considered "pressed" when this axis goes above DEFAULT_THRESHOLD
     * 
     * @param joystick the joystick to use
     * @param axisNum the number of the analog axis
     */
    public JoystickAxisButton(GenericHID joystick, int axisNum) {
        this(joystick, axisNum, DEFAULT_THRESHOLD);
    }

    /**
     * Gets the value of the joystick button.
     *
     * @return The value of the joystick button
     */
    @Override
    public boolean get() {
        if (m_threshold > 0) {
            return m_joystick.getRawAxis(m_axisNum) > m_threshold;
        } else {
            return m_joystick.getRawAxis(m_axisNum) < m_threshold;
        }
    }
}
