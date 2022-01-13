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
 * Like {@link JoystickButton}, but it checks for the PoV hat rather than a button press.
 */
public class JoystickPovButton extends JoystickButton {
  private final GenericHID m_joystick;
  private final int m_povDirDeg;

  /**
   * Create a joystick button for triggering commands.
   *
   * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
   *                     etc)
   * @param povDirDeg    The PoV hat direction, in degrees
   */
  public JoystickPovButton(GenericHID joystick, int povDirDeg) {
    super(joystick, povDirDeg);
    m_joystick = joystick;
    m_povDirDeg = povDirDeg;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean get() {
    return m_joystick.getPOV() == m_povDirDeg;
  }
}
