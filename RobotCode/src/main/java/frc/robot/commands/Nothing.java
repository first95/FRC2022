package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Nothing extends CommandBase {

	@Override
	public boolean isFinished() {
		// This move sits and waits forever.
        // This is actually really useful at the end of certain moves.
        // If you want a commandgroup to run as a JoystickButton.whileHeld(), 
        // it will repeat while the button is held, unless you end that commandgroup
        // with one of these.
		return false;
	}

}
