package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Pause extends WaitCommand {
	// This move waits and does nothing for a specified duration
	public Pause(double pauseDurationS) {
		super(pauseDurationS);
    }
    
    @Override
    public void initialize() {
        System.out.println("Pause.start()");
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Pause.end()");
        super.end(interrupted);
    }
}
