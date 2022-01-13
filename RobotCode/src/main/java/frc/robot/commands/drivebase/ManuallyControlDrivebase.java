package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class ManuallyControlDrivebase extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveBase m_drivebase;

    public ManuallyControlDrivebase(DriveBase driveBase) {
        m_drivebase = driveBase;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveBase);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_drivebase.driveWithJoysticks();
    }
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        
    }
}