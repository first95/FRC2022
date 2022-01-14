package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class ManuallyControlDrivebase extends CommandBase {

    private DriveBase m_DriveBase;

    public ManuallyControlDrivebase(DriveBase driveBase) {
        m_DriveBase = driveBase;
        addRequirements(driveBase);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_DriveBase.driveWithJoysticks();
        m_DriveBase.SetSuckerPower(RobotContainer.oi.getSuckerAxis());
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