package frc.robot.commands.drivebase;

import java.io.Console;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class ManuallyControlDrivebase extends CommandBase {

    private DriveBase m_DriveBase;
    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public ManuallyControlDrivebase(DriveBase driveBase) {
        m_DriveBase = driveBase;
        addRequirements(driveBase);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        final var xSpeed = -m_speedLimiter
                .calculate(RobotContainer.oi.getForwardAxis()) * Constants.MAX_SPEED_MPS;
        final var rot = -m_rotLimiter.calculate(RobotContainer.oi.getTurnAxis()) * Constants.MAX_SPEED_MPS;
        m_DriveBase.drive(xSpeed, rot);
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