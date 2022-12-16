package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.SwerveBase;

public class FollowTrajectory extends CommandBase{
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final RamseteController follower = new RamseteController(Drivebase.RAMSETE_B, Drivebase.RAMSETE_ZETA);
    private SwerveBase drivebase;
    private double prevTime;
    
    public FollowTrajectory(SwerveBase drivebase, Trajectory trajectory) {
        addRequirements(drivebase);
        this.drivebase = drivebase;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        prevTime = -1;
        var initialState = trajectory.sample(0);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double curTime = timer.get();
        double dt = curTime - prevTime;

        if (prevTime < 0) {
            drivebase.drive(
                new Translation2d(),
                0,
                false,
                false);
            prevTime = curTime;
            return;
        }

        var targetVelocity = follower.calculate(drivebase.getPose(), trajectory.sample(curTime));
        drivebase.drive(
            new Translation2d(
                targetVelocity.vxMetersPerSecond,
                targetVelocity.vyMetersPerSecond),
            targetVelocity.omegaRadiansPerSecond,
            false,
            false);
        prevTime = curTime;

    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        if (interrupted) {
            drivebase.drive(
                new Translation2d(),
                0,
                false,
                false);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
        
}
