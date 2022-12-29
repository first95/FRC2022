package frc.robot.commands.autoCommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.SwerveBase;

public class autoMove extends SequentialCommandGroup{
    public autoMove(SwerveBase swerve) {
        addRequirements(swerve);

        PathPlannerTrajectory example = PathPlanner.loadPath("Example Path",
            new PathConstraints(Drivebase.MAX_SPEED, Drivebase.MAX_ACCELERATION));
        
        addCommands(new FollowTrajectory(swerve, example, true));
    }
}
