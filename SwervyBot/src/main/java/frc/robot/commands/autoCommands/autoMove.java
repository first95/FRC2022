package frc.robot.commands.autoCommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.SwerveBase;

public class autoMove extends SequentialCommandGroup{
    public autoMove(SwerveBase swerve) {
        addRequirements(swerve);

        PathPlannerTrajectory example = PathPlanner.loadPath("New Path",
            new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION));
        
        addCommands(new FollowTrajectory(swerve, example, true));
    }
}
