package frc.robot.commands.autoCommands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
