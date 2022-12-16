package frc.robot.commands.autoCommands;

import java.util.List;

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

        TrajectoryConfig config = 
            new TrajectoryConfig(Drivebase.MAX_SPEED, Drivebase.MAX_ACCELERATION);
        
            Trajectory example = TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d()), config);
            swerve.field.getObject("traj").setTrajectory(example);
            swerve.resetOdometry(example.getInitialPose());
            addCommands(new FollowTrajectory(swerve, example));
    }
}
