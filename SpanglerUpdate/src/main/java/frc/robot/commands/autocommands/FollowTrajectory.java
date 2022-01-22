package frc.robot.commands.autocommands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class FollowTrajectory extends SequentialCommandGroup {
    public FollowTrajectory(DriveBase drivebase, Trajectory trajectory) {
        addRequirements(drivebase);
        
        //TrajectoryConfig config =
        //    new TrajectoryConfig(Constants.MAX_SPEED_MPS, Constants.MAX_ACCELERATION_MPSPS)
        //        .setKinematics(Constants.DRIVE_KINEMATICS);
        
        //Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //    new Pose2d(0, 0, new Rotation2d(0)),
        //    List.of(new Translation2d(1, 1), new Translation2d(2, -1)), 
        //    new Pose2d(3, 0, new Rotation2d(0)),
        //    config);
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            drivebase::getPose,
            new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            new SimpleMotorFeedforward(Constants.KS, Constants.KV, Constants.KA),
            Constants.DRIVE_KINEMATICS,
            drivebase::getWheelSpeeds,
            new PIDController(Constants.KP, 0, 0),
            new PIDController(Constants.KP, 0, 0),
            drivebase::tankDriveVolts,
            drivebase);
        
        drivebase.resetOdometry(trajectory.getInitialPose());

        addCommands(ramseteCommand);
    }
}
