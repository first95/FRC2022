package frc.robot.commands.autocommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Drivebase;
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
            new RamseteController(Drivebase.RAMSETE_B, Drivebase.RAMSETE_ZETA),
            new SimpleMotorFeedforward(Drivebase.KS, Drivebase.KV, Drivebase.KA),
            Drivebase.DRIVE_KINEMATICS,
            drivebase::getWheelSpeeds,
            new PIDController(Drivebase.KP, 0, 0),
            new PIDController(Drivebase.KP, 0, 0),
            drivebase::tankDriveVolts,
            drivebase);
        

        addCommands(new InstantCommand(() -> {drivebase.resetOdometry(trajectory.getInitialPose());})
            .andThen(ramseteCommand.andThen(() -> drivebase.tankDriveVolts(0, 0))));
    }
}
