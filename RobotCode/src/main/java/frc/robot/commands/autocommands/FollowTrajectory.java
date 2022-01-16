package frc.robot.commands.autocommands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class FollowTrajectory extends SequentialCommandGroup {
    public FollowTrajectory(DriveBase drivebase) {
        addRequirements(drivebase);

        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.KS, Constants.KV, Constants.KA), Constants.DRIVE_KINEMATICS, 10);
        //Generate trajectory config
        TrajectoryConfig config =
        new TrajectoryConfig(Constants.MAX_SPEED_MPS, Constants.MAX_ACCELERATION_MPSPS)
            .setKinematics(Constants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint);
        //Generate a trajectory (replace with import)
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(2, 1, new Rotation2d(-90)), 
        config);
        
        //Create Ramsete follower:
        RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            drivebase::getPose,
            new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            new SimpleMotorFeedforward(Constants.KS, Constants.KV, Constants.KA),
            Constants.DRIVE_KINEMATICS,
            drivebase::getWheelSpeeds,
            new PIDController(Constants.KP, 0, 0),
            new PIDController(Constants.KP, 0, 0),
            drivebase::tankDriveVolts,
            drivebase);

        //Set Robot starting position
        drivebase.resetOdometry(exampleTrajectory.getInitialPose());

        addCommands(ramseteCommand);
    }    
}
