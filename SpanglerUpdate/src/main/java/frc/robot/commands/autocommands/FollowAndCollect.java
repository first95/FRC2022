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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.CollectColoredBall;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;

public class FollowAndCollect extends SequentialCommandGroup {
        public FollowAndCollect(DriveBase drivebase, LimeLight limelight) {
                Subsystem[] subsystems = { drivebase, limelight };
                addRequirements(subsystems);

                TrajectoryConfig config = new TrajectoryConfig(Constants.MAX_SPEED_MPS,
                                Constants.MAX_ACCELERATION_MPSPS)
                                                .setKinematics(Constants.DRIVE_KINEMATICS);

                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(new Translation2d(1, -1)),
                                new Pose2d(3, -3, new Rotation2d(180)),
                                config);

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

                CollectColoredBall testVision = new CollectColoredBall(drivebase, limelight);

                drivebase.resetOdometry(exampleTrajectory.getInitialPose());

                addCommands(ramseteCommand);
                addCommands(testVision);
        }
}
