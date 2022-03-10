package frc.robot.commands.autocommands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Drivebase;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;

public class OneCargoAuto extends SequentialCommandGroup {
    public OneCargoAuto(DriveBase drivebase, LimeLight limelightport) {
        TrajectoryConfig config = new TrajectoryConfig(Drivebase.MAX_SPEED_MPS, Drivebase.MAX_ACCELERATION_MPSPS);
        config.setKinematics(Drivebase.DRIVE_KINEMATICS);
        config.setReversed(true);
        Trajectory back = new Trajectory();
        back = TrajectoryGenerator.generateTrajectory(
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
            List.of(),
            new Pose2d(new Translation2d(-2, 0), new Rotation2d(0)),
             config);

        addRequirements(drivebase);
        addRequirements(limelightport);

        addCommands(new AutoAim(true, drivebase, limelightport).withTimeout(2));
        addCommands(new FollowTrajectory(drivebase, back));
    }
}