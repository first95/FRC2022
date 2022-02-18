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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.DriveBase;

public class FollowTrajectory extends SequentialCommandGroup {
    public FollowTrajectory(DriveBase drivebase, Trajectory trajectory) {
        addRequirements(drivebase);
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
