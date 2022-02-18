package frc.robot.commands.autocommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class FollowTrajectory extends SequentialCommandGroup {
    public FollowTrajectory(DriveBase drivebase, Trajectory trajectory) {
        addRequirements(drivebase);

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
        

        addCommands(new InstantCommand(() -> {drivebase.resetOdometry(trajectory.getInitialPose());})
            .andThen(ramseteCommand.andThen(() -> drivebase.tankDriveVolts(0, 0))));
    }
}
