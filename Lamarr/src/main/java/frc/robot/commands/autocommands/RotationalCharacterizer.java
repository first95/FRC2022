package frc.robot.commands.autocommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveBase;

public class RotationalCharacterizer extends SequentialCommandGroup {
    public RotationalCharacterizer(DriveBase drivebase) {
        addCommands(
            new InstantCommand(() ->
                {drivebase.resetGyro();
                drivebase.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
                drivebase.driveWithTankControls(0.2, -0.2);})
            .andThen(new WaitCommand(4))
            .andThen(new InstantCommand(() ->
                {SmartDashboard.putNumber("Left Distance", drivebase.getWheelPositions()[0]);
                SmartDashboard.putNumber("Right Distance", drivebase.getWheelPositions()[1]);
                SmartDashboard.putNumber("Angle", drivebase.getYaw().getDegrees());}))
        );
    }
}