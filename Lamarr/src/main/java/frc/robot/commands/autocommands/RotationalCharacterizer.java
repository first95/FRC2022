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
        addRequirements(drivebase);

        addCommands(
            new InstantCommand(() ->
                {drivebase.resetGyro();
                drivebase.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
                drivebase.driveWithTankControls(0.2, -0.2);}
            )
        );
        addCommands(new WaitCommand(4));
        addCommands(
            new InstantCommand(() ->
                {drivebase.driveWithTankControls(0, 0);})
        );
        addCommands(
            new InstantCommand(() -> {
                double angle = drivebase.getYaw().getRadians();
                double leftMeters = drivebase.getWheelPositions()[0];
                double rightMeters = drivebase.getWheelPositions()[1];
                // Average the two.  Would be divided by two, but the next formula would double it, so
                // both are omitted
                double arcLength = (Math.abs(leftMeters) + Math.abs(rightMeters));
                double trackwidth = arcLength / angle;

                SmartDashboard.putNumber("Trackwidth", trackwidth);
                
                SmartDashboard.putNumber("Angle", Math.toDegrees(angle));
                SmartDashboard.putNumber("leftDist", leftMeters);
                SmartDashboard.putNumber("rightDist", rightMeters);
            })
        );
    }
}