package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.CargoHandler;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterHood;

public class Shoot extends CommandBase {
    private LimeLight limelightport;
    private ShooterHood shooterhood;
    private DriveBase drivebase;

    private boolean highHub, far;
    private double range;

    public Shoot(boolean highHub, DriveBase drivebase, LimeLight limelightport, ShooterHood shooterhood) {
        this.limelightport = limelightport;
        this.shooterhood = shooterhood;
        this.highHub = highHub;
        this.drivebase = drivebase;
        addRequirements(limelightport);
        addRequirements(shooterhood);
    }

    @Override
    public void initialize() {
        range = limelightport.getFloorDistanceToTarg();

        // HOOD DOWN at 137
        // HOOD UP at 138
        if (Math.abs(Vision.DESIRED_RANGE_INCH - range) < Math.abs(Vision.FAR_RANGE_INCH - range)) {
            shooterhood.setHood(true);
            far = false;
        } else {
            shooterhood.setHood(false);
            far = true;
        }
    
    }

    @Override
    public void execute() {
        if (far) {
            RobotContainer.oi.auto_shooting_speed = CargoHandler.farDistanceToShooterRPM(range);
            RobotContainer.oi.auto_roller_speed = CargoHandler.farDistanceToShooterRPM(range) *
              SmartDashboard.getNumber("Shooter Ratio", Constants.CargoHandling.SHOOTER_RATIO);
          } else {
            RobotContainer.oi.auto_shooting_speed = highHub ? CargoHandler.distanceToShooterRPM(range)
                : Constants.CargoHandling.SHOOTING_LOW_SPEED;
            RobotContainer.oi.auto_roller_speed = highHub
                ? CargoHandler.distanceToShooterRPM(range) * SmartDashboard.getNumber("Shooter Ratio", Constants.CargoHandling.SHOOTER_RATIO)
                : Constants.CargoHandling.ROLLER_LOW_SPEED;
          }

        drivebase.setAirBrakes(true);
        RobotContainer.oi.auto_shooting = true;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setAirBrakes(false);
        RobotContainer.oi.auto_shooting = false;
        RobotContainer.oi.auto_shooting_speed = 0;
        RobotContainer.oi.auto_roller_speed = 0;
  }
}
