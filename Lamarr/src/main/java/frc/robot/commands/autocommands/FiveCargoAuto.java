// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.Auton;
import frc.robot.Constants.CargoHandling;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.subsystems.CargoHandler;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterHood;

/** An example command that uses an example subsystem. */
public class FourCargoAuto extends SequentialCommandGroup {
  public FourCargoAuto(DriveBase drivebase, LimeLight limelightport, ShooterHood shooterhood, Trajectory [] trajectories) {
    addRequirements(drivebase);
    addRequirements(limelightport);

    // Deploy and run the collector
    addCommands(new InstantCommand(() -> 
      {RobotContainer.oi.auto_collector_toggle = true;
      RobotContainer.oi.auto_collect_speed = 0.8;}));

    // Drive to the first cargo
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.5Ball1_Backup]));

    // Retract the collector
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_collector_toggle = true;}));

    // Pew Pew
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_shooting_speed = CargoHandler.distanceToShooterRPM(limelightport.getFloorDistanceToTarg());
      RobotContainer.oi.auto_roller_speed = (CargoHandler.distanceToShooterRPM(limelightport.getFloorDistanceToTarg()) * 
        SmartDashboard.getNumber("Shooter Ratio", CargoHandling.SHOOTER_RATIO));
      RobotContainer.oi.auto_shooting = true;}));

    addCommands(new WaitCommand(2));

    //don't we want to return to shooter idle speed, no 0?
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_shooting = false;
      RobotContainer.oi.auto_shooting_speed = 0;
      RobotContainer.oi.auto_roller_speed = 0;}));

    // Lineup
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.5Ball1_K1]));
    // Deploy the collector
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_collector_toggle = true;}));
    // Get the last cargo
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.5Ball3_Get3rd]));
    // Retract the collector
    addCommands(new InstantCommand(() ->
        {RobotContainer.oi.auto_collector_toggle = true;}));
    
    //shoot 3rd cargo
    addCommands(new InstantCommand(() ->
        {RobotContainer.oi.auto_shooting_speed = CargoHandler.distanceToShooterRPM(limelightport.getFloorDistanceToTarg());
        RobotContainer.oi.auto_roller_speed = (CargoHandler.distanceToShooterRPM(limelightport.getFloorDistanceToTarg()) * 
          SmartDashboard.getNumber("Shooter Ratio", CargoHandling.SHOOTER_RATIO));
        RobotContainer.oi.auto_shooting = true;}));
  
    addCommands(new WaitCommand(1));
    
    //deploy collector
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_collector_toggle = true;}));

    // Drive to get cargos 4 and 5
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.5Ball_Get45]));
    addCommands(new WaitCommand(1));

    //drive to shoot cargos 4 and 5
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.5Ball_Shoot45]));

    // Pew Pew
    addCommands(new AutoAim(true, drivebase, limelightport, shooterhood).withTimeout(3));
  }
}
