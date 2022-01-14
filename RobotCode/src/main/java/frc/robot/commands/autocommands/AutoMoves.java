/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.OI;

/**
 * An example command. You can replace me with your own command.
 */
public class AutoMoves extends CommandGroupBase {
  public AutoMoves(Command[] commands) {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.m_subsystem);
    // addSequential(new AutoSpinUpShooter(1000));
    Robot.AutoDriveSpeed = SmartDashboard.getNumber("Automode Drive speed (neg for backwards)", 0.3);
    Robot.PSAutoDriveSpeed = SmartDashboard.getNumber("Pre-Shoot Automode Drive speed", 0.3);

    //addSequential(new AutoDrive(3000, -Robot.PSAutoDriveSpeed));
    //addSequential(new AutoShoot(8000));
    //addSequential(new AutoDrive(3000, -Robot.AutoDriveSpeed));
    OI.auto_collector_deploy = true;
    // Command[] commands = {new AutoAim(81), new AutoCollect(Robot.AutoDriveSpeed), new AutoAim(141)};
    // registerGroupedCommands(commands);
  }

  @Override
  public void addCommands(Command... commands) {
    // TODO Auto-generated method stub
    
  }

}
