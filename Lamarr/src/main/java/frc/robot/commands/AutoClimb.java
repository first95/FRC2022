// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class AutoClimb extends SequentialCommandGroup {

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoClimb(Climber climber) {
        addRequirements(climber);
        addCommands(new InstantCommand(() -> 
          {
              climber.travelDistance(-10);
          }));

        addCommands(new WaitCommand(1));

        addCommands(new InstantCommand(() -> 
        {
            climber.togglePistons();
        }));

        addCommands(new WaitCommand(1));

        addCommands(new InstantCommand(() -> 
        {
            climber.travelDistance(10);
        }));

        addCommands(new WaitCommand(1));

        addCommands(new InstantCommand(() -> 
        {
            climber.travelDistance(-2);
        }));

        addCommands(new WaitCommand(1));

        addCommands(new InstantCommand(() -> 
        {
            climber.togglePistons();
        }));

        addCommands(new InstantCommand(() -> 
        {
            climber.travelDistance(-8);
        }));
    }
}
