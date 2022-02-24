// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/** An example command that uses an example subsystem. */
public class AutoClimbStage3 extends SequentialCommandGroup {

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoClimbStage3(Climber climber) {
        addRequirements(climber);
        Arrays.fill(RobotContainer.oi.ClimberStageThreeSteps, 0);
        
        if (RobotContainer.oi.ClimberStageThreeSteps[0] == 0) { 
            addCommands(new InstantCommand(() -> {
                climber.travelDistance(-95);
            }));
            addCommands(new WaitUntilCommand(climber.hasLeftReachedReference(-95)));
            addCommands(new WaitCommand(1));
            RobotContainer.oi.ClimberStageThreeSteps[0] = 1;
        }
        if (RobotContainer.oi.ClimberStageThreeSteps[1] == 0) { 
            addCommands(new InstantCommand(() -> {
                climber.togglePistons();
            }));    
            addCommands(new WaitCommand(1));
            RobotContainer.oi.ClimberStageThreeSteps[1] = 1;
        }
        if (RobotContainer.oi.ClimberStageThreeSteps[2] == 0) { 
            addCommands(new InstantCommand(() -> {
                climber.travelDistance(-30);
            }));
            addCommands(new WaitUntilCommand(climber.hasLeftReachedReference(-30)));
            RobotContainer.oi.ClimberStageThreeSteps[2] = 1;
        }
        if (RobotContainer.oi.ClimberStageThreeSteps[3] == 0) { 
            addCommands(new InstantCommand(() -> {
                climber.togglePistons();
            }));
            RobotContainer.oi.ClimberStageThreeSteps[3] = 1;
        }
        if (RobotContainer.oi.ClimberStageThreeSteps[4] == 0) { 
            addCommands(new InstantCommand(() -> {
                climber.travelDistance(10);
            }));
            addCommands(new WaitUntilCommand(climber.hasLeftReachedReference(10)));
            RobotContainer.oi.ClimberStageThreeSteps[4] = 1;
        }
    }
}
