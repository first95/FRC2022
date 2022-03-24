// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoClimbStage3 extends SequentialCommandGroup {

    public AutoClimbStage3(Climber climber) {
        addRequirements(climber);
        Arrays.fill(RobotContainer.oi.ClimberStageThreeSteps, 0);
        
        if (RobotContainer.oi.ClimberStageThreeSteps[0] == 0) { 
            addCommands(new InstantCommand(() -> {
                climber.travelDistance(-158);
            }));
            addCommands(new WaitUntilCommand(climber.hasLeftReachedReference(-158)));
            addCommands(new WaitCommand(0.25));
            RobotContainer.oi.ClimberStageThreeSteps[0] = 1;
        }
        if (RobotContainer.oi.ClimberStageThreeSteps[1] == 0) { 
            addCommands(new InstantCommand(() -> {
                climber.togglePistons();
            }));    
            addCommands(new WaitCommand(0.25));
            RobotContainer.oi.ClimberStageThreeSteps[1] = 1;
        }
        if (RobotContainer.oi.ClimberStageThreeSteps[2] == 0) { 
            addCommands(new InstantCommand(() -> {
                climber.travelDistance(-50);
            }));
            addCommands(new WaitUntilCommand(climber.hasLeftReachedReference(-50)));
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
                climber.travelDistance(0);
            }));
            addCommands(new WaitUntilCommand(climber.hasLeftReachedReference(0)));
            addCommands(new WaitCommand(0.5));
            addCommands(new InstantCommand(() -> {
                climber.travelDistance(-50);
            }));
            addCommands(new WaitUntilCommand(climber.hasLeftReachedReference(-50)));
            RobotContainer.oi.ClimberStageThreeSteps[4] = 1;
        }
    }
}
