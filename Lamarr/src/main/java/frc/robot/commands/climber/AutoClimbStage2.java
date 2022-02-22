// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberStep;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;

import java.lang.reflect.Array;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/** An example command that uses an example subsystem. */
public class AutoClimbStage2 extends SequentialCommandGroup {

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoClimbStage2(Climber climber) {
        addRequirements(climber);
        Arrays.fill(RobotContainer.oi.ClimberStageOneSteps, 0);

        if (RobotContainer.oi.ClimberStageTwoSteps[0] == 0) { 
            addCommands(new WaitUntilCommand(climber.travelDistance(-30)));
            RobotContainer.oi.ClimberStageTwoSteps[0] = 1;
        }
        if (RobotContainer.oi.ClimberStageTwoSteps[1] == 0) { 
            addCommands(new InstantCommand(() -> {
                climber.togglePistons();
            }));
            RobotContainer.oi.ClimberStageTwoSteps[1] = 1;
        }
        if (RobotContainer.oi.ClimberStageTwoSteps[2] == 0) { 
            addCommands(new WaitUntilCommand(climber.travelDistance(0)));
            RobotContainer.oi.ClimberStageTwoSteps[2] = 1;
        }
    }
}
