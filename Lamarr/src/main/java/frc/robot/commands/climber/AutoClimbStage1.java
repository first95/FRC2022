// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoClimbStage1 extends SequentialCommandGroup {

    public AutoClimbStage1(Climber climber) {
        addRequirements(climber);
        Arrays.fill(RobotContainer.oi.ClimberStageFourSteps, 0);

        if (RobotContainer.oi.ClimberStageOneSteps[0] == 0) {
            addCommands(new InstantCommand(() -> {
                climber.travelDistance(-95);
            }));
            addCommands(new WaitUntilCommand(climber.hasLeftReachedReference(-95)));
            RobotContainer.oi.ClimberStageOneSteps[0] = 1;
        }

        if (RobotContainer.oi.ClimberStageOneSteps[1] == 0) {
            addCommands(new InstantCommand(() -> {
                climber.togglePistons();
            }));
            RobotContainer.oi.ClimberStageOneSteps[1] = 1;
        }
    }
}
