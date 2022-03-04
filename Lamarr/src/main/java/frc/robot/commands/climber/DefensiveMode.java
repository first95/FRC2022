package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;

public class DefensiveMode extends SequentialCommandGroup {
    public DefensiveMode(Climber climber) {
        addRequirements(climber);
        addCommands(new InstantCommand(() -> {
            climber.travelDistance(-100);
        }));
        addCommands(new WaitUntilCommand(climber.hasLeftReachedReference(-100)));
    }
}
