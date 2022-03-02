package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;

public class PassiveMode extends SequentialCommandGroup {
    public PassiveMode(Climber climber) {
        addRequirements(climber);
        addCommands(new InstantCommand(() -> {
            climber.travelDistance(0);
        }));
        addCommands(new WaitUntilCommand(climber.hasLeftReachedReference(0)));
    }
}
