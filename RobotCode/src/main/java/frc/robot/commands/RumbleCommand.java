/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OI;
import frc.robot.RobotContainer;

/**
 * Rumble for a given amount of time
 */
public class RumbleCommand extends WaitCommand {
    private OI.Controller controller;
    private Joystick.RumbleType side;
    private double severity;
    private double duration;

    /**
     * Rumble one of the controllers
     * 
     * @param controller which controller to rumble
     * @param side       side to rumble
     * @param severity   severity at which to rumble - 0.0 to 1.0
     * @param duration   how long to rumble, in seconds
     */
    public RumbleCommand(OI.Controller controller, Joystick.RumbleType side, double severity, double duration) {
        super(duration);
        this.controller = controller;
        this.side = side;
        this.severity = severity;
        this.duration = duration;
    }

    /**
     * This is the actual action, must be executed exactly once
     */
    private void doRumble() {
        RobotContainer.oi.Rumble(controller, side, severity, duration);
    }

    @Override
    public void execute() {
        // System.out.println("RumbleCommand.execute()");
    }

    @Override
    public boolean isFinished() {
        boolean done = super.isFinished();
        // System.out.println("RumbleCommand.isFinished(): " + done);
        return done;
    }

    @Override
    public void initialize() {
        // System.out.println("RumbleCommand.initialize()");
        doRumble();
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("RumbleCommand.end()");
        super.end(interrupted);
    }
}
