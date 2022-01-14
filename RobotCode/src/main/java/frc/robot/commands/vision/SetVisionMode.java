/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.VisionProcessor;

/**
 * Command to set the main camera into one of its modes
 */
public class SetVisionMode extends CommandBase {
    private VisionProcessor.VisionMode targetMode;
    private VisionProcessor m_VisionProcessor;

    /**
     * Command the camera to enter a mode
     * 
     * @param isHumanVisible true if the camera should be configured for human use,
     *                       or false to configure the camera for machine vision.
     */
    public SetVisionMode(VisionProcessor visionProcessor, VisionProcessor.VisionMode mode) {
        targetMode = mode;
        m_VisionProcessor = visionProcessor;
        addRequirements(visionProcessor);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_VisionProcessor.SetMode(targetMode);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
}
