/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionCoprocessor;

/**
 * Command to set the main camera into one of its modes
 */
public class SetCameraMode extends CommandBase {
    private boolean camShouldBeHumanVisible = false;
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final VisionCoprocessor m_VisionCoprocessor;
    
    /**
     * Command the camera to enter a mode
     * @param isHumanVisible true if the camera should be configured for human use, 
     * or false to configure the camera for machine vision.
     */
    public SetCameraMode(VisionCoprocessor visionCoprocessor, boolean isHumanVisible) {
        camShouldBeHumanVisible = isHumanVisible;
        m_VisionCoprocessor = visionCoprocessor;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(visionCoprocessor);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_VisionCoprocessor.setCameraIsHumanVisible(camShouldBeHumanVisible);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }
}
