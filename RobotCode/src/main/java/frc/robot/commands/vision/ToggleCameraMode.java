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
public class ToggleCameraMode extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final VisionCoprocessor m_VisionCoprocessor;

    /**
     * Command the camera to enter whatever mode it is not in
     */
    public ToggleCameraMode(VisionCoprocessor visionCoprocessor) {
        m_VisionCoprocessor = visionCoprocessor;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(visionCoprocessor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        m_VisionCoprocessor.setCameraIsHumanVisible(!m_VisionCoprocessor.isCameraHumanVision());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
