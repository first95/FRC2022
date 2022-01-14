/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.LinkedList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.cscore.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.vision.SetVisionMode;

/**
 * A subsystem to read from the camera(s), process them, and output video to the
 * smart dashboard
 */
public class VisionProcessor extends SubsystemBase {
    public enum VisionMode {
        /**
         * Camera is set to show the view of the upper port to a human, with both lights
         * off
         */
        UPPER_PORT_HUMAN,
        /** Camera is configured for machine vision, green light on */
        UPPER_PORT_MACHINE,
        /**
         * Camera is set to show the view of the switch to a human, with both lights off
         */
        SWITCH_HUMAN,
        /** Camera is configured for machine vision, white light on */
        CONTROL_PANEL_MACHINE,
    }

    private enum LightMode {
        GREEN, WHITE, OFF
    }

    /** The camera aimed at the upper port */
    UsbCamera upperPortCam;
    /** The view exposed to the drivers for them to see through */
    MjpegServer fpsViewServer;
    /** The camera aimed upward at the climber/control panel */
    UsbCamera lookupCam;

    private TalonSRX ringLightController;

    // Note that these paths are based on the physical port into which each camera
    // is plugged.
    // Using these paths makes it not matter which camera is found first, which is
    // what we get with /dev/videoN
    public final String UPPER_PORT_CAM_PATH = "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0";
    public final String LOOKUP_CAM_PATH = "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0";

    private double[] bearingsList = null;
    private double[] rangesList = null;

    public VisionProcessor() {
        super();
        // upperPortCam = new UsbCamera("Upper port cam", UPPER_PORT_CAM_PATH);
        // upperPortCam.setResolution(320, 240);
        // upperPortCam.setFPS(20);
        // fpsViewServer = new MjpegServer("First person view", 1181);
        // fpsViewServer.setSource(upperPortCam);
        lookupCam = new UsbCamera("Upward-facing cam", LOOKUP_CAM_PATH);
        lookupCam.setResolution(800, 600);
        lookupCam.setFPS(24);

        ringLightController = new TalonSRX(Constants.TARGET_CAM_GREEN_RINGLIGHT_TALON_ID);
    }

    public class VisionTargetInfo {
        VisionTargetInfo(double bearingDegrees, double rangeInches) {
            this.bearingDegrees = bearingDegrees;
            this.rangeInches = rangeInches;
        }

        public double bearingDegrees;
        public double rangeInches;
    }

    /**
     * Returns the last-seen list of vision targets.
     * 
     * @return the last-seen list of vision targets.
     */
    public LinkedList<VisionTargetInfo> getCurVisibleVisionTargets() {
        LinkedList<VisionTargetInfo> vvts = new LinkedList<VisionTargetInfo>();
        for (int i = 0; i < bearingsList.length; ++i) {
            vvts.add(new VisionTargetInfo(bearingsList[i], rangesList[i]));
        }
        return vvts;
    }

    /**
     * Set the vision system to a specific mode
     * 
     * @param mode
     */
    public void SetMode(VisionMode mode) {
        System.out.println("Setting vision mode to: " + mode);
        switch (mode) {
            default:
                /*
                 * case UPPER_PORT_HUMAN:
                 * SetLightMode(LightMode.OFF);
                 * fpsViewServer.setSource(upperPortCam);
                 * // TODO: adjust exposure
                 * break;
                 * case UPPER_PORT_MACHINE:
                 * SetLightMode(LightMode.GREEN);
                 * fpsViewServer.setSource(upperPortCam);
                 * // TODO: adjust exposure
                 * break;
                 */
            case SWITCH_HUMAN:
                SetLightMode(LightMode.OFF);
                fpsViewServer.setSource(lookupCam);
                break;
            case CONTROL_PANEL_MACHINE:
                SetLightMode(LightMode.WHITE);
                fpsViewServer.setSource(lookupCam);
                break;
        }
    }

    /**
     * Set the mode for the lights
     * 
     * @param mode
     */
    private void SetLightMode(LightMode mode) {
        System.out.println("Setting ringlight mode to: " + mode);
        switch (mode) {
            case GREEN:
                ringLightController.set(ControlMode.PercentOutput, 1);
                break;
            case WHITE:
                ringLightController.set(ControlMode.PercentOutput, -1);
                break;
            case OFF: // fallthrough
            default:
                ringLightController.set(ControlMode.PercentOutput, 0);
                break;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
