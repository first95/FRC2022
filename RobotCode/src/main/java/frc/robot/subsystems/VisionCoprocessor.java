/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.LinkedList;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem to read from the vision coprocessor
 */
public class VisionCoprocessor extends SubsystemBase {
    private NetworkTableEntry bearingsListEntry = null;
    private NetworkTableEntry rangesListEntry = null;
    private double[] bearingsList = null;
    private double[] rangesList = null;

    private NetworkTableEntry isCameraHumanVisible;

    public void initDefaultCommand(CommandBase defaultCommand) {
        // Set the default command for a subsystem here.
        setDefaultCommand(defaultCommand);
        NetworkTableInstance allTables = NetworkTableInstance.getDefault();
        NetworkTable visionTable = allTables.getTable("vision_metrics");
        bearingsListEntry = visionTable.getEntry("target bearings (deg)");
        rangesListEntry = visionTable.getEntry("target bearings (deg)");
        NetworkTableEntry numVisionTargetsVisibleEntry = visionTable.getEntry("target bearings (deg)");
        numVisionTargetsVisibleEntry.addListener(event -> {this.onNumVtUpdated(event);} , EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        isCameraHumanVisible = allTables.getTable("camera_control").getEntry("camera_for_humans");
    }

    private void onNumVtUpdated(EntryNotification event) {
        int numVisionTargets = (int)(event.value.getDouble());
        double[] bearings = bearingsListEntry.getDoubleArray(new double[0]);
        double[] ranges = rangesListEntry.getDoubleArray(new double[0]);

        // Simple synchronization confiration
        if(numVisionTargets == bearings.length && numVisionTargets == ranges.length) {
            bearingsList = bearings;
            rangesList = ranges;
        }
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
     * @return the last-seen list of vision targets.
     */
    public LinkedList<VisionTargetInfo> getCurVisibleVisionTargets() {
        LinkedList<VisionTargetInfo> vvts = new LinkedList<VisionTargetInfo>();
        for(int i = 0; i < bearingsList.length; ++i) {
            vvts.add(new VisionTargetInfo(bearingsList[i], rangesList[i]));
        }
        return vvts;
    }

    /**
     * Get current camera configuration
     * @return true if the camera is configured for human use, 
     * or false if configured for machine vision.
     */
    public boolean isCameraHumanVision() {
        return isCameraHumanVisible.getBoolean(false);
    }

    /**
     * Command the camera to enter a mode
     * @param isHumanVisible true if the camera should be configured for human use, 
     * or false to configure the camera for machine vision.
     */
    public void setCameraIsHumanVisible(boolean isHumanVisible) {
        isCameraHumanVisible.setBoolean(isHumanVisible);
    }
}
