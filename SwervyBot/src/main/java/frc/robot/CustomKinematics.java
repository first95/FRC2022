package frc.robot;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.BetterSwerveModuleState;

public class CustomKinematics {

    private final int numModules;
    private final Translation2d[] modules;
    private Translation2d prevCoR = new Translation2d();

    private double[] moduleDistances;
    private double[] rotationVectorAngles;

    public CustomKinematics(Translation2d... wheelsMeters) {
        if (wheelsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
          }
          numModules = wheelsMeters.length;
          modules = Arrays.copyOf(wheelsMeters, numModules);
          moduleDistances = new double[numModules];
          rotationVectorAngles = new double[numModules];
        
          for (int i = 0; i < numModules; i++) {
              moduleDistances[i] = Math.hypot(modules[i].getX(), modules[i].getY());
              rotationVectorAngles[i] = Math.PI - Math.atan2(modules[i].getX(), modules[i].getY());
          }
    }

    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
        if (!centerOfRotationMeters.equals(prevCoR)) {
            for (int i = 0; i < numModules; i++) {
                moduleDistances[i] = Math.hypot(
                    modules[i].getX() - centerOfRotationMeters.getX(),
                    modules[i].getY() - centerOfRotationMeters.getY());
                rotationVectorAngles[i] = Math.PI - Math.atan2(
                    modules[i].getX() - centerOfRotationMeters.getX(),
                    modules[i].getY() - centerOfRotationMeters.getY());
            }
            prevCoR = centerOfRotationMeters;
        }
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[numModules];
        for (int i = 0; i < numModules; i++) {
            double x = chassisSpeeds.omegaRadiansPerSecond * moduleDistances[i] * Math.cos(rotationVectorAngles[i])
                + chassisSpeeds.vxMetersPerSecond;
            double y = chassisSpeeds.omegaRadiansPerSecond * moduleDistances[i] * Math.sin(rotationVectorAngles[i])
            + chassisSpeeds.vyMetersPerSecond;
            double moduleSpeed = Math.hypot(x, y);
            Rotation2d moduleAngle = new Rotation2d(x, y);
            swerveModuleStates[i] = new SwerveModuleState(moduleSpeed, moduleAngle);
        }
        return swerveModuleStates;
    }

    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        return toSwerveModuleStates(chassisSpeeds, new Translation2d());
    }
}
