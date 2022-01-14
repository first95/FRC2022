/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 
import frc.robot.commands.ManuallyControlShooter;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Shooter extends SubsystemBase {
    private CANSparkMax leader, follower;

    public Shooter() {
        leader = new CANSparkMax(Constants.LEADER_SHOOT, MotorType.kBrushless);
        follower = new CANSparkMax(Constants.FOLLOWER_SHOOT, MotorType.kBrushless);

        init();
    }

	private void init() {
		leader.restoreFactoryDefaults();
        follower.restoreFactoryDefaults();
        follower.follow(leader, true); //true inverts the follower from the leader
	}

    /**
     * Run the shooter
     * @param speed 0 for stationary, 1 for full forward, -1 for full reverse
     */
    public void runShooterOpen(double speed) {
        //System.out.println("Setting shooter speed to " + speed);
        leader.set(speed);
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
