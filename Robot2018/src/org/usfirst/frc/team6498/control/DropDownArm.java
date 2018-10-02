package org.usfirst.frc.team6498.control;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class DropDownArm {
	public DoubleSolenoid actuator;
	
	public DropDownArm(DoubleSolenoid actuatorC) {
		actuator=actuatorC;
	}
	
	public void drop(boolean move) {
			if(move) {
			actuator.set(DoubleSolenoid.Value.kForward);
			}else {
				actuator.set(DoubleSolenoid.Value.kReverse);
			}
		}
}
