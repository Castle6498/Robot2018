package org.usfirst.frc.team6498.control;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
public class Claw {
 
 public Solenoid solenoid;
 public Spark motor;
 public DigitalInput topSwitch;
 public DigitalInput bottomSwitch;
 
 public double speed=0;
 
 public Claw(Solenoid solenoidC, Spark motorC, DigitalInput topSwitchC, DigitalInput bottomSwitchC) {
  
  solenoid = solenoidC;
  motor=motorC;
  solenoid.set(false);
  topSwitch = topSwitchC;
  bottomSwitch = bottomSwitchC;
 }
 
 public void execute() {
	
	 motor.set(speed);
	 speed=0;
	 
 }
 
 public boolean topLimit() {
		return topSwitch.get();
	}
	
	public boolean bottomLimit() {
		return bottomSwitch.get();
	}
	
	public boolean setSpeed(double speed) {
		if(speed>=0&&!topLimit()) {
			this.speed=speed;
			return true;
			}
		else if(speed<0&&!bottomLimit()) {
			this.speed=speed;
			return true;
		}
		else {
			motor.set(0);
			this.speed=0;
			return false;
		}
	}
 
 public void setOpen(boolean shift) {
  
  solenoid.set(shift);
  
 }
 
 	
 	public boolean moveDownBusy=false;
 	public boolean trigger=true;
 	//true busy false not
 	
 	public boolean moveDown(double speed) {
 		boolean status=false;
 		moveDownBusy=true;
 		trigger=setSpeed(speed);
 		if(speed<0&&bottomLimit()) {
 			moveDownBusy=false;
 			status=true;
 		}else if(speed>0&&topLimit()){
 			moveDownBusy=false;
 			status=true;
 		}
 		
 		return status;
 	}
 	
 	public boolean isBusy() {
 		if(!moveDownBusy) {
 		return false;	
 		}else {
 			return true; 			
 		}
 	}
}