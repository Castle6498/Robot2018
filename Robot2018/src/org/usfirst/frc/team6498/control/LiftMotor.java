package org.usfirst.frc.team6498.control;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.NidecBrushless;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;

public class LiftMotor{

	/*
	 * limit switches:
	 * 1 at top 
	 * 1 at bottom
	 *  
	 * ultrasonic
	 * 
	 */
	public NidecBrushless motor;
	public DigitalInput topLimit;
	public DigitalInput bottomLimit;
	public Encoder encoder;
	public Solenoid spindleLock;
	public Solenoid shifter;
	
	public double speed=0;
	
	public double liftConstantLow = .0625;
	public double liftConstantHigh = .0625*2;
	
	public double stickTolerance = .01;
	
	public LiftMotor(NidecBrushless motorC, DigitalInput topLimitC, DigitalInput bottomLimitC, Encoder encoderC, Solenoid spindleLockC, Solenoid shifter) {
		motor=motorC;
		topLimit=topLimitC;
		bottomLimit=bottomLimitC;
		encoder=encoderC;
		spindleLock=spindleLockC;
		this.shifter=shifter;
		lockSpindle(false);
		
		//encoder.setDistancePerPulse(.0165);//.016906
	}
	
	public void execute() {
		motor.set(speed);
		speed=0;
	}
	
	
	
	public void lockSpindle(boolean lock) {
		spindleLock.set(!lock); //inverted to close when true 
		//spindleLock.set(true) opens
		//lockSpindle(true) closes
	}
	
	
	public boolean topLimit() {
		return topLimit.get();
	}
	
	public boolean bottomLimit() {
		return bottomLimit.get();
	}
	
	public boolean setSpeed(double speed) {
		if(bottomLimit()) {
			encoder.reset();
		}
		
		if(spindleLock.get()==false&&Math.abs(speed)>.08) { //
			lockSpindle(false); //false due to inversion in 
		}
		if(speed>=stickTolerance&&!topLimit()) {
			this.speed=speed;
			return true;
			}
		else if(speed<-stickTolerance&&!bottomLimit()) {
			this.speed=speed;
			return true;
		}else if(!bottomLimit()&&!topLimit()) {
			if(shifter.get()==false) {
				this.speed=liftConstantLow;
			}else {
				this.speed=liftConstantHigh;
			}		
			return true;
		}
		else {
			this.speed=0;
			motor.set(0);
			return false;
		}
	}
	
	public double getDistance() {
		double x = encoder.getRaw();
		//Equation:  y = 0.0035x + 15
		x=.0035*x+15;
		return x;
	}

	public String drivePhase="start";
	public double slowDownSpeed=.4;
	public double slowDownDistance=2;
	public double direction=1;
	private void driveToDistanceSpeedSetter(double speed){
		if(!setSpeed(speed)) {
			drivePhase="finish";
		}
	}
	
	public boolean driveToDistance(double distance, double speed, double give) {
		boolean status=false;
		switch(drivePhase) {
		case "start":
			double currentPos=getDistance();
			
			if(distance-currentPos>=0) direction=1;
			else if(distance-currentPos<0) direction=-1;
			
			drivePhase="move";
			
			break;
		case "move":
			driveToDistanceSpeedSetter(speed*direction);
			if(Math.abs(getDistance()-distance)<=give+slowDownDistance) {
				drivePhase="slow down";
			}
			break;
		case "slow down":
			driveToDistanceSpeedSetter(slowDownSpeed*direction);
			if(Math.abs(getDistance()-distance)<=give) {
				driveToDistanceSpeedSetter(0);
				drivePhase="finish";
			}
			break;
		case "finish":
			status=true;
			drivePhase="start";
			break;
		}
		return status;
	}

	public String driveAddPhase="start";
	public double directionAdd=1;
	public double targetAddDistance=0;
	private void driveToDistanceAddSpeedSetter(double speed){
		if(!setSpeed(speed)) {
			driveAddPhase="finish";
		}
	}
	
	public boolean driveToAdditionalDistance(double distance, double speed, double give) {
		boolean status=false;
		switch(driveAddPhase) {
		case "start":
			double currentPos=getDistance();
			targetAddDistance=currentPos+distance;
			if(distance>=0) directionAdd=1;
			else if(distance<0) directionAdd=-1;
			
			driveAddPhase="move";
			
			break;
		case "move":
			driveToDistanceAddSpeedSetter(speed*directionAdd);
			if(Math.abs(getDistance()-targetAddDistance)<=give) {
				driveAddPhase="finish";
			}
			break;
		case "finish":
			status=true;
			driveAddPhase="start";
			break;
		}
		return status;
	}
	
	public boolean isBusy() {
		if(drivePhase =="start"&& driveAddPhase=="start") {
			return false;
		}else {
			return true;
		}
	}
	
}
