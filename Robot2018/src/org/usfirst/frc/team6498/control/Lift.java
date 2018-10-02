package org.usfirst.frc.team6498.control;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;

public class Lift {
	/*
	 * LiftMotor
	 * Solenoid
	 * Ultrasonic
	 */
	public LiftMotor motor;
	public Ultrasonic ultrasonic;
	public Solenoid shifter;
	

	
	public Lift(LiftMotor motorC, Ultrasonic ultrasonicC, Solenoid shifterC) {
		motor=motorC;
		ultrasonic=ultrasonicC;
		shifter=shifterC;
	}
	
	public void startUltrasonic(boolean start) {
		ultrasonic.setAutomaticMode(start);
	}
	
	public boolean setSpeed(double speed) {
		return motor.setSpeed(speed);
	}
	
	public double height() {
		return motor.getDistance();
	}
	
	public double frontDistance() {
		 return ultrasonic.getRangeInches();
	}

	public void execute() {
		motor.execute();
	}

	public boolean liftToSetHeight(double height, double speed, double give) {
		return motor.driveToDistance(height, speed, give);
	}
	
	public boolean liftToAdditionalHeight(double height, double speed, double give) {
		return motor.driveToAdditionalDistance(height, speed, give);
	}
	
	public void lockPin(boolean lock) {
		motor.lockSpindle(lock);
	}
	
	
	public void shiftGear(boolean shift) {
		shifter.set(shift);
	}
	
	public String liftToScalePhase="start";
	private void liftToScaleSpeedSetter(double speed) {
		if(!motor.setSpeed(speed)){
			liftToScalePhase="finish";
		}
	}	
	
	public boolean liftToScale(double speed, double tolerance, double adjustHeight) {
		boolean status=false;
		switch(liftToScalePhase) {
		case "start":
			liftToScalePhase="look";
			break;
		case "look":
			liftToScaleSpeedSetter(speed);
			if(frontDistance()<=tolerance) {
				liftToScaleSpeedSetter(0);
				liftToScalePhase="adjust";
			}
			break;
		case "adjust":
			if(liftToAdditionalHeight(adjustHeight, speed, 1)) {	
				liftToScalePhase="finish";
			}
			break;
		case "finish":
			status=true;
			liftToScalePhase="start";
			break;
		}
		return status;
	}

	public double switchHeight=25;
	public boolean liftToSwitch(double speed) {
		return motor.driveToDistance(switchHeight, speed, 5);
	}

	public boolean maxOut(double speed) {
		if(motor.topLimit()) {
			motor.setSpeed(0);
			return true;
		}else {
			motor.setSpeed(Math.abs(speed));
			return false;
		}
	}
	
	public boolean minOut(double speed) {
		if(motor.bottomLimit()) {
			motor.setSpeed(0);
			return true;
		}else {
			motor.setSpeed(-Math.abs(speed));
			return false;
		}
	}
	
	public boolean isBusy() {
		if( liftToScalePhase=="start"&&!motor.isBusy()) {
			return false;
		}else {
			return true;
		}
	}
	
	
	public String autoShiftStage = "raise";
	
	public boolean autoShift(){
		
		boolean autoShiftStatus = false;
		
		switch(autoShiftStage) {
		
		case "raise":
			if(liftToSetHeight(25, 1, 2)) {
				shiftGear(true);
				autoShiftStage = "lower";				
			}
			break;		
		case "lower":
			if(minOut(-1)) {
				autoShiftStage = "finish";
			}
		case "finish":
			autoShiftStage = "raise";
			autoShiftStatus=true;
			break;
		}
		return autoShiftStatus;
	}
	

}
