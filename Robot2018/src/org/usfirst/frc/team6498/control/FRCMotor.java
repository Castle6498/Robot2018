
package org.usfirst.frc.team6498.control;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;

public class FRCMotor {
/*
 * setSpeed
 * distancePerTicks
 * getDistance
 * resetEncoder
 * driveToDistance 
 */
	public Spark motor;
	public Encoder encoder;
	
	public FRCMotor(Spark motorC, Encoder encoderC, double distancePerTicks) {
		motor=motorC;
		encoder=encoderC;
		encoder.setDistancePerPulse(distancePerTicks);
	}
	
	public void setSpeed(double speed) {
		motor.set(speed);
	}

	public double getDistance() {
		return encoder.getDistance();
	}

	public void resetEncoder() {
		encoder.reset();
	}
	
	String drivePhase="start";
	double driveTarget=0;
	public double slowDownSpeed=.3;
	public double slowDownDistance=10;
	
	public boolean driveToDistance(double distance, double speed, double give) {
		boolean running=true;
		switch(drivePhase) {
		case "start":
			double currentPos=getDistance();
			driveTarget=currentPos+distance;
			
			if(distance>=0) motor.set(speed);
			else if(distance<0) motor.set(-speed);
			
			drivePhase="move";
			
			break;
		case "move":
			if(Math.abs(getDistance()-driveTarget)<=give+slowDownDistance) {
				motor.set(slowDownSpeed);
				drivePhase="slow down";
			}
			break;
		case "slow down":
			if(Math.abs(getDistance()-driveTarget)<=give) {
				motor.set(0);
				drivePhase="finish";
			}
			break;
		case "finish":
			running=false;
			drivePhase="start";
			break;
		}
		return running;
	}
}
