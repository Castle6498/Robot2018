package org.usfirst.frc.team6498.control;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PidGyroRate implements PIDSource {
	public AHRS nav;
	
	
	public PidGyroRate(AHRS nav) {
		this.nav=nav;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kRate;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		double degreesPerSecond=radiansToDegrees(nav.getRate());
		return degreesPerSecond;
	}
	
	public double radiansToDegrees(double radians) {
		return (radians/Math.PI)*180;
	}
	
	public double rate() {
		return radiansToDegrees(nav.getRate());
	}
	
}
