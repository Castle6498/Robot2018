package org.usfirst.frc.team6498.control;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PidGyroDisplacement implements PIDSource {
	public double lockPoint = 0;
	
	public AHRS nav;
	public PidGyroDisplacement(AHRS nav) {
		this.nav=nav;
	}
	
	
	public void lock() {
		lockPoint=nav.getAngle();
	}
	public void lock(double angle) {
		lockPoint=angle;
	}
	
	
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub

	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		return nav.getAngle()-lockPoint;
	}

}
