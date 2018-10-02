package org.usfirst.frc.team6498.control;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PidEncoderAverageRate implements PIDSource {
	public Encoder leftEncoder;
	public Encoder rightEncoder;
	public PidEncoderAverageRate(Encoder leftEncoder, Encoder rightEncoder) {
		this.leftEncoder=leftEncoder;
		this.rightEncoder=rightEncoder;
	}
	public double getAverageSpeed() {
		double left=leftEncoder.getRate();
		double right=rightEncoder.getRate();
		
		return (left+right)/2;
	}
	
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kRate;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return getAverageSpeed();
	}

}
