package org.usfirst.frc.team6498.control;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class CurveHelper {
	
	 public PIDControlHelper controllerTurn;
     public double resultTurn;     
     public double kPTurn = 0.0015;
     public double kITurn = 0.000;
     public double kDTurn = 0.00;
     public double kFTurn = 0.00;      
     public double kToleranceDegreesTurn = 0f;
     
     
     
     public PIDControlHelper controllerSpeed;
    public double resultSpeed;    
    public double kPSpeed = 0.002;//.0008
    public double kISpeed = 0.00;
    public double kDSpeed = 0.00;
    public double kFSpeed = 0.00;
    public double kToleranceDegreesSpeed = 2.0f;
     
    
	
      public double kDegrees=0;
      public double kRadius=0;
      public double kWheelToCenterDistance=14.875;
      public double kDegreesPerSecond=0;
      public double kStartAngle=0;
      public double kStartDistance=0;
      public double kInchesPerSecondMiddle=0;
      public double kInchesPerSecondOuter=0;
      public double kOuterDistance=0;
      public double kMiddleDistance=0;
      public double kTime=0;
      public double kDirection=1;
      public double kArcDistanceTolerance=2;
    
      
     class OuterEncoder implements PIDSource {
  		public Encoder leftEncoder;
  		public Encoder rightEncoder;
  		public Encoder correctEncoder;
  		
  		public OuterEncoder(Encoder leftEncoder, Encoder rightEncoder) {
  			this.leftEncoder=leftEncoder;
  			this.rightEncoder=rightEncoder;
  			correctEncoder=leftEncoder;
  		}
  		
  		public void setOuter(double degrees) {
  			if(degrees>0) {
    				correctEncoder=leftEncoder;
    				
    			}else {
    				correctEncoder=rightEncoder;  				
    			}
  		}
  		
  		public double getDistance() {
  			return correctEncoder.getDistance();
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
  			return correctEncoder.getRate();
  		}
  		
  		
  		
  	}
  
    public OuterEncoder outerEncoder;
    public PidGyroRate navPID;
    
	public CurveHelper(Encoder leftEncoder,Encoder rightEncoder, AHRS nav) {
		
		outerEncoder=new OuterEncoder(leftEncoder, rightEncoder);
		navPID=new PidGyroRate(nav);
		
		controllerSpeed=new PIDControlHelper(kPSpeed, kISpeed, kDSpeed, kFSpeed, kToleranceDegreesSpeed, kInchesPerSecondOuter, outerEncoder, -130, 130);
		
		
		controllerTurn=new PIDControlHelper(kPTurn, kITurn, kDTurn, kFTurn, kToleranceDegreesTurn, kDegreesPerSecond, navPID, -180, 180);
		
	}
	 
	
	
	
	void calculateParameters(double degrees, double radius, double speed) {
		kDegrees=degrees;
		kRadius=radius;
		kInchesPerSecondMiddle=speed;
		outerEncoder.setOuter(degrees);
		kStartAngle=navPID.nav.getAngle();
		kStartDistance=outerEncoder.getDistance();
		
		
	if(kDegrees<0) {
		kDirection=-1;
	}
		
	//calculate kOuterDistance
	double completeRadius=kRadius+kWheelToCenterDistance;
	kOuterDistance=Math.abs((2*completeRadius*Math.PI)*(kDegrees/360));
	//System.out.println("Outer Distance: "+kOuterDistance);
	//calculate kMiddleDistance
	kMiddleDistance=Math.abs((2*kRadius*Math.PI)*(kDegrees/360));
	//System.out.println("Middle Distance: "+kMiddleDistance);
	//calculate Middle to Outer Distance Ratio
	double midToBigDistanceRatio=kMiddleDistance/kOuterDistance;
	//System.out.println("Mid To Big: "+midToBigDistanceRatio);
	//calculate kInchesPerSecondOuter
	kInchesPerSecondOuter=kInchesPerSecondMiddle/midToBigDistanceRatio;
	//System.out.println("Inches per second outer: "+kInchesPerSecondOuter);
	//calculate kTime
	kTime=kOuterDistance/kInchesPerSecondOuter;
	//System.out.println("Time: "+kTime);
	//calculateDegreesPerSecond
	kDegreesPerSecond=kDegrees/kTime; //ONLY NEGATIVE VALUE
	//System.out.println("Degrees per second: "+kDegreesPerSecond);
	
	controllerSpeed.set(kInchesPerSecondOuter);
	controllerTurn.set(kDegreesPerSecond);
	}
	
	public double turnResult() {
		return controllerTurn.result;
	} 
	
	public double speedResult() {
		return controllerSpeed.result+.3;
	}
	
	public void enable() {
		controllerSpeed.enable();
		controllerTurn.enable();		
	}
	
	public void disable() {
		controllerSpeed.disable();
		controllerTurn.disable();		
	}
	
	public void free() {
		controllerSpeed.free();
		controllerTurn.free();		
	}
	
	public boolean onSpeedTarget() {
		if(controllerSpeed.onTarget()) {
			 return true;
		 }else {
			 return false;
		 }
	}
	
	public boolean onTurnTarget() {
		if(controllerTurn.onTarget()) {
			 return true;
		 }else {
			 return false;
		 }
	}
	
	public boolean doneTurning() {
		if(Math.abs(navPID.nav.getAngle()-kStartAngle)>Math.abs(kDegrees)-2) { //added abs over kDegrees
			return true;
		}
		else {
			return false;
		}
	}
	
	
	
	
	
	
	
}
