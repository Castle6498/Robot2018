package org.usfirst.frc.team6498.control;
/*
* Base
* gyro
* solenoid
* 2 encoders
* driveBase
* 
 * driveStraight
* driveDistance
* turn
* gearShift
* tankDrive
* arcadeDrive
*/

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Base {
              public AHRS nav;
              public Encoder leftEncoder;
              public Encoder rightEncoder;
              public DifferentialDrive differentialDrive;
              public Solenoid shifter;
              
              public double speed=0;
              public double turn=0;
              public boolean transition=false;
              
              
              public Base(AHRS navC, Encoder leftEncoderC, Encoder rightEncoderC, DifferentialDrive differentialDriveC, Solenoid shifterC) {
                           nav=navC;
                           leftEncoder=leftEncoderC;
                           rightEncoder=rightEncoderC;
                           differentialDrive=differentialDriveC;
                           shifter=shifterC;
                           
                           double wheelDiameter = 6;
                           double ticksPerRevolution = 360;
                           double gearBoxRatio=3; //3:1 Wheel rotates 3 times more than encoder
                           double distancePerPulse=(wheelDiameter*3.141592654)/ticksPerRevolution*gearBoxRatio;
                           leftEncoder.setDistancePerPulse(distancePerPulse);
                           rightEncoder.setDistancePerPulse(distancePerPulse);
                           
                           rightEncoder.setReverseDirection(true);
                           
                           instantiatePID();
                           
                         //SmartDashboard.putData("Gyro",nav);
                           
                           shifter.set(false);
              }
              
              
              
              public PidGyroDisplacement pidGyroDisplacementSource;  
              public PidEncoderAverageRate speedPidSource;
              
              
              public PIDControlHelper turnController;            
              static double kPturnAngle = 0.004;
              static double kIturnAngle = 0.00014;//26;1
              static double kDturnAngle = 0.003;//5;
              static double kFturnAngle = 0.00;
              static double kToleranceDegreesturnAngle = 0.5f;
                
                
                
                 
                
                
                public PIDControlHelper straightController;
                static double kPdriveStraight = .05;//.075;//.002;//0.0015;
                static double kIdriveStraight = .0020;//.002;
                static double kDdriveStraight = 0.01;
                static double kFdriveStraight = 0.00;             
                static double kToleranceDegreesdriveStraight = 0f;
                         
                public PIDControlHelper straightSpeedController;              
                static double kPstraightSpeed = 0.002;
                static double kIstraightSpeed = 0.00;//0.0004;
                static double kDstraightSpeed = 0.00;
                static double kFstraightSpeed = 0.00;         
                static double kToleranceDegreesstraightSpeed = 2.0f;
                
                
                public CurveHelper curveHelper;
                
                
                
                public void instantiatePID() {
              	  	
              	pidGyroDisplacementSource=new PidGyroDisplacement(nav);
              	speedPidSource=new PidEncoderAverageRate(leftEncoder,rightEncoder);
              	
              	
              	turnController = new PIDControlHelper(kPturnAngle, kIturnAngle, kDturnAngle, kFturnAngle, kToleranceDegreesturnAngle, 0, pidGyroDisplacementSource, -180,180);
          		//turnController.turnController.initSendable(builder);	  
              	//SmartDashboard.putData(turnController.turnController);
              	  
          		straightController = new PIDControlHelper(kPdriveStraight, kIdriveStraight, kDdriveStraight, kFdriveStraight, kToleranceDegreesdriveStraight, 0, pidGyroDisplacementSource, -180, 180);
          		//straightController.turnController.initSendable(builder);
          		//SmartDashboard.putData(straightController.turnController);
          		
          		straightSpeedController = new PIDControlHelper(kPstraightSpeed, kIstraightSpeed, kDstraightSpeed, kFstraightSpeed, kToleranceDegreesstraightSpeed, speed, speedPidSource, -130, 130);      			
          		//straightSpeedController.turnController.initSendable(builder);
          		//SmartDashboard.putData(straightSpeedController.turnController);
         		
          		curveHelper=new CurveHelper(leftEncoder, rightEncoder, nav);
          		//SmartDashboard.putData(curveHelper.controllerSpeed.turnController);
          		//SmartDashboard.putData(curveHelper.controllerTurn.turnController);
          		
                } 
              
                
              
              public double getRightDistance() {
            	  return rightEncoder.getDistance();
              }
              
              public double getLeftDistance() {
            	  return leftEncoder.getDistance();
              }
              
              public double angle() {             
                           return nav.getAngle();              
              }
              
              public void move(double speed, double turn) {
                  this.speed=speed;
                  this.turn=turn;
              }
              
              public void execute() {
             	 differentialDrive.arcadeDrive(speed, turn);
           		
           		if(transition) {
           			transition=false;
           		}else {
           			this.speed=0;
               		this.turn=0;
           		}
               }

              public void tankDrive(double leftSpeed, double rightSpeed){
                           differentialDrive.tankDrive(leftSpeed, rightSpeed);
              }
              
              public void arcadeDrive(double speed, double turn) {
                           differentialDrive.arcadeDrive(speed, turn);
              }
              
              public void shiftGear(boolean shift) {
                           shifter.set(shift);
              }
              
              public double getRawTicks() {
            	  return leftEncoder.getRaw();
              }
              
              
            public String turnAngleStage = "start";
          	double targetAngle=0;
          	double slowDownAngle = 15;
          	double slowTurnSpeed = 0.5;
          	int direction;		
          			
          	public boolean turnAngle(double angle){
          		
          		boolean turnAngleStatus = false;
          		
          		switch(turnAngleStage) {
          		
          		case "start":
          			double currentAngle=angle();
          			targetAngle = currentAngle + angle;
          			
          			pidGyroDisplacementSource.lock(targetAngle);
          			//turnController.setInputRange(angle-90, angle+90);
          			turnController.enable();
          			
          			turnAngleStage = "move";
          			
          		break;
          		
          		case "move":
          			
          			this.turn=turnController.result; 
          			
          			if(turnController.onTarget()){      
          				turnAngleStage = "finish";
          			}
          			
          		break;
          		
          		case "finish":
          			
          			turnController.disable();
          			              		
          			turnAngleStatus = true;
          			turnAngleStage = "start";
          			
          		break;
          		
          		}
          		
          		return turnAngleStatus;
          		
          	}
          	
          	
          	/*
          	 * Drive Straight Raw moves the robot forward a distance without using a gyro to keep straight
          	 */
          	
          	public String driveStraightRawStage = "start";
          	double slowDownDistanceRaw = 20;
          	double slowDistanceSpeedRaw = 0.5;
          	double slowDownGiveRaw=1;
          	double straightTargetDistanceRaw = 0;
          	
          	public boolean driveStraightRaw(double distance, double speed, double give) {
          		
          		boolean driveStraightStatus = false;
          		
          		
          		switch(driveStraightRawStage) {
          	
          		case "start":
          			
          			straightTargetDistanceRaw=distance+getLeftDistance();
          			
          			if(distance > 0) {
          				
          				direction = 1;
          				
          			}
          			
          			else if(distance == 0) {
          				
          				driveStraightRawStage = "finish";
          				
          			}
          			
          			else{
          				
          				direction = -1;
          				
          			}
          			
          			driveStraightRawStage = "move";
          			
          		break;
          		
          		case "move":
          			
          			this.speed=speed*direction;
          			
          			if(Math.abs(straightTargetDistanceRaw - getLeftDistance()) <= slowDownDistanceRaw + give) {
          			
          			driveStraightRawStage = "slow down";
          			
          		}
          		
          		break;
          		
          		case "slow down":
          			
          			this.speed=slowDistanceSpeedRaw*direction;
          			
          			if(Math.abs(straightTargetDistanceRaw - getLeftDistance()) <= slowDownGiveRaw) {
          				
          				driveStraightRawStage = "finish";
          				
          			}
          			
          		break;
          		
          		case "finish":
          		
          	
          		driveStraightStatus = true;
          		driveStraightRawStage = "start";
          		
          		break;
          		
          		}
          		
          		return driveStraightStatus;
          		
          	}
          	
          	/*
          	 * Drive Straight moves the robot forward a distance using the gyro to keep straight 
          	 */
          	
          	public String driveStraightStage = "start";
          	double slowDownDistance = 20;
          	double slowDistanceSpeed = 0.5;
          	double slowDownGive=1;
          	double straightTargetDistance = 0;
          	
          	public boolean driveStraight(double distance, double speed, double give) {
          		
          		boolean driveStraightStatus = false;
          		
          		switch(driveStraightStage) {
          	
          		case "start":
          			this.turn=0;
          			//gyroPID=new PidGyroRate(nav);
          			nav.reset();
          			
          			//double currentAngle=angle();
          			
          			straightTargetDistance=distance+getLeftDistance();       			
          			
          			pidGyroDisplacementSource.lock();         			
          			straightController.enable();
          			
          	
          			straightSpeedController.enable();
          			straightSpeedController.set(speed);
          			
          			if(distance > 0) {     				
          				direction = 1;
          			}          			
          			else if(distance == 0) {
          				driveStraightStage = "finish";
          			}else{	
          				direction = -1;        				
          			}       			
          			
          			driveStraightStage = "move";
          			System.out.println("DriveStraight: "+driveStraightStage);
          		break;
          		
          		case "move":
          			this.speed=straightSpeedController.result*direction;
          			this.turn=straightController.turnController.get();
          			
          			if(Math.abs(straightTargetDistance - getLeftDistance()) <= give) {
          			
          			driveStraightStage = "finish";
          			
          			
          		}
          			//System.out.println(" Correction: "+straightController.turnController.get()+" vs "+straightController.result+" Error: "+straightController.turnController.getError());
          		break;
          		
          		case "finish":
          		
          		straightController.disable();
          		
          		
          		straightSpeedController.disable();
          		
          		
          		
          		driveStraightStatus = true;
          		driveStraightStage = "start";
          		
          		break;
          		
          		}
          		
          		return driveStraightStatus;
          		
          	}
          	
          	
          	public String arcPhase="start";
          	public double kickStart=.3;
          	
          	public boolean arc(double degrees, double radius, double speed){
          		
          		
          		boolean arcStatus = false;
          		
          		switch(arcPhase) {
          		
          		case "start":
          			
          			if(degrees>0) {         				
          				kickStart=kickStart;
          			}else {         				
         				kickStart=-kickStart;
          			}
          			
          			
          			curveHelper.calculateParameters(degrees, radius, speed);         			
          			curveHelper.enable();
          			
          			
          			
          			arcPhase = "move";
          			System.out.println("ArcPhase: "+arcPhase);
          		break;
          		
          		case "move":
          			
          			this.turn=curveHelper.turnResult();//+kickStart; 
          			this.speed=curveHelper.speedResult();//+Math.abs(kickStart);
          			
          			if(curveHelper.doneTurning()){      
          				arcPhase = "finish";
          				System.out.println("ArcPhase: "+arcPhase);
          			}
          			//System.out.println(" Correction Turn: "+curveHelper.controllerTurn.turnController.get()+" Speed:  "+curveHelper.controllerSpeed.turnController.get()+" Error Turn: "+curveHelper.controllerTurn.turnController.getError()+ "Error Speed: "+curveHelper.controllerSpeed.turnController.getError());
          			
          		break;
          		
          		case "finish":
          			
          			curveHelper.disable();
          			
          			System.out.println("ArcPhase: "+arcPhase);
          			arcStatus = true;
          			arcPhase = "start";
          			
          		break;
          		
          		}
          		
          		return arcStatus;
          		
          	}
          	
          	public String getDriveStraightStage() {
          		return driveStraightStage;
          	}
          	public String getArcPhase() {
          		return arcPhase;
          	}
          	public String getTurnAngleStage() {
          		return turnAngleStage;        	
          	}
          	
          	public boolean isBusy() {
          		if(turnAngleStage=="start"&&driveStraightRawStage=="start"&&driveStraightStage=="start"&&arcPhase=="start") {
          			return false;
          		}else {
          			return true;
          		}
          	}
          	
          	

}

