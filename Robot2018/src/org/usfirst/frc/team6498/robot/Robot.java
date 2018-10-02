/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6498.robot;

import org.usfirst.frc.team6498.control.*;
import org.usfirst.frc.team6498.control.Lights.Color;
import org.usfirst.frc.team6498.robot.Robot.PinType;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.NidecBrushless;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {

	public XboxController driver;
	public XboxController operator;
	public Base base;
	public Lift lift;
	public Compressor compressor;
	public Claw claw;
	public DropDownArm dropDownArm;
	public Route route;
	public Lights led;
	public TeleOp teleOp;
	
	@Override
	public void robotInit() {
		//usb camera
		CameraServer.getInstance().startAutomaticCapture();
		
		//initialize compressor
		compressor=new Compressor();
		
		//initialize Controllers
		driver=new XboxController(0);
		operator=new XboxController(1);
		
		//initialize drive base
		Spark leftMotor = new Spark(0);
		Spark rightMotor=new Spark(1);
		DifferentialDrive differentialDriveBase= new DifferentialDrive(leftMotor,rightMotor);
		Encoder leftEncoder=new Encoder(0,1);
		Encoder rightEncoder=new Encoder(2,3);
		AHRS nav = new AHRS(SPI.Port.kMXP);
		Solenoid shifter=new Solenoid(0);
		base=new Base(nav, leftEncoder, rightEncoder, differentialDriveBase, shifter);
		
		//initialize lift
		NidecBrushless liftMotorNidec=new NidecBrushless(4, 6);
		DigitalInput liftTopLimit=new DigitalInput(getChannelFromPin( PinType.DigitalIO, 8 ));//6);
		DigitalInput liftBottomLimit=new DigitalInput(7);
		Encoder liftEncoder = new Encoder(4,5);
		Solenoid spindleLock=new Solenoid(4);
		
		Solenoid liftShifter = new Solenoid(3);
		LiftMotor liftMotor=new LiftMotor(liftMotorNidec, liftTopLimit, liftBottomLimit, liftEncoder, spindleLock, liftShifter);
		
		Ultrasonic liftViewSensor=new Ultrasonic(8,9);
		
		lift = new Lift(liftMotor, liftViewSensor, liftShifter); 
		
		//initialize claw
		Spark clawMotor = new Spark(3);
		Solenoid clawSolenoid = new Solenoid(1);
		DigitalInput clawTopLimit = new DigitalInput(getChannelFromPin( PinType.DigitalIO, 2 ));
		DigitalInput clawBottomLimit = new DigitalInput(getChannelFromPin( PinType.DigitalIO, 0));
		claw = new Claw(clawSolenoid, clawMotor, clawTopLimit, clawBottomLimit);
		
		//initialize DropDownArm
		DoubleSolenoid dropDownActuator = new DoubleSolenoid(2,5);
		dropDownArm=new DropDownArm(dropDownActuator);
		
		//initialize Route
		route = new Route(base, lift, claw);
		
		//initialize Led's
		DigitalOutput red=new DigitalOutput(getChannelFromPin( PinType.DigitalIO, 4 ));
		DigitalOutput blue=new DigitalOutput(getChannelFromPin( PinType.DigitalIO, 6 ));
		led=new Lights(red,blue);
		
		//initialize TeleOp program
		teleOp=new TeleOp(driver, operator, base, lift, claw, dropDownArm, led);
		
	}
	
	
	@Override
	public void autonomousInit() {
		//lift.startUltrasonic(true);
		compressor.start();
		
		route.dashboardUpdater();
		route.findGameData();
		route.determineRoute();
		
		lift.motor.motor.enable();
	} 

	public String autoPhase="start";
	@Override
	public void autonomousPeriodic() {
		
		switch(autoPhase) {
		case "start":
			if(route.route()) {  
				autoPhase="done";
			}
			break;
		case "done":
			claw.setOpen(true);	
			break;
			
		}
		
		
		//System.out.println("switch: "+route.switchSide);
		//System.out.println("scale: "+route.scaleSide);
		
		base.execute();
		claw.execute();
		lift.execute();
		
		//System.out.println("Route Phase: "+route.routePhase);
		
		
		}
	@Override
	public void teleopInit() {
		compressor.start();
		//lift.startUltrasonic(true);
		lift.motor.motor.enable();
	}
	
	@Override
	public void teleopPeriodic() {
		
		teleOp.execute();
		
		//System.out.println("Bottom: "+claw.bottomLimit());
		//System.out.println("Top: "+claw.topLimit());
		
		//System.out.println("Claw: "+claw.speed);
		
		//System.out.println("Speed: "+lift.motor.speed);
		
		System.out.println("Height"+lift.height());
		
		//System.out.println("Height: "+lift.height());
		 
		SmartDashboard.setDefaultString("DB/String 8", "GEAR INDICATOR - LED 3");
		SmartDashboard.putString("DB/String 8", "GEAR INDICATOR - LED 3");
		
		SmartDashboard.setDefaultString("DB/String 9", "On-High Off-Low");
		SmartDashboard.putString("DB/String 9", "On-High Off-Low");
		
		SmartDashboard.putBoolean("DB/LED 3", lift.shifter.get()); 
		
		
		//System.out.println("Spindle: "+lift.motor.spindleLock.get());
		
		//led.update();
		
	}

	public void testInit() {
		
		//compressor.stop();
		//lift.startUltrasonic(true);
		lift.motor.lockSpindle(false);
		
		lift.motor.motor.enable();
		
	}
	
	
	String testPhase="pause";  //clawDown
	TimerHelper timer = new TimerHelper();
	
	@Override
	public void testPeriodic() {
		
		//System.out.println("L: "+base.getLeftDistance());
		//System.out.println("R: "+base.getRightDistance());
		
		 
		switch(testPhase) {
		
		//Shift gear using autoShift
		case "autoShift": 
			lift.motor.lockSpindle(false);
			if(timer.stopWatch(2)) { 
				System.out.println("testPhase: "+testPhase);
				testPhase="autoShiftTwo"; 
			}    
			break;
		case "autoShiftTwo":
			if(lift.autoShift()) {	
				System.out.println("testPhase: "+testPhase);
				testPhase="autoShiftThree"; 
			}    
			break;
		case "autoShiftThree":
			if(lift.liftToAdditionalHeight(100, 1, 2)) {
				System.out.println("testPhase: "+testPhase);
				testPhase="done";
			}
			System.out.println("liftingup");
			break;
			
		//Shift Gear at Bottom pause-move up
		case "pause":
			lift.motor.lockSpindle(false);
			lift.shiftGear(true);
			if(timer.stopWatch(2)) { 
				System.out.println("testPhase: "+testPhase);
				testPhase="move up"; 
			}   			
			break;
		case "move up":
			boolean speedStat = lift.setSpeed(1);
			if(!speedStat) {
				System.out.println("testPhase: "+testPhase);
				testPhase="done";
			}
			break;
			
			
		case "start": 
			if(base.driveStraight(200, 80, 2)) {
				System.out.println("testPhase: "+testPhase);
				  testPhase="done"; 
			}    
			base.transition=true; 
			break;
		case "secondStraight": 
			if(base.driveStraight(400, 80, 2)) {
				System.out.println("testPhase: "+testPhase);
				
				testPhase="done";
			} 
			break;  
		case "turn": 
			if(base.arc(90, 60, 30)) { 
				System.out.println("testPhase: "+testPhase);
				testPhase="secondStraight";
			}
			base.transition=true;
			break;
		case "turnTwo":
			if(base.arc(90, 60, 30)) {
				System.out.println("testPhase: "+testPhase);
				testPhase="done";
			}
		 
			break;
		
		
		
		case "liftToAdditional":
			if(lift.liftToAdditionalHeight(20,.5,.5)) {
				System.out.println("testPhase: "+testPhase);
				testPhase="done"; 
			}
			break;
		
		case "liftToScale":
			if(lift.liftToScale(.6,40,36)) {
				System.out.println("testPhase: "+testPhase);
				testPhase="done"; 
			}
			break;
		
		case "liftToSetHeight":
			
			if(lift.liftToSetHeight(20, .6, 2)) {
				System.out.println("testPhase: "+testPhase);
				
				testPhase="done";
				
			}
			break;
			
		case "clawDown":
			
			if(claw.moveDown(-.8)) {
				System.out.println("testPhase: "+testPhase);
				testPhase="done";
				
			}
		
		break;
		
		case "liftAndDrive":
			boolean liftStatus=false;
			boolean driveStatus=false;
			if(!liftStatus&&lift.liftToAdditionalHeight(20,.5,.5)) {			
				liftStatus=true;			
			}
			if(!driveStatus&&base.driveStraight(200, 80, 2)) {			
				driveStatus=true;			
			}
			if(liftStatus&&driveStatus) {
				System.out.println("testPhase: "+testPhase);
				testPhase="done";
			}
			break;
		
		case "clawAndDrive":
			boolean clawStatus=false;
			boolean clawDriveStatus=false;
			if(!clawStatus&&claw.moveDown(-.8)) {			
				clawStatus=true;			
			}
			if(!clawDriveStatus&&base.driveStraight(200, 80, 2)) {			
				clawDriveStatus=true;			
			}
			if(clawDriveStatus&&clawStatus) {
				System.out.println("testPhase: "+testPhase);
				testPhase="done";
			}
			break;
		case "clawDownPin":	
			lift.motor.lockSpindle(false);
			if(claw.moveDown(-.8)&&claw.bottomLimit()) {
				System.out.println("testPhase: "+testPhase);
				testPhase="liftAfterPin";
				System.out.println("claw moved down!!!");
				}
			System.out.println("Bottom Switch"+claw.bottomLimit());
			break;
			
		case "liftAfterPin":	
				if(lift.liftToAdditionalHeight(15,.5,.5)) {
					System.out.println("testPhase: "+testPhase);
					testPhase="done";
					System.out.println("lift moved up!!!");
					claw.setOpen(true);
				}
			break;
			
			
		
		}
		//System.out.println(base.driveStraightStage);
		//System.out.println("Speed:"+lift.motor.speed);
		//System.out.println(lift.motor.spindleLock.get());
		//System.out.println("Status"+lift.motor.driveAddPhase);
		base.execute();
		claw.execute();
		lift.execute();
	}
	
	@Override
	public void disabledInit() {
		base.curveHelper.disable();
		base.arcPhase="start";
		
		base.straightController.disable();
		base.straightSpeedController.disable();
		base.driveStraightStage="start";
		
		base.driveStraightRawStage="start";
		
		base.turnController.disable();
		base.turnAngleStage="start";
	
		base.speed=0;
		base.turn=0;
		
		claw.speed=0;
		claw.moveDownBusy=false;
		
		lift.motor.speed=0;
		lift.motor.drivePhase="start";
		lift.motor.driveAddPhase="start";
		lift.liftToScalePhase="start";
		
		lift.motor.lockSpindle(true);		
		lift.motor.motor.disable();
		
		lift.autoShiftStage="raise";
		
		route.routePhase="start";
		route.clawDownPhase="start";
		route.scaleStraightPhase="start";
		route.scaleCrossPhase="start";
		route.switchPhase="start";
		route.switchCrossPhase="start";
		route.switchCenterRightPhase="start";
		route.switchCenterLeftPhase="start";
		route.switchBlockPhase="start";
		
		autoPhase="start";
		
		
		
	}
	
	@Override
	public void disabledPeriodic() {
		/*led.update();
		route.dashboardUpdater();
		route.findGameData();
		route.determineRoute();*/
	}
	
	 public enum PinType { DigitalIO, PWM, AnalogIn, AnalogOut };
	    public final int MAX_NAVX_MXP_DIGIO_PIN_NUMBER      = 9;
	    public final int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER   = 3;
	    public final int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER  = 1;
	    public final int NUM_ROBORIO_ONBOARD_DIGIO_PINS     = 10;
	    public final int NUM_ROBORIO_ONBOARD_PWM_PINS       = 10;
	    public final int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS  = 4;
	    /* getChannelFromPin( PinType, int ) - converts from a navX-MXP */

	    /* Pin type and number to the corresponding RoboRIO Channel     */

	    /* Number, which is used by the WPI Library functions.          */
	    public int getChannelFromPin( PinType type, int io_pin_number ) 

	               throws IllegalArgumentException {

	        int roborio_channel = 0;

	        if ( io_pin_number < 0 ) {

	            throw new IllegalArgumentException("Error:  navX-MXP I/O Pin #");

	        }

	        switch ( type ) {

	        case DigitalIO:

	            if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {

	                throw new IllegalArgumentException("Error:  Invalid navX-MXP Digital I/O Pin #");

	            }

	            roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS + 

	                              (io_pin_number > 3 ? 4 : 0);

	            break;

	        case PWM:

	            if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {

	                throw new IllegalArgumentException("Error:  Invalid navX-MXP Digital I/O Pin #");

	            }

	            roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_PWM_PINS;

	            break;

	        case AnalogIn:

	            if ( io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER ) {

	                throw new IllegalArgumentException("Error:  Invalid navX-MXP Analog Input Pin #");

	            }

	            roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;

	            break;

	        case AnalogOut:

	            if ( io_pin_number > MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER ) {

	                throw new IllegalArgumentException("Error:  Invalid navX-MXP Analog Output Pin #");

	            }

	            roborio_channel = io_pin_number;            

	            break;

	        }

	        return roborio_channel;

	    }
}
