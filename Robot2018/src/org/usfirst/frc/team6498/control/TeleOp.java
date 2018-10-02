package org.usfirst.frc.team6498.control;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleOp implements Sendable {
	
	public Base base;
	public Claw claw;
	public Lift lift;
	public DropDownArm dropDownArm;
	public Lights led;
	
	//Driver
	public XboxController driver;
	boolean climbControls=false;
	boolean dropDownToggle=false;
	boolean baseGear=false;
	//DropDownArm dropDownArm;
	boolean dropDownPosition=false;
	
	boolean driveReduction=false;
	double driveReductionAmount = .7; //remember a higher number means less reduction
	
	double turnReduction=.85;
	
	//Operator
	public XboxController operator;
	boolean liftGear=false;
	boolean pin=false;
	boolean clawPosition=false;
	
	public SendableBuilder builder;
	
	public TeleOp(XboxController driver, XboxController operator, Base base, Lift lift, Claw claw, DropDownArm dropDownArm, Lights led) {
		this.base=base;
		this.claw=claw;
		this.lift=lift;
		this.driver=driver;
		this.dropDownArm=dropDownArm;
		this.operator=operator;
		this.led=led;
		
		SmartDashboard.putData("TeleOp", this);
	}
	
	
	
	
	public void driverArcadeDrive() {
		double speed=0;
		double turn=driver.getX(Hand.kLeft)*turnReduction;
		if(driver.getTriggerAxis(Hand.kRight)>.05) {
			speed=driver.getTriggerAxis(Hand.kRight);			
		}else if(driver.getTriggerAxis(Hand.kLeft)>.05) {
			speed=-driver.getTriggerAxis(Hand.kLeft);
			turn=-turn;
		}else {
			speed=0;
		}
		if(driveReduction) {
			turn=turn*driveReductionAmount;
			speed=speed*driveReductionAmount;
		}
		base.move(speed, turn);
	}
	
	public void driverBaseGear() {
		if(driver.getBButton()) {
			if(baseGear==false) {
				base.shiftGear(true);
			}
			baseGear=true;
		}else if(driver.getAButton()) {
			if(baseGear==true) {
				base.shiftGear(false);
			}
			baseGear=false;
		}
	}
	
	public void driverPin() {//x-3 y-4
		if(driver.getRawButton(3)) {
			if(pin==false) {
				lift.lockPin(true);
			}
			pin=true;
		}else if(driver.getRawButton(4)) {
			if(pin==true) {
				lift.lockPin(false);
			}
			pin=false;
		}
	}
	
	public void driverLiftSpeed() {
		lift.setSpeed(driver.getY(Hand.kRight));
	}
	
	public void operatorLiftSpeed() {
		lift.setSpeed(-operator.getY(Hand.kLeft));
	}
	
	public void operatorClawSpeed() {
		claw.setSpeed(-operator.getY(Hand.kRight));
		System.out.println(-operator.getY(Hand.kRight));
	}
	
	public void operatorLiftGear() {//x-3 y-4
		if(operator.getRawButton(3)) {
			if(liftGear==false) {
				lift.shiftGear(true);
			}
			liftGear=true;
		}else if(operator.getRawButton(4)) {
			if(liftGear==true) {
				lift.shiftGear(false);
			}
			liftGear=false;
		}
	}
	
	public void driverLiftGear() {//x-3 y-4
		if(driver.getBumper(Hand.kRight)) {
			if(liftGear==false) {
				lift.shiftGear(true);
			}
			liftGear=true;
		}else if(driver.getBumper(Hand.kLeft)) {
			if(liftGear==true) {
				lift.shiftGear(false);
			}
			liftGear=false;
		}
	}

	public void operatorClaw() {
		if(operator.getBumper(Hand.kLeft)) {
			if(clawPosition==false) {
				claw.setOpen(true);
			}
			clawPosition=true;
		}else if(operator.getBumper(Hand.kRight)) {
			if(clawPosition==true) {
				claw.setOpen(false);
			}
			clawPosition=false;
		}
	}

	public void driverDropDownArm() {
		if(climbControls) {
			
		if(driver.getRawButton(7)) {
			dropDownPosition=true;
		}
		
		}else {
			dropDownPosition=false;
		}
		
		dropDownArm.drop(dropDownPosition);
		
	}
	
	public void testForDriveReduction() {
		
		if(climbControls) {
			driveReduction=true;
		}else if(driver.getRawButton(4)) { //x-3 y-4
			driveReduction=true;
		}else {
			driveReduction=false;
		}
		
	}
	
	public void execute() {
	
		driverArcadeDrive();
		
		
		driverBaseGear();
		
		
		
		System.out.println("Climb Controls: "+climbControls);
		if(driver.getRawButtonReleased(8)) {
			if(climbControls) {
			climbControls=false;
			}else {
			climbControls=true;
			}
		}
		
		testForDriveReduction();
		driverDropDownArm();
		
		if(climbControls) {
			driverPin();
			driverLiftSpeed();
			driverLiftGear();
			
		}else {
			operatorLiftSpeed();
			operatorClawSpeed();
			operatorLiftGear();
			operatorClaw();
		}
		
		base.execute();
		lift.execute();
		claw.execute();
	}
	
	boolean getBaseGear(){
		return baseGear;
	}
	boolean getLiftGear(){
		return liftGear;
	}
	boolean getPin(){
		return pin;
	}
	boolean getClawPosition(){
		return clawPosition;
	}
	boolean getDropDownArm(){
		return dropDownPosition;
	}
	
	
	@Override
	public void initSendable(SendableBuilder builder) {
		// TODO Auto-generated method stub
		// TODO Auto-generated method stub
				 builder.addBooleanProperty("baseGear",this::getBaseGear, null);
				 builder.addBooleanProperty("liftGear",this::getLiftGear, null);
				 builder.addBooleanProperty("pin",this::getPin, null);
				 builder.addBooleanProperty("clawPosition",this::getClawPosition, null);
				 builder.addBooleanProperty("dropDownArm",this::getDropDownArm, null);
	}

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setName(String name) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public String getSubsystem() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setSubsystem(String subsystem) {
		
		
		 
	}

}
