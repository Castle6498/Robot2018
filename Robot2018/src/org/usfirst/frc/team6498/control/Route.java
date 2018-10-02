package org.usfirst.frc.team6498.control;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Route {
	
	//ScaleStraight
	double straightDisSS = 237.046;
	double straightSpSS = 100;
	
	double arcDisSS = 45;
	double arcRadSS = 39.661;
	double arcSpSS = 30;
	
	double deliverDisSS = 12;
	double deliverSpSS = 50;

	//ScaleCross
	double straightDisSC = 175.269;
	double straightSpSC = 100;

	double arcOneRadSC = 39.423;
	double arcOneDisSC = 90;
	double arcOneSpSC = 30;

	double crossSpSC = 80;
	double crossDisSC = 193.068;

	double arcTwoRadSC = 15.91;
	double arcTwoDisSC = 135;
	double arcTwoSpSC = 20;

	double deliverDisSC = 32.863;
	double deliverSpSC = 20;

	//Switch
	double straightDisSW = 128.161;
	double straightSpSW = 100;

	double arcRadSW = 28;//19.423;
	double arcDisSW = 90;
	double arcSpSW = 30;

	double deliverDisSW = 0;
	double deliverSpSW = 0;

	//SwitchCenter Right
	double arcOneDisSWCR = 33.38;
	double arcOneRadSWCR = 90.495;
	double arcOneSpSWCR = 20;

	double arcTwoRadSWCR = 90.495;
	double arcTwoDisSWCR = 33.38;
	double arcTwoSpSWCR = 20;
	
	double deliverDisSWCR = 0;
	double deliverSpSWCR = 0;
	
	//SwithCenter Left
	double arcOneDisSWCL = 76.26;
	double arcOneRadSWCL = 51.26;
	double arcOneSpSWCL = 20;

	double arcTwoRadSWCL = 51.26;
	double arcTwoDisSWCL = 76.26;
	double arcTwoSpSWCL = 20;
	
	double deliverDisSWCL = 0;
	double deliverSpSWCL = 0;
	
	//SwitchCross
	double straightDisSWC = 185;//175.394;
	double straightSpSWC = 100;

	double arcOneRadSWC = 39.423;
	double arcOneDisSWC = 90;
	double arcOneSpSWC = 30;
 
	double crossSpSWC = 80;
	double crossDisSWC = 173;

	double arcTwoRadSWC = 50;//41;//33.554;
	double arcTwoDisSWC = 180;
	double arcTwoSpSWC = 30;

	double deliverDisSWC = 0;
	double deliverSpSWC = 0;
	
	//Switch Block
	double backUpDisSWB = 10;
	double backUpSpSWB = 70;
	
	double turnAngleSWB = 90;
	
	double straightDisSWB = 60.75;
	double straightSpSWB = 80;

	double arcRadSWB = 18.125;
	double arcDisSWB = 180;
	double arcSpSWB = 5;
	
	double deliverDisSWB = 0;
	double deliverSpSWB = 0;
	
	//Direction 
	//if direction is a positive 1, on the left side
	//if direction is a -1, on the right side
	double direction = 1;
	
	
	
	
	public Base base;
	public Claw claw;
	public Lift lift;
	//public Claw claw;
	//public Lift lift;
	String gameData;
	public String switchSide="right";
	public String scaleSide="right";
	public String robotStation="left";
	public boolean overrideDecision=false;
	public String destination="switch";
	public String decidedRoute="switchStraight";
	public String additionalRoute="done";
	
	public static final double liftScaleStraightTime = .5
			;
	public static final double liftScaleCrossTime=5;
	public double liftTime=2;
	
	public static final double clawSwitchTime = 0;
	public static final double clawScaleTime=2;
	public double clawTime=.5;
	
	public Route(Base baseC, Lift lift, Claw claw) {
		base=baseC;
		this.claw=claw;
		this.lift=lift;
		dashboardUpdater();
		findGameData();
		
		
	}  
		 
	public void dashboardUpdater() {
		//SmartDashboard.putBoolean("DB/LED 0", true);
		
		SmartDashboard.setDefaultString("DB/String 5", "Slider 0: 0-L, 1-M, 2-R");
		SmartDashboard.putString("DB/String 5", "Slider 0: 0-L, 1-M, 2-R");
		
		SmartDashboard.setDefaultString("DB/String 6", "LMR is ROBOT location");
		SmartDashboard.putString("DB/String 6", "LMR is ROBOT location");
		
		SmartDashboard.setDefaultString("DB/String 1", "New Name: 0-Sca, 1-Swi");
		SmartDashboard.putString("DB/String 1", "New Name: 0-Sca, 1-Swi");
		
		SmartDashboard.setDefaultString("DB/String 2", "DB/Button1: 0-No Ovr, 1-Ovr");
		SmartDashboard.putString("DB/String 2", "DB/Button1: 0-No Ovr, 1-Ovr");
		
	}
	
	public void findGameData() {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
			System.out.println("Game: "+gameData);
			if(gameData.length()>0) {
				if(gameData.charAt(0)=='L') switchSide="left";
				else if(gameData.charAt(0)=='R') switchSide="right";
				
				if(gameData.charAt(1)=='L') scaleSide="left";
				else if(gameData.charAt(1)=='R') scaleSide="right";
						
			}
			
			int position = (int) SmartDashboard.getNumber("DB/Slider 0", 0);
			
			if(position==0) robotStation="left";
			else if(position==1) robotStation="middle";
			else if(position==2) robotStation="right";
			System.out.println("Position: "+position+" Station: "+robotStation);
			boolean destinationButton = SmartDashboard.getBoolean("DB/Button 0", false);
			SmartDashboard.putBoolean("DB/LED 0", destinationButton); 
			if(destinationButton) destination="switch";
			else if(!destinationButton) destination="scale";
			System.out.println("Destination Button: "+destinationButton+" Destination: "+destination);
			overrideDecision=SmartDashboard.getBoolean("DB/Button 1", false);
			SmartDashboard.putBoolean("DB/LED 1", overrideDecision);
			
			/*
			destination="switch";
			robotStation="right";*/ 
					  
			
		}
	
	public void determineRoute() { 
		
		if(robotStation=="left") {
			
			direction=1;
			
			if(destination=="scale") {
				
				//additionalRoute="done";
				
				if(scaleSide=="left") {
					
					decidedRoute="scaleStraight";
						
				}else if(scaleSide=="right") {
					if(overrideDecision&&switchSide=="left") {
						decidedRoute="switchStraight";
						//additionalRoute="switchBlock";
					}else {
					decidedRoute="scaleCross";
					}
				}
				
			}else if(destination=="switch") {
				
				//additionalRoute="switchBlock";
				
				if(switchSide=="left") {
					
					decidedRoute="switchStraight";
					
				}else if(switchSide=="right") {
					
					decidedRoute="switchCross";
					
				}
				
			}
		}else if(robotStation=="right") {
			direction=-1;
			
			if(destination=="scale") {
				
				//additionalRoute="done";
				
				if(scaleSide=="right") {
					
					decidedRoute="scaleStraight";
					
				}else if(scaleSide=="left") {
					if(overrideDecision&&switchSide=="right") {
						decidedRoute="switchStraight";
						//additionalRoute=""
					}else {
					decidedRoute="scaleCross";
					}
				}
				
			}else if(destination=="switch") {
				
				//additionalRoute="switchBlock";
				
				if(switchSide=="right") {
					
					decidedRoute="switchStraight";
					
				}else if(switchSide=="left") {
					
					decidedRoute="switchCross";
					
				}
			}
			
		}else if(robotStation=="middle"){
			//additionalRoute="done";
			
			if(switchSide=="left") {
				
				decidedRoute="switchCenterLeft";
				
			}else if(switchSide=="right") {
				
				decidedRoute="switchCenterRight"; 
				
			}
		}
		System.out.println("Route: "+decidedRoute+" Direction: "+direction);
		
		
		if(decidedRoute=="scaleStraight") liftTime=liftScaleStraightTime;
		else if(decidedRoute=="scaleCross") liftTime=liftScaleCrossTime;
		

		if(destination=="switch") clawTime = clawSwitchTime;
		else if(destination=="scale") clawTime = clawScaleTime;
	}
	
	
	
	public String routePhase="start";
	public String clawDownPhase="start";
	public String liftUpPhase = "start";
	
	TimerHelper clawTimer = new TimerHelper();
	TimerHelper liftTimer = new TimerHelper();
	public boolean route() {
		boolean status=false;
		switch(routePhase) {
		case "start":
			
			findGameData();
			determineRoute();
			System.out.println("Route Phase: "+routePhase);
			
			routePhase=decidedRoute;
			break;
		case "scaleStraight":
			if(scaleStraight()){
				routePhase=additionalRoute;
			}
			break;
		case "scaleCross":
			if(scaleCross()){
				routePhase=additionalRoute;
			}
			break;
		case "switchStraight":
			if(switchStraight()){
				routePhase=additionalRoute;
			}
			break;
		case "switchCross":
			if(switchCross()){
				routePhase=additionalRoute;
			}
			break;
		case "switchCenterRight":
			if(switchCenterRight()){
				routePhase=additionalRoute;
			}
			break;
		case "switchCenterLeft":
			if(switchCenterLeft()){
				routePhase=additionalRoute;
			}
			break;
		case "switchBlock":
			if(switchBlock()){
				routePhase="done";
			}
			break;
		case"done":
			System.out.println("THE MAGICAL AUTO HAS FINISHED WITH EPICNESS");
			claw.setOpen(true);	
			status=true;
			break;		
		}
		
		
		switch(clawDownPhase){
		case "start":
			if(clawTimer.stopWatch(clawTime)) {
				clawDownPhase="down";
			}
			break;
		case "down":
			if(claw.moveDown(-1)&&claw.bottomLimit()) {
				clawDownPhase="done";
				System.out.println("claw moved down!!!");
				}
			//System.out.println("Bottom Switch"+claw.bottomLimit());
			break;	
		}
		
		if(destination=="switch") {
			
		switch(liftUpPhase) {
		case "start":	
			lift.motor.lockSpindle(false);
			if(liftTimer.stopWatch(.5)) {
				liftUpPhase="lift";
			}
			break;
		case "lift":
			if(lift.liftToAdditionalHeight(24,1,1)) {
				liftUpPhase="done";
				System.out.println("lift moved up!!!");
			}
		break;
		}
		
		}else if(destination=="scale") {
			//Shift Gear at Bottom pause-move up
				switch(liftUpPhase) {	
					case "start":
						lift.motor.lockSpindle(false);
						lift.shiftGear(true);
						if(liftTimer.stopWatch(liftTime)) { 
							System.out.println("liftUpPhase: "+liftUpPhase);
							liftUpPhase="move up"; 
						}   			
						break;
					case "move up":
						boolean speedStat = lift.setSpeed(1);
						if(!speedStat) {
							System.out.println("liftUpPhase: "+liftUpPhase);
							liftUpPhase="done";
						}
						break;
				}
		}
		
		return status;
	}
	
	
	public double pidSetValue=0;
	public double beforeTurn=0;
	
	public String scaleStraightPhase="start";
	public boolean scaleStraight(){
		boolean status=false;
		
		switch(scaleStraightPhase) {
		case "start":
			System.out.println("scaleStraightPhase: "+scaleStraightPhase);
			scaleStraightPhase="straight";
			
			break;
		case "straight":
			if(base.driveStraight(straightDisSS-20,straightSpSS,2)){
				System.out.println("scaleStraightPhase: "+scaleStraightPhase);
				scaleStraightPhase="arc";
			}
			base.transition=true;
			break;
		case "arc":
			if(base.arc(arcDisSS*direction,arcRadSS,arcSpSS)){
				System.out.println("scaleStraightPhase: "+scaleStraightPhase);
				scaleStraightPhase="deliver";
				
			}
			break;
		case "deliver":
			if(base.driveStraight(deliverDisSS,deliverSpSS,2)){
				System.out.println("scaleStraightPhase: "+scaleStraightPhase);
				scaleStraightPhase="done";
				claw.setOpen(true);
			}
			break;
		case "done":
			System.out.println("scaleStraightPhase: "+scaleStraightPhase);
			status=true;
			scaleStraightPhase="start";
			break;
		}
		
		
		return status;
	}
	
	public String scaleCrossPhase="start";
	public boolean scaleCross(){
		boolean status=false;
		
		switch(scaleCrossPhase) {
		case "start":
			System.out.println("scaleCrossPhase: "+scaleCrossPhase);
			scaleCrossPhase="straight";
			
			break;
		case "straight":
			if(base.driveStraight(straightDisSC, straightSpSC, 2)){
				System.out.println("scaleCrossPhase: "+scaleCrossPhase);
				scaleCrossPhase="arcOne";
				beforeTurn=base.angle();
			}
			base.transition=true;
			break;
		case "arcOne":
			if(base.arc(arcOneDisSC*direction, arcOneRadSC, arcOneSpSC)){
				System.out.println("scaleCrossPhase: "+scaleCrossPhase);
				scaleCrossPhase="cross";
			}
			base.transition=true;
			break;
		case "cross":
			if(base.driveStraight(crossDisSC, crossSpSC, 2)){
				System.out.println("scaleCrossPhase: "+scaleCrossPhase);
				scaleCrossPhase="arcTwo";
			}
			base.transition=true;
			pidSetValue=base.straightController.turnController.getSetpoint();
			break;
		case "arcTwo":
			if(base.arc(arcTwoDisSC*(-direction), arcTwoRadSC, arcTwoSpSC)){
				System.out.println("scaleCrossPhase: "+scaleCrossPhase);
				scaleCrossPhase="deliver";
				
			}
			base.transition=true;
			break;
		case "deliver":
			if(base.driveStraight(deliverDisSC, deliverSpSC, 2)){
				System.out.println("scaleCrossPhase: "+scaleCrossPhase);
				scaleCrossPhase="done";
				claw.setOpen(true);
			}
			base.transition=true;
			break;		
		case "done":
			System.out.println("scaleCrossPhase: "+scaleCrossPhase);
			status=true;
			scaleCrossPhase="start";
			break;
		}
		
		return status;
	}
	
	public String switchPhase="start";
	public boolean switchStraight(){
		boolean status=false;
		
		switch(switchPhase) {
		case "start":
			System.out.println("switchPhase: "+switchPhase);
			switchPhase="straight";
			
			break;
		case "straight":
			if(base.driveStraight(straightDisSW, straightSpSW, 2)){
				System.out.println("switchPhase: "+switchPhase);
				switchPhase="arc";
			}
			base.transition=true;
			break;
		case "arc":
			if(base.arc(arcDisSW*direction, arcRadSW, arcSpSW)){
				System.out.println("switchPhase: "+switchPhase);
				switchPhase="deliver";
				claw.setOpen(true);	
			}
			base.transition=true;
			break;
		case "deliver":
			if(base.driveStraight(deliverDisSW, deliverSpSW, 2)){
				System.out.println("switchPhase: "+switchPhase);
				switchPhase="done";
			}
			break;
		case "done":
			System.out.println("switchPhase: "+switchPhase);
			status=true;
			switchPhase="start";
			break;
		}
		
		
		return status;
	}
	
	public String switchCrossPhase="start";
	public boolean switchCross(){
		boolean status = false;
		switch(switchCrossPhase){
		case "start":
			System.out.println("switchCrossPhase: "+switchCrossPhase);
			switchCrossPhase="straight";
				
		break;
		case "straight":
			if(base.driveStraight(straightDisSWC, straightSpSWC, 2)){
				System.out.println("switchCrossPhase: "+switchCrossPhase);
				switchCrossPhase="arcOne";
			}
			base.transition=true;
			break;
		case "arcOne":
			if(base.arc(arcOneDisSWC*direction, arcOneRadSWC, arcOneSpSWC)){
				System.out.println("switchCrossPhase: "+switchCrossPhase);
				switchCrossPhase="cross";
			}
			base.transition=true;
			break;
		case "cross":
			if(base.driveStraight(crossDisSWC, crossSpSWC, 2)){
				System.out.println("switchCrossPhase: "+switchCrossPhase);
				switchCrossPhase="arcTwo";
			}
			base.transition=true;
			break;
		case "arcTwo":
			if(base.arc(arcTwoDisSWC*direction, arcTwoRadSWC, arcTwoSpSWC)){
				System.out.println("switchCrossPhase: "+switchCrossPhase);
				switchCrossPhase="deliver";
				claw.setOpen(true);	
			}
			base.transition=true;
			break;
		case "deliver":
			if(base.driveStraight(deliverDisSWC, deliverSpSWC, 2)){
				System.out.println("switchCrossPhase: "+switchCrossPhase);
				switchCrossPhase="done";
			}
			break;
		case "done":
			System.out.println("switchCrossPhase: "+switchCrossPhase);
		
		switchCrossPhase="start";
		status=true;
		break;
		}
		
		return status;
	}

	public String switchCenterRightPhase="start";
	public boolean switchCenterRight (){
		boolean status = false;
		switch(switchCenterRightPhase){
		case "start":
			System.out.println("switchCenterRightPhase: "+switchCenterRightPhase);
			switchCenterRightPhase="arcOne";
		
		break;
		case "arcOne":
			if(base.arc(arcOneDisSWCR, arcOneRadSWCR, arcOneSpSWCR)){
				System.out.println("switchCenterRightPhase: "+switchCenterRightPhase);
				switchCenterRightPhase="arcTwo";
			}
			break;
		case "arcTwo":
			if(base.arc(-arcTwoDisSWCR, arcTwoRadSWCR, arcTwoSpSWCR)){
				System.out.println("switchCenterRightPhase: "+switchCenterRightPhase);
				switchCenterRightPhase="deliver";
			}
			break;
		case "deliver":
			if(base.driveStraight(deliverDisSWCR, deliverSpSWCR, 2)){
				System.out.println("switchCenterRightPhase: "+switchCenterRightPhase);
				switchCenterRightPhase="done";
			}
			break;
		case "done":
			
		System.out.println("switchCenterRightPhase: "+switchCenterRightPhase);
		switchCenterRightPhase="start";
		status=true;
		break;
		}
		
		return status;
	}

	public String switchCenterLeftPhase="start";
	public boolean switchCenterLeft (){
		boolean status = false;
		switch(switchCenterLeftPhase){
		case "start":
			System.out.println("switchCenterLeftPhase: "+switchCenterLeftPhase);
			switchCenterLeftPhase="arcOne";
		
		break;
		case "arcOne":
			if(base.arc(-arcOneDisSWCL, arcOneRadSWCL, arcOneSpSWCL)){
				System.out.println("switchCenterLeftPhase: "+switchCenterLeftPhase);
				switchCenterLeftPhase="arcTwo";
			}
			break;
		case "arcTwo":
			if(base.arc(arcTwoDisSWCL, arcTwoRadSWCL, arcTwoSpSWCL)){
				System.out.println("switchCenterLeftPhase: "+switchCenterLeftPhase);
				switchCenterLeftPhase="deliver";
			}
			break;
		case "deliver":
			if(base.driveStraight(deliverDisSWCL, deliverSpSWCL, 2)){
				System.out.println("switchCenterLeftPhase: "+switchCenterLeftPhase);
				switchCenterLeftPhase="done";
			}
			break;
		case "done":
			System.out.println("switchCenterLeftPhase: "+switchCenterLeftPhase);
			switchCenterLeftPhase="start";
		status=true;
		break;
		}
		
		return status;
	}

	public String switchBlockPhase="start";
	public boolean switchBlock(){
		boolean status = false;
		switch(switchBlockPhase){
		case "start":
			System.out.println("switchBlockPhase: "+switchBlockPhase);
			switchBlockPhase="backUp";
		
		break;
		case "backUp":
			if(base.driveStraight(-backUpDisSWB, backUpSpSWB, 2)){
				System.out.println("switchBlockPhase: "+switchBlockPhase);
				switchBlockPhase="turn";
			}
			break;
		case "turn":
			if(base.turnAngle(-turnAngleSWB*direction)){
				System.out.println("switchBlockPhase: "+switchBlockPhase);
				switchBlockPhase="straight";
			}
			break;
		case "straight":
			if(base.driveStraight(straightDisSWB, straightSpSWB, 2)){
				System.out.println("switchBlockPhase: "+switchBlockPhase);
				switchBlockPhase="arc";
			}
			break;
		case "arc":
			if(base.arc(arcDisSWB*direction, arcRadSWB, arcSpSWB)){
				System.out.println("switchBlockPhase: "+switchBlockPhase);
				switchBlockPhase="deliver";
			}
			break;
		case "deliver":
			if(base.driveStraight(deliverDisSWB, deliverSpSWB, 2)){
				System.out.println("switchBlockPhase: "+switchBlockPhase);
				switchBlockPhase="done";
			}
			break;
		case "done":
		System.out.println("switchBlockPhase: "+switchBlockPhase);
		switchBlockPhase="start";
		status=true;
		break;
		}
		
		return status;
	}
	

}
