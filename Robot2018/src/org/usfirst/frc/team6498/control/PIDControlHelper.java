package org.usfirst.frc.team6498.control;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class PIDControlHelper implements PIDOutput {
	
	public PIDController turnController;
    public double result;
    
     double kP = 0.14;

     double kI = 0.001;

     double kD = 0.00;

     double kF = 0.00;
     
     double kToleranceDegrees = 2.0f;
	
	
	public PIDControlHelper(double kP, double kI, double kD, double kF,  double kToleranceDegrees, double setPoint, PIDSource nav, double inputMin, double inputMax) {
		this.kP=kP;
		this.kI=kI; 
		this.kD=kD;
		this.kF=kF;
		this.kToleranceDegrees=kToleranceDegrees;
		
		turnController = new PIDController(kP, kI, kD, kF, nav, this,.02);

		turnController.setInputRange(inputMin,  inputMax);

		turnController.setOutputRange(-1.0, 1.0); 

		turnController.setAbsoluteTolerance(kToleranceDegrees);

		turnController.setContinuous(false);
		//turnController.setContinuous(true);
			
		turnController.setSetpoint(setPoint);
		//System.out.println("Set: "+setPoint);
	}
	
	 public void pidWrite(double output) {
         result = output;
         //System.out.println("Output: "+output);
     }  
	 
	 public void enable() {
		 turnController.enable();
	 }
	 
	 public void disable() {
		 turnController.disable();
	 }
	 
	 public void free() {
		 turnController.free();
	 }
	 
	 public void setP(double p) {
		 turnController.setP(p);
	 }
	 
	 public void setI(double i) {
		 turnController.setI(i);
	 }
	 
	 public void setD(double d) {
		 turnController.setD(d);
	 }
	 
	 public void setF(double f) {
		 turnController.setF(f);
	 }
	 
	 public void setTolerance(double tolerance) {
		 turnController.setAbsoluteTolerance(tolerance);
	 }
	 
	 public void setOutputRange(double minimumOutput, double maximumOutput) {
		 turnController.setOutputRange(minimumOutput, maximumOutput);
	 }
	 
	 public void setInputRange(double  minimumInput, double maximumInput) {
		 turnController.setInputRange(minimumInput, maximumInput);
	 }
	 	
	 
	 public void set(double setPoint) {
		 turnController.setSetpoint(setPoint);
	 }
	 
	 public boolean onTarget() {
		 if(turnController.onTarget()) {
			 return true;
		 }else {
			 return false;
		 }
		 
	 }
	
}
