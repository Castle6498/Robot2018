package org.usfirst.frc.team6498.control;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lights {
	
	String status = "castle";
	public DigitalOutput red;
	public DigitalOutput blue;
	
	public Lights(DigitalOutput redC, DigitalOutput blueC) {
		red=redC;
		blue=blueC;
	}
	
	public enum Color {red, blue, castle};
	
	public void set(Color mode) {
		if(mode==Color.red) {
			status="red";
			red.set(true);
			blue.set(false);
			System.out.println("Led: Red");
		}else if(mode==Color.blue) {
			status="blue";
			red.set(false);
			blue.set(true);
			System.out.println("Led: Blue");
		}else {
			status="castle";
			red.set(false);
			blue.set(false);
			System.out.println("Led: Castle");
		}
	}
	
	public void update() {
		
		SmartDashboard.setDefaultString("DB/String 8", "Slider 3: 0-C, 1-R, 2-B");
		SmartDashboard.putString("DB/String 8", "Slider 3: 0-C, 1-R, 2-B");
		
		int position = (int) SmartDashboard.getNumber("DB/Slider 3", 0);
		
		if(position==0) set(Color.castle);
		else if(position==1) set(Color.red);
		else if(position==2) set(Color.blue);		
	}
	
}


/*
KAYWA HI KAYWA
*/