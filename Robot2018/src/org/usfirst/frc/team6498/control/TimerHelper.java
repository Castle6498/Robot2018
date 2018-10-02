package org.usfirst.frc.team6498.control;

import edu.wpi.first.wpilibj.Timer;

public class TimerHelper {
	double seconds=0;
	Timer timer;
	
	public TimerHelper() {
		timer=new Timer();
	}
	
	public void start() {
		timer.start();
	}
	
	public void stop() {
		timer.stop();
	}
	
	public double get() {
		return timer.get();
	}
	
	public String timerStatus = "start";
	public boolean stopWatch(double seconds) {
			boolean status=false;
			
			switch(timerStatus) {
			case "start":
				start();
				timerStatus="timer";
				break; 
			case"timer":
				if(get()>=seconds) {
					timerStatus="finish";
				}
				break;
			case "finish":
				stop();
				timerStatus="start";
				status=true;
				break;
			}
			return status;
		}
	
}
