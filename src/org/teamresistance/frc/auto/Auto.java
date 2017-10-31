package org.teamresistance.frc.auto;

import java.util.ArrayList;

import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.util.Time;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto {
	
	ArrayList<AutoMode> modes = new ArrayList<>();
	private int currentMode = 1;
	
	private double CLIMBER_JOG_TIME = 0.5;
	private double initialTime;
	private int currentState = 0;
	
	public void init() {
		//currentMode = (int) SmartDashboard.getNumber("Auto Mode", currentMode);
		
		modes.add(new ShootThenDriveOverLineAutoMode());
		modes.add(new AutoTimedShoot());
		modes.add(new GearPlacementLeft());
		/*
		for(int i = 0; i < modes.size(); i++) {
			SmartDashboard.putNumber(modes.get(i).toString(), i);
		}
		*/
		
	    //IO.drive.init(IO.navX.getAngle(), 0.06, 0.0, 0.0);
	    // TODO: Comments
	    modes.get(currentMode).init();
	}
	
	public void update() {
	    switch(currentState) {
	    case 0:
	    	initialTime = Time.getTime();
	    	IO.climberMotor.set(1.0);
	    	currentState = 1;
	    case 1:
	    	if(Time.getTime() - initialTime >= CLIMBER_JOG_TIME) {
	    		currentState = -1;
	    	}
	    	break;
	    default:
	    	IO.climberMotor.set(0.0);
	    	break;
	    }
	    
	    modes.get(currentMode).update();		
	}

}
