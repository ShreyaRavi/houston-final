package org.teamresistance.frc;

import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.util.JoystickIO;
import org.teamresistance.frc.util.Time;

public class Gear {

	private final double INSERT_DELAY = 0.5;
	private final double GRAB_DELAY = 0.1;
	private final double START_ROTATE_DELAY = 1.0;
	private final double RELEASE_DELAY = 3.0;
	private final double ROTATE_TIMEOUT = 5.0;
	private final double PICKUP_DELAY = 0.9;
	
	private int currentState = 0;
	private double initialTime = Time.getTime();
	
	public void init() {
		
	}
	
	public void update() {
		switch(currentState) {
		case 0: // Default
			// control lights
			// 
			setPos(true, false, false);
			if(JoystickIO.btnPickupGear.onButtonPressed()) {
				currentState = 1;
				initialTime = Time.getTime();
			} else if(JoystickIO.btnPlaceGear.onButtonPressed()) {
				currentState = 10;
			}
			break;
		case 1: // Waiting
			setPos(false, true, false);
			if(!JoystickIO.btnPickupGear.isDown()) {
				currentState = 2;
			} else if(IO.gearFindBanner.get() && Time.getTime() - initialTime > PICKUP_DELAY) {
				currentState = 2;
			}
			break;
		case 2: // Extend and start delay
			setPos(false, true, true);
			initialTime = Time.getTime();
			currentState = 3;
		case 3:
			if(Time.getTime() - initialTime >= INSERT_DELAY) {
				currentState = 4;
			}
			break;
		case 4: // Expand and start delay
			setPos(true, true, true);
			initialTime = Time.getTime();
			currentState = 5;
		case 5:
			if(Time.getTime() - initialTime >= GRAB_DELAY) {
				currentState = 6;
			}
			break;
		case 6: // Retract
			setPos(true, true, false);
			if(IO.extendSolenoid.isRetracted()) {
				currentState = 7;
			}
			break;
		case 7: // Rotate up and start delay
			setPos(true, false, false);
			initialTime = Time.getTime();
			currentState = 8;
			break;
		case 8:
			if(Time.getTime() - initialTime >= START_ROTATE_DELAY) {
				currentState = 12;
			}
			break;
		case 12: // Run motor to align gear
			initialTime = Time.getTime();
			currentState = 9;
			IO.gearRotatorMotor.set(0.25);
			break;
		case 9:
			if(IO.gearAlignBanner.get() || (Time.getTime() - initialTime >= ROTATE_TIMEOUT)) {
				IO.gearRotatorMotor.set(0.0);
				currentState = 0;
			} else if(JoystickIO.btnPickupGear.onButtonPressed()) {
				IO.gearRotatorMotor.set(0.0);
				currentState = 1;
			}
			break;
		case 10: // Release gear and start delay
			setPos(false, false, false);
			initialTime = Time.getTime();
			currentState = 11;
			break;
		case 11:
			if(Time.getTime() - initialTime >= RELEASE_DELAY) {
				currentState = 0;
			} else if(JoystickIO.btnPickupGear.onButtonPressed()) {
				currentState = 1;
			}
			break;
		default:
			currentState = 0;
			break;
		}
	}
	
	public void setPos(boolean grip, boolean rotate, boolean extend) {
		if(grip) {
			// set lights to false here
			IO.gearLights.extend();
			IO.gripSolenoid.extend();
		} else {
			// set lights to true here
			IO.gearLights.retract();
			IO.gripSolenoid.retract();
		}
		if(rotate)
			IO.rotateSolenoid.extend();
		else
			IO.rotateSolenoid.retract();
		
		if(extend)
			IO.extendSolenoid.extend();
		else
			IO.extendSolenoid.retract();
	}
	
}