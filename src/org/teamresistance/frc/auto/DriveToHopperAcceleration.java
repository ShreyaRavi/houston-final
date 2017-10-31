package org.teamresistance.frc.auto;

import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.util.MecanumDrive;
import org.teamresistance.frc.util.Time;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToHopperAcceleration implements AutoMode {

	private double STOP_DELAY = 1.5;
	private double initialTime;
	
	private double ANGLE = 80.0;
	private double SPEED = 0.85;
	
	private double ACCELERATION = 1.2;
	
	private boolean done = false;
	
	public void init() {
		//SmartDashboard.putNumber("Acceleration Threshhold", ACCELERATION);
		
		initialTime = Time.getTime();
	}
	
	public boolean update() {
		IO.drive.setState(MecanumDrive.DriveType.KNOB_FIELD);
		double acceleration = Math.sqrt((Math.pow(IO.navX.getAHRS().getWorldLinearAccelX(), 2) + Math.pow(IO.navX.getAHRS().getWorldLinearAccelY(), 2)));
		SmartDashboard.putNumber("Acceleration", acceleration);
		ACCELERATION = SmartDashboard.getNumber("Acceleration Threshhold", ACCELERATION);
		if(!done && acceleration > ACCELERATION && Time.getTime() - initialTime > STOP_DELAY) {
			IO.drive.drive(0, 0, 0);
			done = true;
			return true;
		} else if(done) {
			IO.drive.drive(0, 0, 0);
			return true;
		} else {
			IO.drive.drive(SmartDashboard.getNumber("Alliance", 0) * SPEED * Math.sin(Math.toRadians(ANGLE)), -SPEED * Math.cos(Math.toRadians(ANGLE)), 0, 0);
			return false;
		}
		
	}

	@Override
	public String toString() {
		return "Drive To Hopper Acceleration";
	}
}
