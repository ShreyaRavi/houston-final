package org.teamresistance.frc.auto;

import org.teamresistance.frc.Robot;
import org.teamresistance.frc.Shooter;
import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.util.MecanumDrive.DriveType;
import org.teamresistance.frc.util.Time;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootThenDriveOverLineAutoMode implements AutoMode {

	private static final double DELAY_TILL_SHOOT = 1.0;
	private static final double SHOOT_TIME = 4.0;
	private static final double DRIVE_TIME = 4;
	private static final double ROTATE_TIME = 0.75/2.0;
	
	private static final double DRIVE_SPEED = 1.0;
	private static final double ROTATE_SPEED = 1.0;
	
	private double initialTime;
	private int currentState = 0;
	
	@Override
	public void init() {
		IO.drive.setState(DriveType.KNOB_FIELD);
	}

	@Override
	public boolean update() {
		switch(currentState) {
		case 0:
			initialTime = Time.getTime();
			currentState = 1;
			Shooter.getInstance().update(true, false, false);
		case 1:
			if(Time.getTime() - initialTime > DELAY_TILL_SHOOT) {
				currentState = 2;
				initialTime = Time.getTime();
			}
			Shooter.getInstance().update(true, false, false);
			break;
		case 2:
			if(Time.getTime() - initialTime > SHOOT_TIME) {
				currentState = 3;
				initialTime = Time.getTime();
			}
			Shooter.getInstance().update(true, true, false);
			SmartDashboard.putNumber("Auto CenterX*******", SmartDashboard.getNumber("Shooter Center X", Double.MAX_VALUE));
			break;
		case 3:
			if(Time.getTime() - initialTime > DRIVE_TIME) {
				currentState = 4;
				initialTime = Time.getTime();
			}
			IO.drive.drive(DRIVE_SPEED * SmartDashboard.getNumber("Alliance", 0), 0.2, 0.0, 0.0);
			Shooter.getInstance().update(false, false, false);
			break;
		case 4:
			IO.drive.setState(DriveType.STICK_FIELD);
			if(Time.getTime() - initialTime > ROTATE_TIME) {
				currentState = -1;
				initialTime = Time.getTime();
				IO.drive.setState(DriveType.STICK_FIELD);
			}
			IO.drive.drive(0.0, 0.0, ROTATE_SPEED * SmartDashboard.getNumber("Alliance", 0), 0.0);
			Shooter.getInstance().update(false, false, false);
			break;
//		case 5:
//			/*
//			if(Time.getTime() - initialTime > ROTATE_TIME) {
//				currentState = -1;
//				initialTime = Time.getTime();
//			}
//			IO.drive.drive(DRIVE_SPEED * SmartDashboard.getNumber("Alliance", 0), 0.0, 0, 0.0);
//			Robot.shooter.update(false, false);
//			break;
		default:
			IO.drive.drive(0.0, DRIVE_SPEED, 0.0, 0.0);
			Shooter.getInstance().update(false, false, false);
			break;
		}
		
		return false;
	}

	@Override
	public String toString() {
		return "Shoot Then Drive Over Line";
	}
	
	
}
