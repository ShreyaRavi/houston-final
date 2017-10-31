package org.teamresistance.frc.auto;

import org.teamresistance.frc.Shooter;
import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.mathd.Vector2d;
import org.teamresistance.frc.util.Time;
import org.teamresistance.frc.util.MecanumDrive.DriveType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoTimedShoot implements AutoMode {
	private int currentState = 0;
	 
	private double SIT_FOR_BALLS = 2.75;
	private double STRAIGHT_HOPPER = 0.35;
	private double ANGLE_HOPPER = 1.0;
	private double HOPPER_RAM_TIME = 1.5;
	
	private double initialTime = Time.getTime();
	
	private double driveAngle = 45;
	
	public void init() {
	    currentState = 0;
	}
	
	public boolean update() {  
//		SIT_FOR_BALLS = SmartDashboard.getNumber("SIT_FOR_BALLS",   SIT_FOR_BALLS);   
//		STRAIGHT_HOPPER = SmartDashboard.getNumber("STRAIGHT_HOPPER", STRAIGHT_HOPPER); 
//		ANGLE_HOPPER = SmartDashboard.getNumber("ANGLE_HOPPER",    ANGLE_HOPPER);     
//		HOPPER_RAM_TIME = SmartDashboard.getNumber("HOPPER_RAM_TIME", HOPPER_RAM_TIME);
		boolean done = false;
		double alliance = SmartDashboard.getNumber("Alliance", 1);
		double angle = alliance * driveAngle;
		switch(currentState) {
		// init
		case 0:
			initialTime = Time.getTime();
			currentState = 1;
		
		// drive straight a little
		case 1:
			IO.drive.setState(DriveType.ROTATE_PID);
			if (Time.getTime() - initialTime < STRAIGHT_HOPPER) {
				double speedStraightToHopper = -0.7;
				IO.drive.drive(0, speedStraightToHopper, 0, 0);
			} else {
				currentState = 2;
			}
			Shooter.getInstance().update(false, false, false);
			break;
		// rotate to some angle
		case 2: 
			IO.drive.setState(DriveType.ROTATE_PID);
			if (Math.abs(IO.navX.getRawAngle() - angle) < 5) {
				IO.drive.drive(0,0,0, angle);
			} else {
				currentState = 3;
				initialTime = Time.getTime();
			}
			Shooter.getInstance().update(false, false, false);
			break;
		// drive at some angle along robot's y axis to hopper
		case 3:
			IO.drive.setState(DriveType.ROTATE_PID);
			if (Time.getTime() - initialTime < ANGLE_HOPPER) {
				Vector2d vec = new Vector2d(0,1);
				vec = vec.rotate(Math.toRadians(-angle));
				IO.drive.drive(vec.getX(), -vec.getY(), 0, angle);
			} else {
				currentState = 4;
				initialTime = Time.getTime();
				IO.drive.drive(0,0,0,0);
			}
			Shooter.getInstance().update(false, false, false);
			break;
		// rotate back to 0 degrees, counteract previous momentum
		case 4:
			IO.drive.setState(DriveType.ROTATE_PID);
			if (Math.abs(IO.navX.getRawAngle()) < 5) {
				IO.drive.drive(0, 0.0, 0, 0);
			} else {
				currentState = 5;
				initialTime = Time.getTime();
			}
			Shooter.getInstance().update(false, false, false);
			break;
		// drive sideways to hopper
		case 5:
			// Run Todd's Vibrator?
			IO.drive.setState(DriveType.ROTATE_PID);
			if (Time.getTime() - initialTime < HOPPER_RAM_TIME) {
				double ramSpeed = alliance * 1;
				IO.drive.drive(ramSpeed,0, 0, 0);
			} else {
				IO.drive.drive(0, 0, 0, 0);
				currentState = 6;
				initialTime = Time.getTime();
			}
			IO.vibratorMotor.set(IO.VIBRATOR_SPEED);
			Shooter.getInstance().update(false, false, false);
			break;
		case 6:
			// Run Todd's Vibrator?
			// To move Todd's balls into the robot's hopper
			IO.drive.setState(DriveType.ROTATE_PID);
			if (Time.getTime() - initialTime < SIT_FOR_BALLS) {
				IO.drive.drive(alliance, 0, 0, 0);
			} else {
				IO.drive.drive(0, 0, 0, 0);
				currentState = 7;
				initialTime = Time.getTime();
			}
			IO.vibratorMotor.set(IO.VIBRATOR_SPEED);
			Shooter.getInstance().update(true, false, false);
			break;
		case 7:
			//Start Tracking!
			Shooter.getInstance().update(true, true, true);
			break;
		default:
			done = true;
//			Shooter.getInstance().update(false, false);
			IO.drive.setState(DriveType.ROTATE_PID);
			IO.drive.drive(0, 0, 0, 0);
		}
		return done;
	}

	@Override
	public String toString() {
		return "Auto Timed Shoot";
	}
}
