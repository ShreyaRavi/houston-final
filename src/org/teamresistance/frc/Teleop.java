package org.teamresistance.frc;

import org.teamresistance.frc.auto.AutoGearPlacer;
//import org.teamresistance.frc.auto.NewAutoGearPlacer;
//import org.teamresistance.frc.auto.TestAutoGearPlacer;
import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.mathd.Vector2d;
import org.teamresistance.frc.util.JoystickIO;
import org.teamresistance.frc.util.MecanumDrive.DriveType;
import org.teamresistance.frc.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by ShReYa on 2/20/2017.
 */
public class Teleop {

	private Climber climber;
	private Gear gear;

	private boolean grabAngleOnce = true;
	private double holdAngle = 0;
	
	public void init() {
		climber = new Climber();
		gear = new Gear();
		climber.init();
		gear.init();
		IO.drive.setState(DriveType.STICK_FIELD);
		SmartDashboard.putNumber("Rotate Speed", 0.0);
		SmartDashboard.putNumber("Y Speed", 0.0);
		SmartDashboard.putNumber("X Speed", 0.0);
		AutoGearPlacer.getInstance();
		
		holdAngle = IO.navX.getNormalizedAngle();
		
		SmartDashboard.putNumber("Min X", Constants.MIN_X_SPEED);
		SmartDashboard.putNumber("Min Y", Constants.MIN_Y_SPEED);
		SmartDashboard.putNumber("Min Rotate", Constants.MIN_ROTATE_SPEED_POS);
	}

	public void update() {
		SmartDashboard.putNumber("Joystick 0 X", JoystickIO.leftJoystick.getX());
		SmartDashboard.putNumber("Joystick 0 Y", JoystickIO.leftJoystick.getY());
		SmartDashboard.putNumber("Joystick 1 X", JoystickIO.rightJoystick.getX());
		SmartDashboard.putNumber("Joystick 1 Y", JoystickIO.rightJoystick.getY());
		SmartDashboard.putNumber("Joystick 2 X", JoystickIO.coJoystick.getX());
		SmartDashboard.putNumber("Joystick 2 Y", JoystickIO.coJoystick.getY());

		SmartDashboard.putNumber("Gyro Normalized Angle", IO.navX.getNormalizedAngle());
		SmartDashboard.putNumber("Gyro Raw Angle", IO.navX.getRawAngle());

		double scaledX = Util.scaleInput(JoystickIO.leftJoystick.getX());
		double scaledY = Util.scaleInput(JoystickIO.leftJoystick.getY());
		double scaledRotate = Util.scaleInput(JoystickIO.rightJoystick.getX());
		
		if (JoystickIO.btnHoldLeft.isDown()) {
			IO.drive.setState(DriveType.ROTATE_PID);
			IO.drive.drive(scaledX, scaledY, 0, 330);
			grabAngleOnce = true;
		} else if (JoystickIO.btnHoldCenter.isDown()) {
			IO.drive.setState(DriveType.ROTATE_PID);
			IO.drive.drive(scaledX, scaledY, 0, 270);
			grabAngleOnce = true;
		} else if (JoystickIO.btnHoldRight.isDown()) {
			IO.drive.setState(DriveType.ROTATE_PID);
			IO.drive.drive(scaledX, scaledY, 0, 210);
			grabAngleOnce = true;
		} else if (JoystickIO.btnHoldLeftHopper.isDown()) {
			IO.drive.setState(DriveType.ROTATE_PID);
			IO.drive.drive(-1, scaledY, 0, 0);
			grabAngleOnce = true;
		} else if (JoystickIO.btnHoldRightHopper.isDown()) {
			IO.drive.setState(DriveType.ROTATE_PID);
			IO.drive.drive(1, scaledY, 0, 180);
			grabAngleOnce = true;
		} else if(JoystickIO.leftJoystick.getRawButton(4)) {
			IO.drive.drive(SmartDashboard.getNumber("Min X", Constants.MIN_X_SPEED), 0, 0, 0);
		} else if(JoystickIO.leftJoystick.getRawButton(3)) {
			IO.drive.drive(0, SmartDashboard.getNumber("Min Y", Constants.MIN_Y_SPEED), 0, 0);
		} else if(JoystickIO.leftJoystick.getRawButton(5)) {
			IO.drive.getDrive().mecanumDrive_Cartesian(0, 0, SmartDashboard.getNumber("Min Rotate", Constants.MIN_ROTATE_SPEED_POS), 0);
		} else {
			SmartDashboard.putNumber("Angle Being Held", holdAngle);
			if (scaledRotate == 0) {
				SmartDashboard.putBoolean("Is Holding Angle", true);
				if (grabAngleOnce) {
					grabAngleOnce = false;
					holdAngle = IO.navX.getNormalizedAngle();
				}
				IO.drive.setState(DriveType.ROTATE_PID);
				IO.drive.drive(scaledX, scaledY, 0, holdAngle);
			} else {
				grabAngleOnce = true;
				IO.drive.setState(DriveType.STICK_FIELD);
				IO.drive.drive(scaledX, scaledY, scaledRotate, 0);
			}
		}
		Shooter.getInstance().update(JoystickIO.btnShooter.isDown(), JoystickIO.btnAgitator.isDown(), false);

		climber.update();
		gear.update();
		
		if(JoystickIO.btnShooter.isDown()) {
			grabAngleOnce = true;
		}

		if (JoystickIO.btnGyroReset.onButtonPressed()) {
			IO.navX.reset();
			holdAngle = 0;
		}
	}

	public void disable() {

	}
}
