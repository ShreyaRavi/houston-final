package org.teamresistance.frc.util;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.teamresistance.frc.Constants;
import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.vision.ShooterPipeline;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class AutoTargetFollow {
	
	private long prevTime;
	
	private double tolerance = 0.025; // 0.1  The percent tolerance for the error to be considered on target
	
	private double maxOutput = 0.25;
	private double minOutput = -0.25;
	
	private double centerTarget = 120;
	
	public boolean update(double centerX, double centerY) {
		boolean done = true;
		double error;
		if(Double.isFinite(centerY)) {
			error = centerTarget - centerY;
		} else {
			error = centerTarget;
		}
		
		SmartDashboard.putNumber("Center Y AutoTargetFollow", centerY);
		SmartDashboard.putNumber("Center X AutoTargetFollow", centerX);
		SmartDashboard.putNumber("Error AutoTargetFollow", error);
		
		boolean errorOnTarget = onTargetRotation(error);
		SmartDashboard.putBoolean("Error On Target", errorOnTarget);
		if(errorOnTarget) {
			error = 0.0;
		} else {
			done = false; 
		}
		double result = 0;
		if(error > 0) result = Constants.MIN_ROTATE_SPEED_NEG;
		else if(error < 0) result = Constants.MIN_ROTATE_SPEED_POS;
		
		IO.drive.getDrive().mecanumDrive_Cartesian(JoystickIO.leftJoystick.getX(), JoystickIO.leftJoystick.getY(), result, 0);
		
		return done;
	}
	
	// If the error is less than or equal to the tolerance it is on target
	private boolean onTargetRotation(double error) {
		return Math.abs(error) <= tolerance * centerTarget;
	}
}
