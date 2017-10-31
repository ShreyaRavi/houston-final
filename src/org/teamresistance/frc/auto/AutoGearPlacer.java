package org.teamresistance.frc.auto;

import java.util.ArrayList;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.mathd.Rectangle;
import org.teamresistance.frc.mathd.Vector2d;
import org.teamresistance.frc.util.JoystickIO;
import org.teamresistance.frc.util.Time;
import org.teamresistance.frc.vision.GearPipeline;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class AutoGearPlacer {

	private static AutoGearPlacer instance = null;
	
	private double imageWidth = 320;
	private double imageHeight = 240;
	
	private VisionThread visionThread;

	private Object imgLock = new Object();

	private ArrayList<Rectangle> rects = new ArrayList<>();
	private boolean newData = false;
	
	private Vector2d center = new Vector2d(144,120);
	private double distanceCenters = 0.0;
	
	private double prevXError = 0.0;
	
	private final double STOP_DISTANCE = 123;
	
	private AutoGearPlacer() { }
	
	private void start() {
		visionThread = new VisionThread(IO.gearCamera, new GearPipeline(), pipeline -> {
			SmartDashboard.putNumber("Contour Counts", pipeline.filterContoursOutput().size());
			SmartDashboard.putNumber("Find Contour Counts", pipeline.findContoursOutput().size());
			if(!pipeline.filterContoursOutput().isEmpty()) {
				synchronized(imgLock) {
					newData = true;
					rects.clear();
					for(int i = 0; i < pipeline.filterContoursOutput().size(); i++) {
						Rect rect = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));
						rects.add(new Rectangle(rect.x, rect.y, rect.width, rect.height));
						SmartDashboard.putData("Contour " + i, rects.get(i));
					}
					SmartDashboard.putData("Contour " + rects.size(), rects.get(rects.size()-1));
				}
			}
		});
		visionThread.start();
		SmartDashboard.putNumber("kP Gear Lateral Translate", 0.0);
	}

	public Vector2d update() {
		synchronized(imgLock) {
			if(newData && rects.size() >= 2) {
				newData = false;
//				for(int i = 0; i < rects.size(); i++) {
//					SmartDashboard.putData("Rectangle " + i, rects.get(i));
//				}
				// Find pair of objects closest together
				double minDifference = Double.MAX_VALUE;
				int pair = -1;
				for(int i = 0; i < rects.size() - 1; i++) {
					double difference = Math.max(rects.get(i).size.getY(), rects.get(i+1).size.getY()) - Math.min(rects.get(i).size.getY(), rects.get(i+1).size.getY());
					if(difference < minDifference) {
						minDifference = difference;
						pair = i;
					}
				}
				distanceCenters = Math.abs(rects.get(pair).size.getX() - rects.get(pair+1).size.getX());
				center = rects.get(pair).getCenter().add(rects.get(pair+1).getCenter()).div(2);
			}
		}
		
		// Potentially Array index out of bounds exception
		if(rects.size() < 2) {
			return new Vector2d(0.0, 0.0);
		} 
		
		double robotXSpeed = calcRobotXSpeed();
		double robotYSpeed = calcRobotYSpeed(robotXSpeed);
		
		return new Vector2d(robotXSpeed, robotYSpeed);
	}
	
	private double calcRobotXSpeed() {
		final double ERROR_DEADBAND = 5.0;
		final double SETPOINT = 144;
		SmartDashboard.putData("Center Contours", center);
		SmartDashboard.putNumber("Center Image X", SETPOINT);
		double error = (center.getX() - SETPOINT) / distanceCenters;
		double result;
		
		double errorDeadbandConstantOfProportionality = SmartDashboard.getNumber("ERROR_DEADBAND P", 0.025);
		
		if(Math.abs(error) < ERROR_DEADBAND / distanceCenters * errorDeadbandConstantOfProportionality * (center.getY() - STOP_DISTANCE)) {
			error = 0.0;
			result = 0.0;
		} else {
			double kP = SmartDashboard.getNumber("kP Gear Lateral Translate", 0.0);
			result = (error * kP);
		}
		
		prevXError = error;
		
		return xSpeedCorrection(result);
	}
	
	private double calcRobotYSpeed(double xSpeed) {
		double result = 0.0;
		
		if(center.getY() > STOP_DISTANCE) {
			result = -0.45;
		}
		
		return result;
	}
	
	private double xSpeedCorrection(double in) {
		final double minMoveSpeed = 0.2; // 0.135
		double result = (1 - minMoveSpeed) * in;
		
		if(in == 0.0) {
			result = 0.0;
		} else if(in < 0.0) {
			result -= minMoveSpeed;
		} else if(in > 0.0) {
			result += minMoveSpeed;
		} else if(in < -1.0) {
			result = -1.0;
		} else if(in > 1.0) {
			result = 1.0;
		}
		
		return result;
	}
	
	public static AutoGearPlacer getInstance() {
		if(instance == null) {
			instance = new AutoGearPlacer();
			instance.start();
		}
		return instance;
	}
	
}
