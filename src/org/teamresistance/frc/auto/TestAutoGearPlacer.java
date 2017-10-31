//package org.teamresistance.frc.auto;
//
//import java.util.ArrayList;
//
//import org.opencv.core.Rect;
//import org.opencv.imgproc.Imgproc;
//import org.teamresistance.frc.Constants;
//import org.teamresistance.frc.io.IO;
//import org.teamresistance.frc.mathd.Rectangle;
//import org.teamresistance.frc.mathd.Vector2d;
//import org.teamresistance.frc.util.Time;
//import org.teamresistance.frc.vision.GearPipeline;
//
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.vision.VisionThread;
//
//public class TestAutoGearPlacer {
//
//	private static TestAutoGearPlacer instance = null;
//	
//	private double imageWidth = 320;
//	private double imageHeight = 240;
//	
//	private VisionThread visionThread;
//
//	private Object imgLock = new Object();
//
//	private ArrayList<Rectangle> rects = new ArrayList<>();
//	private boolean newData = false;
//	
//	private Vector2d center = new Vector2d(144,120);
//	private double distanceXCenters = 0.0;
//	
//	private double xGearNormError = 0.0;
//	private double yGearNormError = 0.0;
//	
//	// PID variables
//	private double prevError = 0.0; // The error from the previous loop
//	private double integral = 0.0; // Error integrated over time
//
//	private double errorDeadband = 5.0;
//	
//	public boolean burst = false;
//	private double initialTime = 0;
//	
//	private double errorX = 0;
//	private double errorY = 0;
//	
//	private double initialTimeCounter = 0;
//	private int count = 0;
//	
//	private TestAutoGearPlacer() { }
//	
//	private void start() {
//		initialTime = Time.getTime();
//		visionThread = new VisionThread(IO.gearCamera, new GearPipeline(), pipeline -> {
//			SmartDashboard.putNumber("Contour Counts", pipeline.filterContoursOutput().size());
//			SmartDashboard.putNumber("Find Contour Counts", pipeline.findContoursOutput().size());
//			if(!pipeline.filterContoursOutput().isEmpty()) {
//				synchronized(imgLock) {
//					count++;
//					SmartDashboard.putNumber("Updates Per Second", count / (Time.getTime() - initialTimeCounter));
//					SmartDashboard.putNumber("Count", count);
//					newData = true;
//					rects.clear();
//					for(int i = 0; i < pipeline.filterContoursOutput().size(); i++) {
//						Rect rect = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));
//						rects.add(new Rectangle(rect.x, rect.y, rect.width, rect.height));
//						SmartDashboard.putData("Contour " + i, rects.get(i));
//					}
//					SmartDashboard.putData("Contour " + rects.size(), rects.get(rects.size()-1));
//				}
//			}
//		});
//		visionThread.start();
//		SmartDashboard.putNumber("Speed Range 1", 0.12);
//		SmartDashboard.putNumber("Speed Range 2", 0.17);
//		SmartDashboard.putNumber("Error Left Range", -1.0);
//		SmartDashboard.putNumber("Error Right Range", 1.0);
//		SmartDashboard.putNumber("Y Gear 1 Speed", -0.3);
//		SmartDashboard.putNumber("Burst Speed", -0.5);
//		SmartDashboard.putNumber("Burst Time", 3);
//		SmartDashboard.putNumber("Last Adj Max Y", 140);
//		SmartDashboard.putNumber("Last Adj Min Y", 124);
//
//		
//	}
//
//	public Vector2d update() {
//		synchronized(imgLock) {
//			if(newData && rects.size() >= 2) {
//				newData = false;
////				for(int i = 0; i < rects.size(); i++) {
////					SmartDashboard.putData("Rectangle " + i, rects.get(i));
////				}
//				// Find pair of objects closest together
//				double minDifference = Double.MAX_VALUE;
//				int pair = -1;
//				for(int i = 0; i < rects.size() - 1; i++) {
//					double difference = Math.max(rects.get(i).size.getY(), rects.get(i+1).size.getY()) - Math.min(rects.get(i).size.getY(), rects.get(i+1).size.getY());
//					if(difference < minDifference) {
//						minDifference = difference;
//						pair = i;
//					}
//				}
//				distanceXCenters = Math.abs(rects.get(pair).getCenter().getX() - rects.get(pair+1).getCenter().getX());
//				center = rects.get(pair).getCenter().add(rects.get(pair+1).getCenter()).div(2);
//			}
//		}
//		
//		// Potentially Array index out of bounds exception
//		if(rects.size() < 2) {
//			return new Vector2d(0.0, 0.0);
//		} 
//		return calcRobotSpeed();		
//	}
//	
//	private Vector2d calcRobotSpeed() {
//		double SETPOINT = 131;
//		SmartDashboard.putData("Center Contours", center);
//		double error = (center.getX() - SETPOINT);
//		SmartDashboard.putNumber("Raw Image Error", error);
//		error /= distanceXCenters;
//		SmartDashboard.putNumber("Normalized Image Error", error);
//		SmartDashboard.putNumber("Distance Between Centers", distanceXCenters);
//		
//		errorX = error;
//		if (!burst) {
//			if (center.getY() > 140) {
//				if (errorX < 0.5 && errorX > -0.15) {
//					return new Vector2d(0, -0.4);
//				} else {
//					double latAdjSpeed = errorX/(Math.abs(errorX)) * 0.18;
//					return new Vector2d(latAdjSpeed, 0);
//				}
//			} else if (center.getY() < 140 && center.getY() > 124) {
//				if (Math.abs(errorX) < (5.0/distanceXCenters)) {
//					burst = true;
//					initialTime = Time.getTime();
//					return new Vector2d(0.0, 0.0);
//				} else {
//					double latAdjSpeed = errorX/(Math.abs(errorX)) * 0.12;
//					return new Vector2d(latAdjSpeed, 0);
//				}
//			} else {
//				return new Vector2d(0.0, 0.0);
//			}
//		} else {
//			double currTime = Time.getTime();
//			if ((currTime - initialTime) <= SmartDashboard.getNumber("Burst Time", 3)) {
//				double burstSpeed = SmartDashboard.getNumber("Burst Speed", -0.5);
//				return new Vector2d(0, burstSpeed);
//			} else {
//				burst = false;
//				return new Vector2d(0,0);
//			}
//		}
//	}
//	
//	public static TestAutoGearPlacer getInstance() {
//		if(instance == null) {
//			instance = new TestAutoGearPlacer();
//			instance.start();
//		}
//		return instance;
//	}
//	
//}