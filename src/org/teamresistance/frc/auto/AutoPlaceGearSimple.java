//package org.teamresistance.frc.auto;
//
//import org.teamresistance.frc.io.IO;
//import org.teamresistance.frc.mathd.Vector2d;
//import org.teamresistance.frc.util.Time;
//
//public class AutoPlaceGearSimple implements AutoMode {
//
//	private double DRIVE_SPEED = 0.4;
//	private double DRIVE_TIME = 10;
//	private double DELAY_TIME = 1;
//	private double STOP_SPEED = 0.25;
//	
//	private double initialTime;
//	
//	private int currentState = 0;
//	
//	private Vector2d previousPos = new Vector2d(0, 0);
//	
//	public boolean update() {
//		boolean done = false;
//		Vector2d currentPos = IO.ofs.getPos();
//		switch(currentState) {
//		case 0:
//			initialTime = Time.getTime();
//			currentState = 1;
//			IO.drive.drive(0, DRIVE_SPEED, 0);
//		case 1:
//			if(Time.getTime() - initialTime > DRIVE_TIME || (previousPos.sub(currentPos).length() / Time.getDelta() < STOP_SPEED && Time.getTime() - initialTime > DELAY_TIME)) {
//				currentState = -1;
//				done = true;
//			}
//			break;
//		default:
//			IO.drive.drive(0, 0, 0);
//			done = true;
//			break;
//		}
//		previousPos = currentPos;
//		return done;
//	}
//
//	@Override
//	public void init() {
//		
//	}
//	
//}
