package org.teamresistance.frc.auto;

import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.mathd.Vector2d;
import org.teamresistance.frc.util.Time;
import org.teamresistance.frc.util.MecanumDrive.DriveType;

public class GearPlacementLeft implements AutoMode {

	private int state = 0;
	
	private double initialTime;
	
	@Override
	public void init() {
		initialTime = Time.getTime();
	}

	@Override
	public boolean update() {
		switch(state) {
		case 0:
			initialTime = Time.getTime();
			state = 1;
			break;
		case 1:
			IO.drive.drive(0, -0.5, 0, 0);
			if(Time.getTime() - initialTime > 1.27) {
				state = 2;
				initialTime = Time.getTime();
			}
			break;
		case 2:
			IO.drive.drive(0, 0, 0, 0);
			if(Time.getTime() - initialTime > 0.2) {
				state = 3;
				initialTime = Time.getTime();
			}
			break;
		case 3:
//			Vector2d speed = NewAutoGearPlacer.getInstance().update();
			IO.drive.setState(DriveType.ROTATE_PID);
			IO.drive.drive(0, 0, 0, 330);
			break;
		}
		return false;
	}
}
