package org.teamresistance.frc.auto;

import org.teamresistance.frc.io.IO;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToHopper {

	private AutoDrivePath path;
	private int currentTarget = 0;
	
	 /* 	    
	   	ArrayList<Vector2d> targetList = new ArrayList<>();
	    targetList.add(new Vector2d(0, 7));
	    init(new Vector2d(0,1.5), targetList);
	    */
	
	public void init(AutoDrivePath path) {
		//SmartDashboard.putNumber("Current Target", currentTarget);
		//SmartDashboard.putNumber("Drive Angle", ANGLE);
		//SmartDashboard.putNumber("Drive Speed", path.speed);
		currentTarget = 0;
		
		IO.ofs.setPos(path.startingPosition);
	}
	
	public boolean update() {
		//SmartDashboard.putNumber("Current Target", currentTarget);
		path.speed = SmartDashboard.getNumber("Drive Speed", path.speed);
		
		boolean arrived = true /*IO.drive.driveToPos(path.targetList.get(currentTarget), 0, path.speed)*/;
		
		if(arrived) {
			if(currentTarget != path.targetList.size() - 1) {
				currentTarget++;
				return false;
			} else {
				return true;
			}
		} else {
			return false;
		}
	}
}
