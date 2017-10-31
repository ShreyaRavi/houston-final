package org.teamresistance.frc.io;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class NavX {
	
	public AHRS ahrs = new AHRS(SPI.Port.kMXP);
	
	public double getNormalizedAngle() {
		return ((ahrs.getAngle() % 360) + 360) % 360;
	}
	
	public double getRawAngle() {
		return ahrs.getAngle();
	}
	
	public void reset() {
		ahrs.reset();
	}
	
	public AHRS getAHRS() {
		return ahrs;
	}

}
