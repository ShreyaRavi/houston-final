package org.teamresistance.frc;

import org.teamresistance.frc.io.IO;

public class Vision {

	// All 0's here are good for OpenCV detection
	private static final int DEFAULT_BRIGHTNESS = 0;
	private static final int DEFAULT_EXPOSURE = 0;
	private static final int DEFAULT_WHITE_BALANCE = 3000; // kFixedIndoor

	public void init() {
//		IO.gearCamera.setBrightness((int) SmartDashboard.getNumber(
//				"Camera Brightness", DEFAULT_BRIGHTNESS));
//		IO.gearCamera.setExposureManual((int) SmartDashboard.getNumber(
//				"Camera Exposure", DEFAULT_EXPOSURE));
//		IO.gearCamera.setWhiteBalanceManual((int) SmartDashboard.getNumber(
//				"Camera White Balance", DEFAULT_WHITE_BALANCE));
	}

	public void update() {
		IO.gearCamera.setBrightness(DEFAULT_BRIGHTNESS);
		IO.gearCamera.setExposureManual(DEFAULT_EXPOSURE);
		IO.gearCamera.setWhiteBalanceManual(DEFAULT_WHITE_BALANCE);
	}
}
