package org.teamresistance.frc;

import org.teamresistance.frc.auto.Auto;
//import org.teamresistance.frc.auto.NewAutoGearPlacer;
import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.mathd.Rectangle;
import org.teamresistance.frc.util.JoystickIO;
import org.teamresistance.frc.util.Time;
import org.teamresistance.frc.util.MecanumDrive.DriveType;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Main robot class. Override methods from {@link IterativeRobot} to define
 * behavior.
 */
public class Robot extends IterativeRobot {

	public Teleop teleop;
	public Vision vision;

	public static Auto auto = new Auto();

	@Override
	public void robotInit() {
		// Potential Anarchy
		SmartDashboard.putNumber("Alliance", 0);

		IO.init();

		IO.drive.init(1, 0.025, 0.000, 0.03, 0.0);

		SmartDashboard.putNumber("Rotate P", 0.025);
		SmartDashboard.putNumber("Rotate D", 0.03);

		teleop = new Teleop();
		teleop.init();

		vision = new Vision();
		vision.init();

		// NewAutoGearPlacer.getInstance();
		Shooter.getInstance().init();
	}

	@Override
	public void teleopInit() {
		IO.navX.reset();
		IO.drive.setState(DriveType.STICK_FIELD);
		teleop.init();
	}

	@Override
	public void teleopPeriodic() {
		IO.compressorRelay.set(IO.compressor.enabled() ? Relay.Value.kForward : Relay.Value.kOff);

		IO.drive.setkP(SmartDashboard.getNumber("Rotate P", 0.025));
		IO.drive.setkD(SmartDashboard.getNumber("Rotate D", 0.03));

		Time.update();
		JoystickIO.update();
		teleop.update();
	}

	public void autonomousInit() {
		auto.init();
	}

	@Override
	public void autonomousPeriodic() {
		auto.update();
	}

	@Override
	public void disabledInit() {
		// teleop.disable();
	}
}
