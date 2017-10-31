package org.teamresistance.frc.io;

import org.teamresistance.frc.util.MecanumDrive;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;

public class IO {

	public static double VIBRATOR_SPEED = 0.6;

	// PDP
	public static PowerDistributionPanel pdp = new PowerDistributionPanel(0);

	// Motors -- shooting
	public static CANTalon shooterMotor = new CANTalon(3);

	public static VictorSP feederMotor = new VictorSP(2);
	public static VictorSP agitatorMotor = new VictorSP(6);

	// Motors -- other
	public static VictorSP vibratorMotor = new VictorSP(0);
	public static VictorSP gearRotatorMotor = new VictorSP(5);
	public static VictorSP climberMotor = new VictorSP(9);

	// Banner Sensors for Gear
	public static InvertibleDigitalInput gearFindBanner = new InvertibleDigitalInput(1, true);
	public static InvertibleDigitalInput gearAlignBanner = new InvertibleDigitalInput(2, true);
	public static InvertibleDigitalInput gearRetractedLimit = new InvertibleDigitalInput(0, true);

	// Drive Motors
	public static Victor leftFrontMotor = new Victor(7);
	public static Victor leftRearMotor = new Victor(8);
	public static Victor rightFrontMotor = new Victor(3);
	public static Victor rightRearMotor = new Victor(1);

	// Fist of Death
	// public static Victor leftFrontMotor = new Victor(0);
	// public static Victor leftRearMotor = new Victor(2);
	// public static Victor rightFrontMotor = new Victor(1);
	// public static Victor rightRearMotor = new Victor(3);

	// Pneumatic Cylinders (controlled via Solenoids)
	public static SingleSolenoid gripSolenoid = new InvertibleSolenoid(1, 2, true);
	public static SingleSolenoid extendSolenoid = new InvertibleSolenoidWithPosition(1, 0, false, gearRetractedLimit);
	public static SingleSolenoid rotateSolenoid = new InvertibleSolenoid(1, 1, false);
	public static SingleSolenoid gearLights = new InvertibleSolenoid(1,3, true);
	
	public static Compressor compressor = new Compressor(1);
	public static Relay compressorRelay = new Relay(0);

	// Relay for green LEDs
	public static Relay cameraLights = new Relay(1);
	
	

	// NavX-MXP navigation sensor
	public static NavX navX = new NavX();

	public static MecanumDrive drive = new MecanumDrive(new RobotDrive(IO.leftFrontMotor, IO.leftRearMotor, IO.rightFrontMotor, IO.rightRearMotor), navX);

	public static UsbCamera gearCamera = CameraServer.getInstance().startAutomaticCapture(0);
	public static AxisCamera shooterCamera = new AxisCamera("Shooter Camera", "10.0.86.19");
			//CameraServer.getInstance().addAxisCamera("Axis Camera", "axis-camera.local");
	
	public static OFS ofs = new OFS();

	public static void init() {
		IO.rightFrontMotor.setInverted(true);
		IO.rightRearMotor.setInverted(true);

		IO.climberMotor.setInverted(true);

		IO.feederMotor.setInverted(true);

//		gearCamera.setFPS(20);
		IO.gearCamera.setResolution(320, 240);
//		IO.gearCamera.setBrightness(0);
	}
}
