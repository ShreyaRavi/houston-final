package org.teamresistance.frc;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.mathd.Rectangle;
import org.teamresistance.frc.util.AutoTargetFollow;
import org.teamresistance.frc.vision.GearPipeline;
import org.teamresistance.frc.vision.ShooterPipeline;

import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * Created by Shreya on 2/20/2017.
 */
public class Shooter {

	private static Shooter instance;
	
	private VisionThread visionThread;
	private Object imgLock = new Object();
	
	private double centerX = 0;
	private double centerY = 0;
	
	private AutoTargetFollow targetFollow = new AutoTargetFollow();
	
	int count = 0;
	
	private Shooter() { 
	}
	
	public void init() {
		visionThread = new VisionThread(IO.shooterCamera, new ShooterPipeline(), pipeline -> {
			SmartDashboard.putNumber("Contour Counts", pipeline.filterContoursOutput().size());
			SmartDashboard.putNumber("Find Contour Counts", pipeline.findContoursOutput().size());
			if(!pipeline.filterContoursOutput().isEmpty()) {
				synchronized(imgLock) {
					count ++;
					Rect rect = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
					centerX = rect.x + rect.width / 2.0;
					centerY = rect.y + rect.height / 2.0;
					SmartDashboard.putNumber("Shooter Count", count);
				}
			}
		});
		visionThread.start();
		
		SmartDashboard.putNumber("Shooter Setpoint", 3100);
		IO.shooterMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		IO.shooterMotor.reverseSensor(false);
		IO.shooterMotor.reverseOutput(false);
		IO.shooterMotor.configEncoderCodesPerRev(20);
		
		// Not sure what this nominal output voltage is for...
		IO.shooterMotor.configNominalOutputVoltage(+0.0, -0.0);
		// Only allow the motor to spin in the forward direction
		IO.shooterMotor.configPeakOutputVoltage(12.0, 0.0);
		//Will never change speed faster than 24V/Sec
		//IO.shooterMotor.setVoltageRampRate(24.0);
	}

	public void update(boolean shooter, boolean agitator, boolean follow) {
		double motorOutput = IO.shooterMotor.getOutputVoltage() / IO.shooterMotor.getBusVoltage();
		SmartDashboard.putNumber("Talon Motor Output", motorOutput);
		
		double shooterSpeed = 0.0;
		
		synchronized(imgLock) { 
			
			//test 1 - works well
			//shooterSpeed = 0.1607 * centerX * centerX - 25.18 * centerX + 4268;
			
			// test 2 - testing
			//shooterSpeed = (0.0309 * centerX * centerX) + (6.6975 * centerX) + 2421.6;
			
			// test 3 - testing testing
			//shooterSpeed = 0.0071 * centerX * centerX + 8.9334 * centerX + 2495.6;
			
//			shooterSpeed = 13.922 * centerX + 1957.1;
			
			shooterSpeed = -0.0403 * centerX * centerX + 30.003 * centerX;
		}
		SmartDashboard.putNumber("Shooter Center X", centerX);
		SmartDashboard.putNumber("Shooter Center Y", centerY);
//		SmartDashboard.putNumber("Shooter Setpoint", shooterSpeed);
		if(shooter) {
			if (follow) {
				targetFollow.update(centerX, centerY);
			}			
			IO.feederMotor.set(1.0);
			IO.shooterMotor.changeControlMode(TalonControlMode.Speed);
//			IO.shooterMotor.set(SmartDashbsoard.getNumber("Shooter Setpoint", 3100));
			IO.shooterMotor.set(3100);
//			IO.shooterMotor.set(shooterSpeed);
			
			if (agitator) {
				IO.agitatorMotor.set(0.45);
				IO.vibratorMotor.set(IO.VIBRATOR_SPEED);
			} else {
				IO.agitatorMotor.set(0.0);
			}
			
			//SmartDashboard.putNumber("Talon Error", IO.shooterMotor.getClosedLoopError());
		} else {
			IO.shooterMotor.changeControlMode(TalonControlMode.PercentVbus);
			IO.shooterMotor.set(0.0);
			IO.feederMotor.set(0.0);
			IO.agitatorMotor.set(0.0);
			IO.vibratorMotor.set(0.0);
		}
		
		SmartDashboard.putNumber("Talon Speed", IO.shooterMotor.getSpeed());
	}
	
	public static Shooter getInstance() {
		if(instance == null) {
			instance = new Shooter();
		}
		return instance;
	}
}
