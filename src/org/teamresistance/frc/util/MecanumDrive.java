package org.teamresistance.frc.util;

import org.teamresistance.frc.Constants;
import org.teamresistance.frc.io.NavX;

import edu.wpi.first.wpilibj.RobotDrive;

public class MecanumDrive {
	private RobotDrive drive;
	private NavX gyro;
	
	// PID constants
	private double kP = 0.0; // Proportional constant
	private double kI = 0.0; // Integral constant
	private double kD = 0.0; // Derivative constant
	private double kF = 0.0; // Feed Forward constant
	
	// PID variables
	private double prevError = 0.0; // The error from the previous loop
	private double integral = 0.0; // Error integrated over time
	
	private long prevTime;
	
	private double setpoint; // The target orientation for the robot
	
	private double tolerance = 1;
	
	private double maxOutput = 1.0;
	private double minOutput = -1.0;
	
	private DriveType driveState = DriveType.STICK_FIELD;
	
	private double minToMove = 0.17; // determined through testing forward strafing
	
	public MecanumDrive(RobotDrive drive, NavX gyro) {
		this.drive = drive;
		this.gyro = gyro;
	}
	
	public void init(double setpoint, double p, double i, double d, double f) {
		this.setpoint = setpoint;
		this.kP = p;
		this.kI = i;
		this.kD = d;
		this.kF = f;
		this.prevError = 0.0;
		this.integral = 0.0;
		this.prevTime = System.currentTimeMillis();
	}
	
	public void drive(double x, double y, double angle) {
		drive(x, y, 0, angle);
	}
	
	public void drive(double x, double y, double rotation, double knobAngle) {
		long curTime = System.currentTimeMillis(); 
		double deltaTime = (curTime - prevTime) / 1000.0;
		double gyroAngle = gyro.getRawAngle();
		switch(driveState) {
		case KNOB_FIELD:
			double error = knobAngle - gyroAngle;
			/*
			if(!rotationLatch && Math.abs(error) > rotationLatchDeadband) {
				error = 0;
			} else if(!rotationLatch && Math.abs(error) <= rotationLatchDeadband) {
				rotationLatch = true;
			}
			*/
			if(Math.abs(error) >= 300) {
				if(error > 0) {
					error -= 360;
				} else {
					error += 360;
				}
			}
			
			if(onTarget(error)) error = 0.0;
			integral += error;
			
			double result = (error * kP) + (integral * kI * deltaTime) + ((error - prevError) * kD / deltaTime);
			prevError = error;
			
			
			if(result > maxOutput) result = maxOutput;
			else if(result < minOutput) result = minOutput;
			
			if (Math.abs(result) < minToMove) {
				result = 0;
			}
			
			drive.mecanumDrive_Cartesian(x, y, result, gyroAngle);
			break;
		case STICK_FIELD:
			drive.mecanumDrive_Cartesian(x, y, rotation, gyroAngle);
			break;	
		case ROTATE_PID:
			gyroAngle = gyro.getNormalizedAngle();
			knobAngle = ((knobAngle % 360) + 360) % 360;
			error = knobAngle - gyroAngle;
			if (Math.abs(error) > 180) { // if going around the other way is closer
				if (error > 0) { // if positive
					error = error - 360;
				} else { // if negative
				    error =  error + 360;
				}
			}
			
			double maxI = 0.4;
			if (kI != 0) {
	            double potentialIGain = (integral + error) * kI;
	            if (potentialIGain < maxI) {
	              if (potentialIGain > -maxI) {
	                integral += error;
	              } else {
	                integral = -maxI; // -1 / kI
	              }
	            } else {
	              integral = maxI; // 1 / kI
	            }
	        } else {
	        	integral = 0;
	        }
			
			if (Math.abs(error) < 3.0) {
				error = 0;
			}
			
	        result = (kP * error) + (kI * integral) + (kD * (error - prevError));
	        if (result > 0) {
	        	result += kF;
	        } else {
	        	result -= kF;
	        }
	       	prevError = error;
	       	
	        if (result > 1) {
	          result = 1;
	        } else if (result < -1) {
	          result = -1;
	        }
	        
//	        if(Math.abs(result) < Constants.MIN_ROTATE_SPEED && result > 0) {
//	        	if(result < 0) {
//	        		result = -Constants.MIN_ROTATE_SPEED;
//	        	} else {
//	        		result = Constants.MIN_ROTATE_SPEED;
//	        	}
//	        }
	        
			drive.mecanumDrive_Cartesian(x, y, result, gyroAngle);
			break;
		case STICK_FIELD2:
			// currently untested
			
			// normalize gyro angle between 0 and 360
			gyroAngle = gyro.getNormalizedAngle();
			
			// get cosine and sine of gyro angle
			double cosA = Math.cos(gyroAngle * (3.14159 / 180.0));
		    double sinA = Math.sin(gyroAngle * (3.14159 / 180.0));
		    
		    // convert to robot oriented
		    x = x * cosA - y * sinA;
		    y = x * sinA + y * cosA;
		    
		    // account for difference in x,y power
		    x = 1.7 * x;
		    
		    // limits the output between 1 and -1
		    if (Math.abs(x) > 1) {
		    	if (x > 0) {
		    		x = 1;
		    	} else {
		    		x = -1;
		    	}
		    }
		    
		    x = y * sinA + x * cosA;
		    y = y * cosA - x * sinA;
		    
			drive.mecanumDrive_Cartesian(x, y, rotation, gyroAngle);
			break;
		}
	}
	
//	public boolean driveToPos(Vector2d targetPos, double orientation, double speed) {
//		return driveToPos(targetPos, orientation, speed, STOP_DISTANCE);
//	}
	
//	public boolean driveToPos(Vector2d targetPos, double orientation, double speed, double stopDistance) {
//		Vector2d position = IO.ofs.getPos();
//		
//		//Vector2d direction1 = new Vector2d(SPEED * Math.cos(Math.toRadians(ANGLE)), SPEED * Math.sin(Math.toRadians(ANGLE)));
//		Vector2d error = targetPos.sub(position);
//		Vector2d direction = error.normalized().mul(speed);
//		double dirMag = direction.length();
//		
//		double xOutput;
//		double yOutput;
//		
//		if(Math.abs(direction.getX()) * xyRatio < Math.abs(direction.getY())) {
//			xOutput = xyRatio * direction.getX() / Math.abs(direction.getY()) * Math.min(1, dirMag);
//			yOutput = direction.getY() / Math.abs(direction.getY()) * Math.min(1, dirMag);
//		} else {
//			xOutput = direction.getX() / Math.abs(direction.getX()) * Math.min(1, dirMag);
//			yOutput = direction.getY() / Math.abs(direction.getX()) / xyRatio * Math.min(1, dirMag);
//		}
//		
//		//SmartDashboard.putNumber("OUTPUT X", xOutput);
//		//SmartDashboard.putNumber("OUTPUT Y", yOutput);
//		
//		//SmartDashboard.putNumber("error length", error.length());
//		
//		if(error.length() > stopDistance) {
//			IO.drive.drive(xOutput, -yOutput, 0);
//			return false;
//		}  else {
//			return true;
//		}
//	}
	
	// If the error is less than or equal to the tolerance it is on target
	private boolean onTarget(double error) {
		return Math.abs(error) <= setpoint * tolerance;
	}

	public void nextState() {
		switch(driveState) {
		case KNOB_FIELD:
			driveState = DriveType.STICK_FIELD;
			break;
		case STICK_FIELD:
			driveState = DriveType.KNOB_FIELD;
			break;
		}
	}
	
	public void setMinToMove(double minToMove) {
		this.minToMove = minToMove;		
	}
	
	public void setState(DriveType type) {
		driveState = type;
	}
	
	public DriveType getState() {
		return driveState;
	}
	
	public RobotDrive getDrive() {
		return drive;
	}

	public void setDrive(RobotDrive drive) {
		this.drive = drive;
	}

	public NavX getGyro() {
		return gyro;
	}

	public void setGyro(NavX gyro) {
		this.gyro = gyro;
	}

	public double getkP() {
		return kP;
	}

	public void setkP(double kP) {
		this.kP = kP;
	}

	public double getkI() {
		return kI;
	}

	public void setkI(double kI) {
		this.kI = kI;
	}

	public double getkD() {
		return kD;
	}

	public void setkD(double kD) {
		this.kD = kD;
	}
	
	public void setkF(double kF) {
		this.kF = kF;
	}
	
	public double getkF() {
		return kF;
	}

	public double getSetpoint() {
		return setpoint;
	}

	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}

	public double getTolerance() {
		return tolerance;
	}
	
	public void setTolerance(double tolerance) {
		this.tolerance = tolerance;
	}

	public double getMaxOutput() {
		return maxOutput;
	}

	public void setMaxOutput(double maxOutput) {
		this.maxOutput = maxOutput;
	}

	public double getMinOutput() {
		return minOutput;
	}

	public void setMinOutput(double minOutput) {
		this.minOutput = minOutput;
	}
	
	public enum DriveType {
		KNOB_FIELD,
		STICK_FIELD,
		ROTATE_PID,
		STICK_FIELD2;
	}
}