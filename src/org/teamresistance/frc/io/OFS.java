package org.teamresistance.frc.io;

import org.teamresistance.frc.mathd.Vector2d;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * Created by Joseph for testing the Optical Flow
 *
 * Registers
 * 0x00  -   Product ID
 * 0x01  -   Revision ID
 * 0x02  -   Motion
 * 0x03  -   Delta x
 * 0x04  -   Delta y
 * 0x05  -   Squall
 *
 */
public class OFS {

  private  final SPI spi;
  private  final byte[] dataReceived;
  private  final byte[] register = new byte[] {0};

  //Amount of change since last read
  private double raw_dx = 0;
  private double raw_dy = 0;

  //Convert for Decimal Values
  //Pixel to Feet conversion factor
  private double X_Left_Per_Ft  = -450;
  private double X_Right_Per_Ft = -450;
  private double Y_Fwd_Per_Ft   = -450;
  private double Y_Rev_Per_Ft   = -450;

  //Distance Covered
  private double xLinear = 0;
  private double yLinear = 0;

  public OFS() {
    spi = new SPI(Port.kOnboardCS0);    //Finds the OF on the SPI ports
    spi.setChipSelectActiveLow();
    spi.setClockActiveHigh();
    spi.setClockRate(500000);

    dataReceived = new byte[1];
    for(int i = 0; i < dataReceived.length; i++) {
      dataReceived[i] = 0;
    }
  }

  public void init() {
    readRegister((byte)2);

    xLinear = 0;
    yLinear = 0;

    raw_dx = 0;
    raw_dy = 0;
  }

  public void update() {
//    SmartDashboard.putNumber("Product ID", readRegister((byte) 0));
//    SmartDashboard.putNumber("Squall:", readRegister((byte) 5));

    int motionRegister = readRegister((byte) 2);
//    SmartDashboard.putNumber("Motion Register:", motionRegister);

    //Refresh raw values after register
    raw_dx = 0;
    raw_dy = 0;

    // Update the raw_dx/y var
    if ((motionRegister & 0x80) != 0) {
      raw_dx = readRegister((byte) 3);   //use registry to update the change in position
      raw_dy = readRegister((byte) 4);
    }

    //---------------------------------------------- Linear ------------------------------------------------------------
    
    //Find Actual Distance Covered
    if (raw_dx < 0) {
    	raw_dx = raw_dx / X_Left_Per_Ft;   //If the bot is going left, use left conversion factor
    } else {
    	raw_dx = raw_dx / X_Right_Per_Ft;  //If the bot is going right, use right conversion factor
    }
    if (raw_dy < 0) {
    	raw_dy = raw_dy / Y_Rev_Per_Ft;    //If the bot is going backwards, use backwards conversion factor
    } else {
    	raw_dy = raw_dy / Y_Fwd_Per_Ft;    //If the bot is going forward, use forward conversion factor
    }
    
    //Update total value
    xLinear += raw_dx;
    yLinear += raw_dy;

    //Ensure that im getting values
//    SmartDashboard.putNumber("Raw X", raw_dx);
//    SmartDashboard.putNumber("Raw Y", raw_dy);

    //Ensure that im getting values Linear
//    SmartDashboard.putNumber("Actual X (Linear)", xLinear);
//    SmartDashboard.putNumber("Actual Y (Linear)", yLinear);
  }

  private int readRegister(byte register) {
    this.register[0] = register;
    spi.write(this.register, 1); // Writes the register to be read
    spi.read(true, dataReceived, 1); // Reads the garbage
    spi.read(false, dataReceived, 1); // Reads the real register value
    return dataReceived[0];
  }

  public Vector2d getPos() {
	  return new Vector2d(getX(), getY());
  }
  
  public void setPos(Vector2d pos) {
	  this.xLinear = pos.getX();
	  this.yLinear = pos.getY();
  }
  
 public double getX() {
   return xLinear; // make dxLinear Negative to go in negative X TODO: verify you want to return dx
 }

  public double getY() {
    return yLinear; // TODO: verify you want to return dy
  }
}

