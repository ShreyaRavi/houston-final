package org.teamresistance.frc.io;


/**
 * @author Shreya Ravi
 */
public interface SingleSolenoid {
  void extend();
  void retract();
  boolean isExtended();
  boolean isRetracted();
}
