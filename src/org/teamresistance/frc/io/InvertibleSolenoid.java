package org.teamresistance.frc.io;

import edu.wpi.first.wpilibj.Solenoid;

public class InvertibleSolenoid implements SingleSolenoid {

  private final Solenoid solenoid;
  private final boolean isInverted;

  public InvertibleSolenoid(int module, int channel) {
    this(module, channel, false);
  }

  public InvertibleSolenoid(int module, int channel, boolean isInverted) {
    solenoid = new Solenoid(module, channel);
    this.isInverted = isInverted;
  }

  @Override
  public void extend() {
	  if (isInverted) {
		  solenoid.set(false);
	  } else {
		  solenoid.set(true);
	  }
    solenoid.set(!isInverted);
  }

  @Override
  public void retract() {
    solenoid.set(isInverted);
  }

  @Override
  public boolean isExtended() {
    return solenoid.get();
  }

  @Override
  public boolean isRetracted() {
    return !solenoid.get();
  }


}
