package org.teamresistance.frc.io;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * @author Shreya Ravi
 */
public class InvertibleDigitalInput {
  private final DigitalInput limitSwitch;
  private final boolean isInverted;

  public InvertibleDigitalInput(int channel, boolean isInverted) {
    this.isInverted = isInverted;
    limitSwitch = new DigitalInput(channel);
  }

  public boolean get() {
    return isInverted ? !limitSwitch.get() : limitSwitch.get();
  }
}
