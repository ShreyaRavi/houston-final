package org.teamresistance.frc.io;

public class InvertibleSolenoidWithPosition extends InvertibleSolenoid {

  private final InvertibleDigitalInput retractedLimit;

  public InvertibleSolenoidWithPosition(
      int module,
      int channel,
      boolean isSolenoidInverted,
      InvertibleDigitalInput retractedLimit) {
    super(module, channel, isSolenoidInverted);
    this.retractedLimit = retractedLimit;
  }

  @Override
  public boolean isRetracted() {
    return retractedLimit.get();
  }
}
