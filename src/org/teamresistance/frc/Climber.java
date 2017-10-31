package org.teamresistance.frc;

import org.teamresistance.frc.io.IO;
import org.teamresistance.frc.util.JoystickIO;
import org.teamresistance.frc.util.Time;

/**
 * Created by shrey on 2/20/2017.
 */
public class Climber {

  private double prevTime;
  private boolean climbed;

  public void init() {
    prevTime = Time.getTime();
    climbed = false;
  }

  public void update() {
    double current = IO.pdp.getCurrent(8);
    double timeDuration = 0.1;
    double spikeLimit = 70;
    double currTime = Time.getTime();

    if(JoystickIO.btnClimber.onButtonPressed()) {
    	climbed = false;
    }
    
    if (JoystickIO.btnClimber.isDown() && !climbed) {
      if (current >= spikeLimit) {
        if (currTime - prevTime >= timeDuration) {
          IO.climberMotor.set(0.0);
          climbed = true;
        } else {
          IO.climberMotor.set(1.0);
        }
      } else {
        prevTime = Time.getTime();
        IO.climberMotor.set(1.0);
      }
    } else {
      IO.climberMotor.set(0.0);
    }
  }


}
