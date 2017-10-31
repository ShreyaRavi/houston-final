package org.teamresistance.frc.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import org.teamresistance.frc.util.joystick.Button;

import java.util.ArrayList;

/**
 * Created by Shreya on 2/20/2017.
 */
public class JoystickIO {
	// Joysticks
	public static Joystick leftJoystick = new Joystick(0);
	public static Joystick rightJoystick = new Joystick(1);
	public static Joystick coJoystick = new Joystick(2);

	// Buttons
	private static ArrayList<Button> buttons = new ArrayList<>();

	public static Button btnGyroReset = createButton(rightJoystick, 6);

	public static Button btnHoldLeft = createButton(rightJoystick, 4);
	public static Button btnHoldCenter = createButton(rightJoystick, 3);
	public static Button btnHoldRight = createButton(rightJoystick, 5);
	
	public static Button btnHoldLeftHopper = createButton(rightJoystick, 8);
	public static Button btnHoldRightHopper = createButton(rightJoystick, 9);
	
	public static Button btnPickupGear = createButton(coJoystick, 2);
	public static Button btnPlaceGear = createButton(coJoystick, 5);

	public static Button btnShooter = createButton(coJoystick, 1);
	public static Button btnAgitator = createButton(coJoystick, 3);
	public static Button btnClimber = createButton(coJoystick, 8);
	
	public static Button btnAuto = createButton(leftJoystick, 10);
	
	public static void update() {
		for (Button b : buttons) {
			b.update();
		}
	}

	private static Button createButton(GenericHID stick, int button) {
		Button newButton = new Button(stick, button);
		buttons.add(newButton);
		return newButton;
	}
}
