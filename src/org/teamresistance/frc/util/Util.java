package org.teamresistance.frc.util;

public class Util {

	public static double span(double input, double inLo, double inHi, double outLo, double outHi) {
		if (input < inLo) {
			return outLo;
		} else if (input > inHi) {
			return outHi;
		} else {
			return (outLo + (((input - inLo) / (inHi - inLo)) * (outHi - outLo)));
		}
	}

	public static double scaleInput(double input) {
		double[] scaleArray = { 0.0, 0.16, 0.18, 0.20, 0.24, 0.28, 0.32, 0.36,
                				0.40, 0.45, 0.50, 0.55, 0.62, 0.72, 0.85, 1.00, 1.00 };
        // get the corresponding index for the scaleInput array.
        int index = (int) (input * scaleArray.length);
        // index should be positive.
        if (index < 0) {
            index = -index;
        }
        // index cannot exceed size of array minus 1.
        if (index >= scaleArray.length) {
            index = scaleArray.length - 1;
        }
        // get value from the array.
        double scaled = 0.0;
        if (input < 0) {
            scaled = -scaleArray[index];
        } else {
            scaled = scaleArray[index];
        }
        // return scaled value.
        return scaled;
	}

	public static double clip(double input, double min, double max) {
		if (input <= min) {
			return min;
		} else if (input >= max) {
			return max;
		} else {
			return input;
		}
	}
}
