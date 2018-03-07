package org.usfirst.frc.team2228.robot;

public class TeleopControllerCfg {
	//  defines minimum joystick moves to be acted on 
    public static double kJoyStickDeadBand = 0.1;
    
    // used the in the sine function for turning
    public static boolean isTurnSensitivityEnabled = true;
    public static boolean isLowSpeedFactorEnabled = true;
    public static final double kTurnSensitivityGain = 0.3; // 0.1 to 1.0 used for chezy turn control
	
     public enum TurnSensitivity {
		Linear,
		Sine,
		Squared,
		ThrottlePowerLimited
	}
    public enum ThrottleSensitivity {
		Linear,
		SCurve,
		Squared,
		Cubed
	}

	public static TurnSensitivity turnSensitivitySet = TurnSensitivity.Squared;
	public static ThrottleSensitivity throttleSensitivitySet = ThrottleSensitivity.Squared;
	
    // used to vary the affect when using cubed sensitivity
    public static double kThrottleCubedGain = 1.0; // 0 to 1; 0 is linear (output==input) 1 is cubed (output=input**3)
    
	// Range of smoothFactor is .5 to .9999; (no smoothing-0), (high smoothing-.99999)
	// factor =(1 - 1/#sampleTimes)) minSampleTimes-2

	public static double kLowSmoothFactor = 0.8;
	public static double kHighSmoothFactor = 0.97;
	public static double kTransitionSmoothFactor = 0.7;
	
	// determination of max delta values are determined by testing
	public static double kMaxDeltaVelocity = 0.2;
	public static double kTransitionMaxDelta = 0.1;

}
