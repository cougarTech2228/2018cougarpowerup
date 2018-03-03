package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// 
public class TeleopController {
	
	//================================
	// OBJECTS
	
	private DriverIF DriverIF;
	private SRXDriveBase driveBase;
	
	//================================
	// SWITCHES
	
	private boolean isTeleopConsoleDataEnabled = false;
	private boolean isStopCheckToggleActive = false;
	
	 // used the in the sine function for turning
    public static boolean isTurnSensitivityEnabled = true;
    public static boolean isLowSpeedFactorEnabled = true;
	
	//=================================
	//CONSTANTS
	
	//  defines minimum joystick moves to be acted on 
    public static double kJoyStickDeadBand = 0.13;
	 // used to vary the affect when using cubed sensitivity
    public static double kThrottleCubedGain = 1.0; // 0 to 1; 0 is linear (output==input) 1 is cubed (output=input**3)
    
	// Range of smoothFactor is .5 to .9999; (no smoothing-0), (high smoothing-.99999)
	public static double kLowSmoothFactor = 0.5;
	public static double kHighSmoothFactor = 0.95;
	public static double kTransitionSmoothFactor = 0.7;
	
	// determination of max delta values are determined by testing
	public static double kMaxDeltaVelocity = 0.1;
	public static double kTransitionMaxDelta = 0.1;
	
    public static double kTurnSensitivityGain = 0.3; // 0.1 to 1.0 used for chezy turn control
	private static double powerCorrectionRatio = .35;
	
	//======================================
	// VARIABLES
	private double previousEMAValue = 0.0; // -1 to 1
	private double smoothFactor = 0;
	private double deltaThrottleForStopCheck =0;
	private double previousThrottleForStopCheck =0;
	private double stopAccum = 0;
	private double previousStopAccum = 0;
	private double EMAThrottleValue = 0;
	private double previousEMAThottleValue = 0;
	private double maxThrottle = 1;
	private boolean isTestBtnActive = false;
	private double CAL_thottle = 0;
	private double CAL_turn = 0;
	private double TST_Turn = 0;
	private double TST_Throttle = 0;
	
	
	private String lastMsgString = " ";
	
    //=========================
	// ENUMs
	
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
	public static TurnSensitivity turnSensitivitySet = TurnSensitivity.ThrottlePowerLimited;
	public static ThrottleSensitivity throttleSensitivitySet = ThrottleSensitivity.Squared;
	
   
	//==============================================
	// TELEOPCONTROLLER CONSTRUCTOR
	public TeleopController(DriverIF _driverIF, SRXDriveBase _driveBase) {
		DriverIF = _driverIF;
		driveBase = _driveBase;
	}
	//==========================================
	// TELEOP INIT
	public void teleopInit(){

		SmartDashboard.putBoolean("TstBtn-TurnSensitivity:", false);
		SmartDashboard.putBoolean("TstBtn-ThrottleSensitivity:", false);
		SmartDashboard.putBoolean("TstBtn-AccelFilter:", false);
		
		SmartDashboard.putNumber("CAL_thottle", CAL_thottle);
		SmartDashboard.putNumber("CAL_turn", CAL_turn);
		}
	//==========================================
	// TELEOP PERIODIC
	public void teleopPeriodic() {
		double origThrottle = -DriverIF.Throttle();
		double origTurn = DriverIF.Turn();

		double turn = origTurn;
		double throttle = origThrottle;

		// Check and cap range of throttle and turn
		throttle = cap(throttle);
		turn = cap(turn);
		
		if(turn != 0){
			turn = CheckTurnSensitivityFilter(throttle, turn);
		}
		if(throttle != 0){
			throttle = CheckThrottleSensitivity(throttle);
			throttle = CheckAccelFilter(throttle);
		//	throttle = CheckDriverStopping(throttle);
		}
		
		//CheckForAdjustSpeedRequest();
		
		throttle *= maxThrottle;
		// drive robot
		driveBase.setThrottleTurn(throttle, turn);
		
		if (isTeleopConsoleDataEnabled){
			System.out.printf("OrThottle: %-4.2f==Throttle: %-4.2f ==OrigTurn: %-4.2f ==Turn: %-4.2f ==EMA-TV: %-4.2f%n", 
								origThrottle,
								throttle,
								origTurn,
								turn,
								EMAThrottleValue);
		}
	}
	
	public void SetMaxThrottlePower(double _MaxSpeed) {
	maxThrottle = _MaxSpeed;
	}
// ===========================================
// DRIVERIF FILTERING FUNCTIONS

	public double CheckTurnSensitivityFilter(double _throttle, double _turn) {
		
		double fThrottle = _throttle;
		double fTurn = _turn;
		
		switch (turnSensitivitySet) {
		case Linear:
			// no change
			break;
		case Sine:
			//Used for chessy turn, developed by team 254 for the turn stick to provide a more realistic feel for turning
			fTurn = ApplySineFunction(fTurn);
			fTurn = ApplySineFunction(fTurn);
			fTurn = ApplySineFunction(fTurn);
			break;
		case Squared:
			fTurn = (Math.signum(fTurn) * Math.pow(fTurn, 2));
			break;
		case ThrottlePowerLimited:
			// square the turn
			//fTurn = fTurn * Math.abs(fTurn));
			// Cap turn with respect to throttle power
			//fTurn = fTurn * ((1-Math.abs(fThrottle)+.2) * powerCorrectionRatio);
			//fTurn = fTurn * (Math.abs(fThrottle - .3/ fThrottle + (.3) * powerCorrectionRatio));
			fTurn = fTurn * powerCorrectionRatio;
			break;
		default:
			break;
		}
		return fTurn;
	}
	
	public double ApplySineFunction(double _turn) {
		// kTurnSensitivityGain should be 0.1 to 1.0 used for chezy turn
		// control
		double factor = (Math.PI / 2.0) * kTurnSensitivityGain;
		return Math.sin(factor * _turn) / Math.sin(factor);
	}
	
	public double CheckThrottleSensitivity(double _throttle) {

		// Sensitivity modifies the input value to provide a different feel of robot motion to the operator. 
		// There are several sensitivity curves that adjust the operator input for driving the robot. 
		// - Linear sensitivity curve: 	The action is linear for the operator, output == input
		// - Sine wave sensitivity: 	The sensitivity provides a larger do little around zero speed and near full speed. 
		//                          	This is the typical elevator curve. 
		// - Squared sensitivity curve: This sensitivity slows down the response of the robot to fast moves on the part of the operator. 
		// - Cubed sensitivity curve: 	This sensitivity really slows down the response of the robot.
		
		double fThrottle = _throttle;

		switch (throttleSensitivitySet) {
		case Linear:
			// no change
			break;
		case SCurve:
			fThrottle = SCurve(_throttle);
			break;
		case Squared:
			fThrottle = _throttle * Math.abs(_throttle);
			break;
		case Cubed:
			// kThrottleCubedGain => from 0 to 1; 0 = linear, 1 = cubed
			fThrottle = (kThrottleCubedGain * (Math.pow(_throttle, 3)))	+ ((1 - kThrottleCubedGain) * _throttle);
			break;
		default:
			break;
		}
		return fThrottle;
	}
	
	public double SCurve(double _value) {
		
		double adjustedValue = _value;
		if (_value < 0) {
			adjustedValue = (2 * (-(Math.pow(_value, 3)))) - (3 * (Math.pow(_value, 2)));
		} else if (_value > 0) {
			adjustedValue = (3 * (Math.pow(_value, 2))) - (2 * (-(Math.pow(_value, 3))));
		}
		return adjustedValue;
	}
	
	public double CheckDriverStopping(double _throttleForStopCheck){
		// check every two scans
		if(isStopCheckToggleActive){
			deltaThrottleForStopCheck = _throttleForStopCheck - previousThrottleForStopCheck;
			previousThrottleForStopCheck = _throttleForStopCheck;
		}
		isStopCheckToggleActive = !isStopCheckToggleActive;
		
		// check if decelerating
		if((_throttleForStopCheck > 0) && (deltaThrottleForStopCheck < 0 ) ||
					(_throttleForStopCheck < 0) && (deltaThrottleForStopCheck > 0 )){
				
			// Decelerating
			if(Math.abs(_throttleForStopCheck) < 0.5) {
				_throttleForStopCheck = _throttleForStopCheck - Math.signum(_throttleForStopCheck)*stopAccum;
				// add delta to stopAccum
				stopAccum = previousStopAccum + Math.abs(deltaThrottleForStopCheck);
				cap(stopAccum);
				previousStopAccum = stopAccum;
			}
		} else {
			stopAccum = 0;
			previousStopAccum = 0;
		}
		
		return _throttleForStopCheck;
	}
	

	// AccelFilter
	// The accel filter follows the actions of
	// the driver with respect to the motion of the throttle driverIF. If the
	// driver exceeds the cap of robot accel/decel capability the tipping
	// filter slows the response of the throttle to protect the robot.
	
	// There are four changes in value from the last driverIF value:
	// 1)Transition from one side of zero to the other side of zero 
	// 2) The positive/negative side of zero 
	// 3) Within the driverIF deadband - reset filter
	// Determination of kMaxDeltaVel is determined by testing.
	
	
	public double CheckAccelFilter(double _ThrottleValue) {
		double AccelFltrCheckThrottleValue = _ThrottleValue;
		
		// determine change for last driverIF read
		double deltaAccelFltrThrottleValue = AccelFltrCheckThrottleValue - previousEMAThottleValue;

		// Check driverIF _AccelFltrThrottleValue transition from one side of zero to the other side of zero
		if (Math.signum(AccelFltrCheckThrottleValue) != Math.signum(EMAThrottleValue)) {

			// If change is large enough to cause a wheelie or cause the
			// robot to start to tip - the robot intervenes to see that this does
			// not occur The following caps the change in driverIF movement
			smoothFactor = kHighSmoothFactor;
//			if (Math.abs(deltaAccelFltrThrottleValue) > kTransitionMaxDelta) {
//				smoothFactor = kTransitionSmoothFactor;
//			} else {
//
//				// If driver behaves
//				smoothFactor = kLowSmoothFactor;
//			}
		}

		// Check for large same sign delta value that may cause a wheelie or rotation torque to a high Center of gravity
		else if (Math.abs(deltaAccelFltrThrottleValue) > kMaxDeltaVelocity) {
				smoothFactor = kHighSmoothFactor;
			} else {
				// If driver behaves
				smoothFactor = kLowSmoothFactor;
			}
		

		// Check if the smoothing filter is within the driverIF dead band and put filter in high response gain
		if (Math.abs( AccelFltrCheckThrottleValue) < kJoyStickDeadBand) {
			 AccelFltrCheckThrottleValue = 0; 
			smoothFactor = kLowSmoothFactor;
		}
		// Run through smoothing filter
		
		// Exponential Avg Filter (EMA) is a recursive low pass filter that can change it's gain to address filter response
		// newAverage = alpha*presentValue + (1-alpha)*lastValue or:
		EMAThrottleValue = previousEMAThottleValue + (1-smoothFactor) * (deltaAccelFltrThrottleValue);
		previousEMAThottleValue = EMAThrottleValue;
		msg("AFCTV" + AccelFltrCheckThrottleValue);
		return  EMAThrottleValue;
	}

	//helper function to keep inside of acceptable %power range
	protected static double cap(double num) {
		if(Math.abs(num) > 1){
			num = Math.signum(num)* 1.0;
		}
		if (Math.abs(num) < kJoyStickDeadBand) {
			num = 0;
		} 
		return num;
	}
	private void msg(String _msgString){
		if (isTeleopConsoleDataEnabled){
			if (_msgString != lastMsgString){
				System.out.println(_msgString);
				lastMsgString = _msgString;
			}
		}
	}
	
	/**
	* =======================================================================================
	* TEST METHODS
	* =======================================================================================
	*/	
	public void testMethodselection() {
		if(SmartDashboard.getBoolean("TstBtn-TurnSensitivity:", false)){
			
			// public double CheckTurnSensitivityFilter(double _throttle, double _turn) 
			System.out.println(CheckTurnSensitivityFilter(SmartDashboard.getNumber("CAL_thottle", CAL_thottle), SmartDashboard.getNumber("CAL_turn", CAL_turn)));			
		}
		if(SmartDashboard.getBoolean("TstBtn-ThrottleSensitivity:", false)){
			// public double CheckThrottleSensitivity(double _throttle)
			System.out.println(CheckThrottleSensitivity(SmartDashboard.getNumber("CAL_thottle", CAL_thottle))); 
		}
		if(SmartDashboard.getBoolean("TstBtn-AccelFilter:", false)){
			// public double CheckAccelFilter(double _ThrottleValue)
			System.out.println(CheckAccelFilter(SmartDashboard.getNumber("CAL_thottle", CAL_thottle))); 
		}	
		
	}
}