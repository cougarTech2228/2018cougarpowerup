package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// 
public class TeleopController {
	
	private boolean isTeleopConsoleDataEnabled = false;
	private boolean isStopCheckToggleActive = false;
	
	private DriverIF DriverIF;
	private SRXDriveBase driveBase;
	
	private double previousEMAValue = 0.0; // -1 to 1
	private double smoothFactor = 0;
	private double deltaThrottleForStopCheck =0;
	private double previousThrottleForStopCheck =0;
	private double stopAccum = 0;
	private double previousStopAccum = 0;
	private double EMAThrottleValue = 0;
	private double previousEMAThottleValue = 0;
	private double PowerCorrectionRatio = .3;
	
	
	private String lastMsgString = " ";

	public TeleopController(DriverIF _driverIF, SRXDriveBase _driveBase) {
		DriverIF = _driverIF;
		driveBase = _driveBase;
	}
	
	public void teleopPeriodic() {
		double origThrottle = -DriverIF.Throttle();
		double origTurn = DriverIF.Turn();

		double turn = origTurn;
		double throttle = origThrottle;

		// Check and limit range of throttle and turn
		throttle = limit(throttle);
		turn = limit(turn);
		
		if(turn != 0){
			turn = CheckTurnSensitivityFilter(limit(throttle), limit(turn));
		}
		if(throttle != 0){
			throttle = CheckThrottleSensitivity(limit(throttle));
			throttle = CheckAccelFilter(limit(throttle));
			throttle = CheckDriverStopping(limit(throttle));
		}
		
		//CheckForAdjustSpeedRequest();
		
		driveBase.setThrottleTurn(throttle, turn, false);
		if (isTeleopConsoleDataEnabled){
			System.out.printf("OrThottle: %-8.2f==Throttle: %-8.2f ==OrigTurn: %-8.2f ==Turn: %-8.2f%n", 
				origThrottle,
				throttle,
				origTurn,
				turn);
		}
	
	}	

// ===========================================
// DriverIF Filtering Functions
// ===========================================

	public double CheckTurnSensitivityFilter(double _throttle, double _turn) {
		
		double fThrottle = _throttle;
		double fTurn = _turn;
		
		switch (TeleopControllerCfg.turnSensitivitySet) {
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
			fTurn = (Math.signum(fTurn) * Math.pow(fTurn, 2));
			// Cap turn with respect to throttle power
			if(Math.abs(fTurn) > (Math.abs(fThrottle) * PowerCorrectionRatio)){
				fTurn = Math.signum(fTurn)*(Math.abs(fThrottle) * PowerCorrectionRatio);
			}
			break;
		default:
			break;
		}
		return fTurn;
	}
	
	public double ApplySineFunction(double _turn) {
		// kTurnSensitivityGain should be 0.1 to 1.0 used for chezy turn
		// control
		double factor = (Math.PI / 2.0) * TeleopControllerCfg.kTurnSensitivityGain;
		return Math.sin(factor * _turn) / Math.sin(factor);
	}
	
	public double CheckThrottleSensitivity(double _throttle) {

		// Sensitivity modifies the input value to provide a different feel of robot motion to the operator. 
		// There are several sensitivity curves that adjust the operator input for driving the robot. 
		// - Linear sensitivity curve: The action is linear for the operator, output == input
		// - Sine wave sensitivity: The sensitivity provides a larger do little around zero speed and near full speed. 
		//                          This the typical elevator curve. 
		// - Squared sensitivity curve: This sensitivity slows down the response of the robot to fast moves on the part of the operator. 
		// - Cubed sensitivity curve: This sensitivity really slows down the response of the robot.
		
		double fThrottle = _throttle;

		switch (TeleopControllerCfg.throttleSensitivitySet) {
		case Linear:
			// no change
			break;
		case SCurve:
			fThrottle = SCurve(_throttle);
			break;
		case Squared:
			fThrottle = Math.signum(_throttle) * Math.pow(_throttle, 2);
			break;
		case Cubed:
			fThrottle = (TeleopControllerCfg.kThrottleCubedGain * (Math.pow(_throttle, 3)))
					+ ((1 - TeleopControllerCfg.kThrottleCubedGain) * _throttle);
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
				limit(stopAccum);
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
	// driver exceeds the limit of robot accel/decel capability the tipping
	// filter slows the response of the throttle to protect the robot.
	
	// There are four changes in value from the last driverIF value:
	// 1)Transition from one side of zero to the other side of zero 
	// 2) The positive/negative side of zero 
	// 3) Within the driverIF deadband - reset filter
	// Determination of kMaxDeltaVel is determined by testing.
	
	public double CheckAccelFilter(double _ThrottleValue) {
		double AccelFltrCheckThrottleValue = _ThrottleValue;
		
		// determine change for last driverIF read
		double deltaAccelFltrThrottleValue = AccelFltrCheckThrottleValue - EMAThrottleValue;

		// Check driverIF _AccelFltrThrottleValue transition from one side of zero to the other side of zero
		if (Math.signum(AccelFltrCheckThrottleValue) != Math.signum(EMAThrottleValue)) {

			// If change is large enough to cause a wheelie or cause the
			// robot to start to tip - the robot intervenes to see that this does
			// not occur The following limits the change in driverIF movement
			if (Math.abs(deltaAccelFltrThrottleValue) > TeleopControllerCfg.kTransitionMaxDelta) {
				smoothFactor = TeleopControllerCfg.kTransitionSmoothFactor;
			} else {

				// If driver behaves
				smoothFactor = TeleopControllerCfg.kLowSmoothFactor;
			}
		}

		// Check for large same sign delta value that may cause a wheelie or rotation torque to a high Center of gravity
		else if (Math.abs(deltaAccelFltrThrottleValue) > TeleopControllerCfg.kMaxDeltaVelocity) {
				smoothFactor = TeleopControllerCfg.kHighSmoothFactor;
			} else {
				// If driver behaves
				smoothFactor = TeleopControllerCfg.kLowSmoothFactor;
			}
		

		// Check if the smoothing filter is within the driverIF dead band and put filter in high response gain
		if (Math.abs( AccelFltrCheckThrottleValue) < TeleopControllerCfg.kJoyStickDeadBand) {
			 AccelFltrCheckThrottleValue = 0; 
			smoothFactor = TeleopControllerCfg.kLowSmoothFactor;
		}
		// Run through smoothing filter
		
		// Exponential Avg Filter (EMA) is a recursive low pass filter that can change it's gain to address filter response
		// newAverage = alpha*presentValue + (1-alpha)*lastValue or:
		EMAThrottleValue = previousEMAValue + (1-smoothFactor) * (deltaAccelFltrThrottleValue);
		previousEMAThottleValue = EMAThrottleValue;
		msg("AccelFltrCheckThrottleValue" + AccelFltrCheckThrottleValue);
		return  AccelFltrCheckThrottleValue;
	}

	//helper function to keep inside of acceptable %power range
	protected static double limit(double num) {
		if(Math.abs(num) > 1){
			num = Math.signum(num)* 1.0;
		}
		if (Math.abs(num) < TeleopControllerCfg.kJoyStickDeadBand) {
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
}
