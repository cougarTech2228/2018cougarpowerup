package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// 
public class TeleopController {
	
	private boolean isTeleopConsoleDataEnabled = false;
	
	private DriverIF DriverIF;
	private SRXDriveBase driveBase;

	private short loggerIterations = 0;
	private short loggerThreshold = 20;
	
	private int autoCmdSequence = 1;
	//private int timePeriodSF = TeleopControllerCfg.kHighSmoothPeriod;
	
	private double previousEMAValue = 0.0; // -1 to 1
	private static double test = 0;
	private double smoothFactor = 0;
	
	private boolean isButtonCmdActiveA = false;
	private boolean lastButtonReadA = false;
	private boolean isButtonCmdActiveB = false;
	private boolean lastButtonReadB = false;
	private boolean isButtonCmdActiveX = false;
	private boolean lastButtonReadX = false;
	private boolean isStopCheckToggleActive = false;
	
	private double maxThrottle = 1;
	private double maxTurn = .25;

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
		System.out.println("Started TeleopController");
		
	}
	public void teleopInit() {
		// clear drivebase and teleop flags

		driveBase.setSRXDriveBaseInit();
		//driveBase.DisplayChangeParmeters();
		// Clear button flags
		isButtonCmdActiveA = false;
		lastButtonReadA = false; 
		isButtonCmdActiveB = false;
		lastButtonReadB = false;
		isButtonCmdActiveX = false;
		lastButtonReadX = false;
	
	}
	public void teleopPeriodic() {
		double origThrottle = -DriverIF.Throttle();
		double origTurn = DriverIF.Turn();
//		if(SRXDriveBaseCfg.isTurnReversed == true){
//		origTurn = -DriverIF.Turn();
//		}
//		else{
//			origTurn = DriverIF.Turn();
//		}
//		if(SRXDriveBaseCfg.isThrottleReversed == true) {
//			origThrottle = -DriverIF.Throttle();
//		}
//		else {
//			origTurn = DriverIF.Throttle();
//		}
		
		double turn = origTurn;
		double throttle = origThrottle;

		
		// ========================================
		// SRXDriveBase test
		// ========================================		
		
		//	getButtonA();
		//	getButtonB();
		//	getButtonX();
		// driveBase.testMethodSelection();

		// ============================================
		// Teleop control
		// ============================================

		// Check and limit range of throttle and turn

	    throttle = limit(throttle);
		turn = limit(turn);
		
		if(turn != 0){
			turn = CheckTurnSensitivityFilter(limit(throttle), limit(turn));
		}
//		if (Math.abs(turn) < TeleopControllerCfg.kJoyStickDeadBand){
//			System.out.println("turn" + turn);
//			++notTurning;
//			if (notTurning == 20) // trying not to zero the yaw too early
//			{
//			  driveBase.setAngleZero();
//			}
//		} else {
//			notTurning = 0;
//		}
//		
		if(throttle != 0){
			throttle = CheckThrottleSensitivity(limit(throttle));
			throttle = CheckAccelFilter(limit(throttle));
			throttle = CheckDriverStopping(limit(throttle));
		}
		
		throttle *= maxThrottle;
		turn *= maxTurn;
		
		//CheckForAdjustSpeedRequest();
		//driveBase.UpdateSRXDriveDataDisplay();
		
		driveBase.setThrottleTurn(throttle, turn, false);
		if (isTeleopConsoleDataEnabled){
			System.out.printf("OrThottle: %-8.2f==Throttle: %-8.2f ==OrigTurn: %-8.2f ==Turn: %-8.2f%n", 
				origThrottle,
				throttle,
				origTurn,
				turn);
		}
	}
	public void SetMaxThrottlePower(double _maxThrottlePowerLevel) {
	maxThrottle = _maxThrottlePowerLevel;
	}
	
	public void SetMaxTurnPower(double _maxTurnPowerLevel) {
	maxTurn = _maxTurnPowerLevel;
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

			// If driverIF change is large enough to cause a wheelie or cause the
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
		if (_msgString != lastMsgString){
			System.out.println(_msgString);
			lastMsgString = _msgString;
		}
	}
//====================================================================
// SRXDriveBase test
// ===================================================================
	// button A used as a toggle button

//	private void getButtonA(){
//		if (!isButtonCmdActiveA){
//			if (DriverIF.RB_Button() && !lastButtonReadA) {
//				isButtonCmdActiveA = true;
//				driveBase.setRightSensorPositionToZero();
//				driveBase.setLeftSensorPositionToZero();
//				driveBase.setDriveTrainRamp(2);
//				Timer.delay(0.2);
//				msg("++Button A hit");
//			}
//		}
//		 else if (isButtonCmdActiveA) {
//				
//			//public boolean testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel)
//
//			//if(!driveBase.testDriveStraightCalibration(50.0, .3)){
//				
//			//velMoveToPosition(double _MoveToPositionIn, double _MoveToPositionPwrLevel, boolean _isCascadeMove) 
//
//			if (!driveBase.velMoveToPosition(30, .3, false)){
//				
//			//public boolean rotateToAngle(double _rotateToAngle, double _rotatePowerLevel)
//			//if (!driveBase.rotateToAngle(90, .2)){
//			
//			// public boolean testDriveStraightWithEncoderHeadingCal(double _testDistanceIn, double _pwrLevel){
//			//(deleted)if(!driveBase.testDriveStraightWithEncoderHeadingCal(50.0, .4)){
//			 
//			 //public boolean autoTuneCorrectionFactor(double _autoTunepowerLevel){
//			 //if (!driveBase.autoTuneCorrectionFactor(.2)){
//			 
//				
//			// turnByEncoderToAngle(double _turnAngleDeg, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn )
//			//if (!driveBase.turnByEncoderToAngle(90.0, 25, 0.1, false, false)) {
//				isButtonCmdActiveA = false;
//				msg("++Button A done");
//			}
//		lastButtonReadA = DriverIF.RB_Button(); 
//		}
//	}
//	private void getButtonB(){
//		if (DriverIF.cascadeBotton() && lastButtonReadB) {
//					isButtonCmdActiveB = true;	
//					autoCmdSequence = 1;
//		} else if (isButtonCmdActiveB) {
//			switch(autoCmdSequence){
//				case 1:
//					// move 10 inches
//					msg("case 1");
//					if (!driveBase.velMoveToPosition(10, 0.2, true)) {
//						autoCmdSequence = 2;
//					};
//					break;
//				case 2:
//					// turn right 90 deg
//					msg("case 2");
//					if(!driveBase.turnByEncoderToAngle(90, 25, .1, false, true )){
//						autoCmdSequence = 3;
//					};
//					break;
//				case 3:
//					// move 10 in
//					msg("case 3");
//					if (!driveBase.velMoveToPosition(10, 0.2, true)) {
//						autoCmdSequence = 4;
//					};
//					break;
//				case 4:
//					// turn left 90 deg
//					msg("case 4");
//					if(!driveBase.turnByEncoderToAngle(-70, 25, .1, false, false )){
//						isButtonCmdActiveB = false;	
//					};
//					break;	
//				default:
//					isButtonCmdActiveB = false;	
//			}	
//		}
//		lastButtonReadB = DriverIF.cascadeBotton();
//	}
//	private void getButtonX(){
//		if (!isButtonCmdActiveX){
//			if (DriverIF.Xbutton() && !lastButtonReadX) {
//				isButtonCmdActiveX = true;
//				driveBase.setRightSensorPositionToZero();
//				driveBase.setLeftSensorPositionToZero();	
//				Timer.delay(1);
//				msg("++Button C hit");
//		
//		}else if(DriverIF.Xbutton() && !lastButtonReadX){
//				isButtonCmdActiveX = false;
//				// Stop square wave
//				driveBase.testMotorPulse_SquareWave(0, 0, 5, true);
//				msg("++Button C done");
//			} else {
//				//public boolean testMotorPulse_SquareWave(double _pulseLowPower, double _pulseHighPower, double _pulseTimeSec, boolean _isTestForRightDrive)
//				driveBase.testMotorPulse_SquareWave(.2, .3, 5, true);
//			}
//		lastButtonReadX = DriverIF.Xbutton();
//		}
//	
//	}
}
