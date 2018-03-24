package org.usfirst.frc.team2228.robot;
/**
* Class SRXBaseDrive
* RELEASE: 2, RevA 180127 
* Team 2228
*/
/* ===================================
 * REVISIONS:
 * Release 2
 * RevA: original
 */

//Carrying over the classes from other libraries
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRXDriveBase {
	// The following is for the addition of a Navx and ultrasonic sensors
	// public class SRXDriveBase(AngleIF _angle, DistanceIF _distance)
	// AngleIF robotAngle = _angle;
	// DistanceIF robotDistance = _distance;
	
	
	// DifferentialDrive or tank motors
	private WPI_TalonSRX driveRightMasterMtr;
	private WPI_TalonSRX driveRightFollowerMtr;
	private WPI_TalonSRX driveLeftMasterMtr;
	private WPI_TalonSRX driveLeftFollowerMtr;

	private DifferentialDrive driveStyle;
	
	private int robotMode = 0;
	private int cycleCount = 1;
	private int SRXTimeoutValueMs = 10;
	private int correctionSensorType = 0;
	private int stepFunctionStopCount = 0;
	private int autoCmdSequence = 1;
	
	private int loggingDataIncrement = 1;
	
	private int mgmvRightCruiseVel = 0;
	private int mgmvRightAccel = 0;
	private int mgmvLeftCruiseVel = 0;
	private int mgmvLeftAccel = 0;
	
	private double mgmvRightDistance = 0;
	private double mgmvLeftDistance = 0;
	
	private double motionMagicLeftPos =0;
	private double motionMagicRightPos =0;
	private double motionMagicLeftVel = 0;
	private double motionMagicRightVel = 0;
	
	private double leftEncoderStopCount = 0;
	private double leftCmdLevel = 0;
	private double rightCmdLevel = 0;
	private double rotationEncoderStopCount = 0;
	private double driveStraightDirCorrection = 0;
	//private double speedRatio = 0;
	private double wheelToCenterDistanceIn = 0;
	private double outerDistanceStopCnt = 0;
	private double headingDeg =0;
	private double calCorrectionFactor = 0;
	private double moveStopCount = 0;
	private double rotateDistanceIn = 0;
	private double delayStartTime = 0;
	private double methodStartTime = 0;
	private double methodTime = 0;
	private double rightSensorStartPositionRead = 0;
	private double rightSensorPositionRead = 0;
	private double leftSensorStartPositionRead = 0;
	private double leftSensorPositionRead = 0;
	private double Kp_encoderHeadingPID = 0.001;
	private double Ki_encoderHeadingPID = 0;
	private double Kd_encoderHeadingPID = 0;
	private double kCapPIDCorrectionValue = 0.01;
	
	private double encoderPIDCorrection = 0;
	private double previousEncoderHeadingDeg = 0;
	private double encoderHeadingDeg = 0;
	private double EncoderHeadingRate = 0;
	private double sensorCorrection = 1;
	private double stepFunctionSpeed = 0;
	private double rightEncoderPosition = 0;
	private double rightSensorPositionValue = 0;
	private double leftEncoderPosition = 0;
	private double leftSensorPositionValue = 0;
	
	private double CAL_kDriveStraightFwdCorrection = SRXDriveBaseCfg.kDriveStraightFwdCorrection;
	private double CAL_kDriveStraightRevCorrection = SRXDriveBaseCfg.kDriveStraightRevCorrection;
	private double CAL_RightDriveCmdLevel = 0;
	private double CAL_LeftDriveCmdLevel = 0;
	private double CAL_Throttle = 0;
	private double CAL_turn = 0;
	private double magicMoveDistance = 0;
	private double methodAutoTime =0;
	private double moveStopTime = 0;
	
		
	//  Program flow switches
	private boolean isConsoleDataEnabled = true;
	private boolean isLoggingDataEnabled = false;
	private boolean islogSRXDriveActive = false;
	private boolean isVelMoveToPositionActive = false;
	private boolean isRotateToAngleActive = false;
	private boolean isTurnToAngleActive = false;
	private boolean isSRXMagicMoveActive = false;
	private boolean isSRXMoveMagicActive = false;
	private boolean isSRXRotateMagicActive = false;
	private boolean isSRXCalStdTrapezoidMoveMagicActive = false;
	private boolean isTestMoveForStraightCalActive = false;
	private boolean isCalAtStop = false;
	private boolean isDelayActive = false;
	private boolean isDriveTrainMoving = false;
	private boolean isSensorStopReached = false;
	private boolean isTestMethodSelectionActive = false;
	private boolean isSensorCorrectionActive = false;
	private boolean isTestStepFunctionActive = false;
	private boolean isTestBtnActive = false;
	private boolean isMtrTestBtnActive = false;
	private boolean isTimeVelMoveToPositionActive = false;
	private boolean isTimeRotateToAngleActive = false;
	private boolean isTimeTurnToAngleActive = false;
	private boolean isLoggingActive = false;
	private boolean isAutoTurnMoveDone = false;
	
	private String logSRXDriveString = " ";
	private String lastMsgString = " ";
	private String logString = " ";
	
	// SRXDriveBase Class Constructor
	public SRXDriveBase() {
	
		
		// Create CAN SRX motor controller objects
		driveRightMasterMtr = new WPI_TalonSRX(RobotMap.CAN_ID_1);
		driveRightFollowerMtr = new WPI_TalonSRX(RobotMap.CAN_ID_2);
		driveLeftMasterMtr = new WPI_TalonSRX(RobotMap.CAN_ID_3);
		driveLeftFollowerMtr = new WPI_TalonSRX(RobotMap.CAN_ID_4);

		// RIGHT MOTORS===========================================
		// =======================================================
		
		// Set min/max output
		driveRightMasterMtr.configNominalOutputForward(0.0, SRXTimeoutValueMs);
		driveRightMasterMtr.configNominalOutputReverse(0.0, SRXTimeoutValueMs);
		driveRightMasterMtr.configPeakOutputForward(1, SRXTimeoutValueMs);
		driveRightMasterMtr.configPeakOutputReverse(-1, SRXTimeoutValueMs);
		
		// Reverse motor if necessary
		driveRightMasterMtr.setInverted(SRXDriveBaseCfg.isDriveRightMasterMtrReversed);
		
		// Configure voltage compensation mode and set max voltage to 11 volts
		driveRightMasterMtr.configVoltageCompSaturation(11.0, SRXTimeoutValueMs);
		// tweak the voltage bus measurement filter, default is 32 cells in rolling average (1ms per sample)
		driveRightMasterMtr.configVoltageMeasurementFilter(32, SRXTimeoutValueMs);
		driveRightMasterMtr.enableVoltageCompensation(true);
		
		// set output zero (neutral) deadband at 4%
		driveRightMasterMtr.configNeutralDeadband(0.04, SRXTimeoutValueMs);

		// Set up stall conditions in SRX for the drive train
		driveRightMasterMtr.configPeakCurrentLimit(SRXDriveBaseCfg.kStallCurrentPeakAmps, SRXTimeoutValueMs);
		driveRightMasterMtr.configPeakCurrentDuration(SRXDriveBaseCfg.kStallTimeMs, SRXTimeoutValueMs);
		driveRightMasterMtr.configContinuousCurrentLimit(SRXDriveBaseCfg.kStallCurrentContinuousAmps, SRXTimeoutValueMs);
		driveRightMasterMtr.enableCurrentLimit(true);
		
		// Configure the velocity measurement period and sample window rolling average
		// Sample period in ms from supported sample periods-default 100ms period/64 sample window
		driveRightMasterMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms,SRXTimeoutValueMs);
		driveRightMasterMtr.configVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample,SRXTimeoutValueMs);
		
		// Set up encoder input
		driveRightMasterMtr.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, SRXDriveBaseCfg.kslotIDx, SRXTimeoutValueMs);
		driveRightMasterMtr.setSensorPhase(SRXDriveBaseCfg.isRightEncoderSensorReversed);
		
		// Clear quadrature position
		driveRightMasterMtr.clearStickyFaults(SRXTimeoutValueMs);
		driveRightMasterMtr.getSensorCollection().setQuadraturePosition(0, SRXTimeoutValueMs);
		
		
		// SET UP RIGHT FOLLOWER =======================
		// Invert SRX output to motors if necessary
		driveRightFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveRightFollowerMtrReversed);
		driveRightFollowerMtr.clearStickyFaults(SRXTimeoutValueMs);
		driveRightFollowerMtr.set(ControlMode.Follower, driveRightMasterMtr.getDeviceID());
		
		// LEFT MOTORS========================================
		//====================================================
		
		// Set min/max output
		driveLeftMasterMtr.configNominalOutputForward(0.0, SRXTimeoutValueMs);
		driveLeftMasterMtr.configNominalOutputReverse(0.0, SRXTimeoutValueMs);
	    driveLeftMasterMtr.configPeakOutputForward(1, SRXTimeoutValueMs);
		driveLeftMasterMtr.configPeakOutputReverse(-1, SRXTimeoutValueMs);
		
		// Reverse direction if necessary
		driveLeftMasterMtr.setInverted(SRXDriveBaseCfg.isDriveLeftMasterMtrReversed);
		
		// Configure voltage compensation mode and set max voltage to 11 volts
		driveLeftMasterMtr.configVoltageCompSaturation(11.0, SRXTimeoutValueMs);
		
		// tweak the voltage bus measurement filter, default is 32 cells in rolling average (1ms per sample)
		driveLeftMasterMtr.configVoltageMeasurementFilter(32, SRXTimeoutValueMs);
		driveLeftMasterMtr.enableVoltageCompensation(true);
		
		// set output zero (neutral) deadband at 4%
		driveLeftMasterMtr.configNeutralDeadband(0.04, SRXTimeoutValueMs);
		
		// Set up stall conditions in SRX for the drive train
		driveLeftMasterMtr.configPeakCurrentLimit(SRXDriveBaseCfg.kStallCurrentPeakAmps, SRXTimeoutValueMs);
		driveLeftMasterMtr.configPeakCurrentDuration(SRXDriveBaseCfg.kStallTimeMs, SRXTimeoutValueMs);
		driveLeftMasterMtr.configContinuousCurrentLimit(SRXDriveBaseCfg.kStallCurrentContinuousAmps, SRXTimeoutValueMs);
		driveLeftMasterMtr.enableCurrentLimit(true);
		
		// Configure the velocity measurement period and sample window rolling average
		// Sample period in ms from supported sample periods-default 100ms period/64 sample window
		driveLeftMasterMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms,SRXTimeoutValueMs);
		driveLeftMasterMtr.configVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample,SRXTimeoutValueMs);
		
		// Set up encoder input
		driveLeftMasterMtr.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, SRXDriveBaseCfg.kslotIDx, SRXTimeoutValueMs);
		driveLeftMasterMtr.setSensorPhase(SRXDriveBaseCfg.isLeftEncoderSensorReversed);
		
		// Clear quadrature position
		driveLeftMasterMtr.clearStickyFaults(SRXTimeoutValueMs);
		driveLeftMasterMtr.getSensorCollection().setQuadraturePosition(0, SRXTimeoutValueMs);
		
		
		// SET UP LEFT FOLLOWER =======================
		driveLeftFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveLeftFollowerMtrReversed);
		driveLeftFollowerMtr.clearStickyFaults(SRXTimeoutValueMs);
		driveLeftFollowerMtr.set(ControlMode.Follower, driveLeftMasterMtr.getDeviceID());
		
		// set all the control modes to zero control output
		driveRightMasterMtr.stopMotor();
		driveLeftMasterMtr.stopMotor();
//				driveRightMasterMtr.set(ControlMode.PercentOutput,0);
//				driveRightMasterMtr.set(ControlMode.Velocity, 0);
//				driveRightMasterMtr.set(ControlMode.MotionMagic, 0); 
//				
//				driveLeftMasterMtr.set(ControlMode.PercentOutput,0);
//				driveLeftMasterMtr.set(ControlMode.Velocity, 0);
//				driveLeftMasterMtr.set(ControlMode.MotionMagic, 0);		
		
		/*
		 * Set Brake-Coast mode to coast
		 */
		setBrakeMode(SRXDriveBaseCfg.isBrakeEnabled);

		/*
		*  Create drive for WPI arcade usage
		*/
		driveStyle = new DifferentialDrive(driveLeftMasterMtr, driveRightMasterMtr) ;
		driveStyle.setSafetyEnabled(false);
		
		// set timeout to zero to stop waiting for confirmations
		SRXTimeoutValueMs = 0;
	}
	// END OF CONSTRUCTOR
	
	// ======================================================================================
	// =======================================================================================
	// SRXBaseDrive SET/CONFIG METHODS
	// =======================================================================================
	// ======================================================================================
	public void setRightSensorPositionToZero() {
		//previousEMAAccelFltrThrottleValue;/ SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		driveRightMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx, 25);
	}
	
	public void setLeftSensorPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		driveLeftMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx, 25);
	}
	
	public void setRightEncPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		driveRightMasterMtr.getSensorCollection().setQuadraturePosition(0, 15);
	}
	
	public void setLeftEncPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		driveLeftMasterMtr.getSensorCollection().setQuadraturePosition(0, 25);
	}

	
	public void setCorrectionSensor(int _CorrectionSensorSelect){
		//0-none, 1-encoder, 2-Distance, 3-IMU
		correctionSensorType = _CorrectionSensorSelect;
	}
	
	public void setBrakeMode(boolean _isBrakeEnabled) {
		driveRightMasterMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		driveRightFollowerMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		driveLeftMasterMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		driveLeftFollowerMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
	}
	
	public void setStopMotors(){
		driveRightMasterMtr.stopMotor();
		driveLeftMasterMtr.stopMotor();

	}
	
	public void setEnableConsoleData(boolean _consoleData){
		isConsoleDataEnabled = _consoleData;
	}
	
	public void setSRXDriveBaseInit(int robotMode) {
		// Clear SRXDriveBase program control flags
		setInitialStateForSRXDrvBasePrgFlgs();
		
		// Load smart dashboard and shuffle board parameters
		loadSmartDashBoardParmeters();
		
		// Stop motors and clear position counters
		setStopMotors();
		setDriveTrainRamp(0);
		setRightSensorPositionToZero();
		setLeftSensorPositionToZero();
		
		// Load drive train PID values
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			driveRightMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			driveRightMasterMtr.configAllowableClosedloopError(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kClosedLoopErr, SRXTimeoutValueMs);
			driveRightMasterMtr.config_kF(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrFeedForwardGain, SRXTimeoutValueMs);
			if(robotMode == 2){
				driveRightMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, 0, SRXTimeoutValueMs);
				driveRightMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, 0, SRXTimeoutValueMs); 
				driveRightMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, 0, SRXTimeoutValueMs);	
			} else {
				driveRightMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrProportionalGain, SRXTimeoutValueMs);
				driveRightMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrIntegralGain, SRXTimeoutValueMs); 
				driveRightMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrDerivativeGain, SRXTimeoutValueMs);
				driveRightMasterMtr.config_IntegralZone(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrIzone, SRXTimeoutValueMs);
			}
			
			driveLeftMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			driveLeftMasterMtr.configAllowableClosedloopError(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kClosedLoopErr, SRXTimeoutValueMs);
			driveLeftMasterMtr.config_kF(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrFeedForwardGain, SRXTimeoutValueMs);
			if(robotMode == 2){
				driveLeftMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, 0, SRXTimeoutValueMs);
				driveLeftMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, 0, SRXTimeoutValueMs); 
				driveLeftMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, 0, SRXTimeoutValueMs);
			} else {
				driveLeftMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrProportionalGain, SRXTimeoutValueMs);
				driveLeftMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrIntegralGain, SRXTimeoutValueMs); 
				driveLeftMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrDerivativeGain, SRXTimeoutValueMs);
				driveLeftMasterMtr.config_IntegralZone(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveleftMstrIzone, SRXTimeoutValueMs);
			}
			
		}
		
		// See if console data display is enabled
		isConsoleDataEnabled = SmartDashboard.getBoolean("TstBtn-EnableSRXDriveBaseConsoleDisplay:", isConsoleDataEnabled);
	}
	
	// Clear all program control flags
	public void setInitialStateForSRXDrvBasePrgFlgs() {
		isVelMoveToPositionActive = false;
		isRotateToAngleActive = false;
		isTurnToAngleActive = false;
		isSRXMagicMoveActive = false;
		isSRXMoveMagicActive = false;
		isSRXRotateMagicActive = false;
		isSRXCalStdTrapezoidMoveMagicActive = false;
		isTestMoveForStraightCalActive = false;
		isCalAtStop = false;
		isDelayActive = false;
		isSensorStopReached = false;
		isTestMethodSelectionActive = false;
		isSensorCorrectionActive = false;
		isTestStepFunctionActive = false;
		isTestBtnActive = false;
		isMtrTestBtnActive = false;
		isTimeVelMoveToPositionActive = false;
		isTimeRotateToAngleActive = false;
		isTimeTurnToAngleActive = false;
		isLoggingActive = false;
		isAutoTurnMoveDone = false;
		
	}
	
	public void setDriveTrainRamp(double _SecToMaxPower){
		if(SRXDriveBaseCfg.isSRXClosedLoopEnabled){
			driveRightMasterMtr.configClosedloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
			driveLeftMasterMtr.configClosedloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
		} else {
			driveRightMasterMtr.configOpenloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
			driveLeftMasterMtr.configOpenloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
		}
	}
	
	// =======================================================================================
	// =======================================================================================
	// SRXBaseDrive GET METHODS
	// =======================================================================================
	//=======================================================================================
	
	// RIGHT MASTER MOTOR ==============
	
	public double getRightSensorPosition(){
		// This value is updated every 20ms
		if(SRXDriveBaseCfg.isRightEncoderSensorReadReversed){
			return -driveRightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		} else {
			return driveRightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		}
	}
	
	public double getRightSensorVelocity() {
		return driveRightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	public double getRightEncoderPosition() {
		// This value is updated every 160ms
		return -driveRightMasterMtr.getSensorCollection().getQuadraturePosition();
	}
	
	public double getRightEncoderVelocity(){
		// This value is updated every 160ms
		return driveRightMasterMtr.getSensorCollection().getQuadratureVelocity();
	}
	
	
	public double getRightMstrMtrCurrent() {
		return driveRightMasterMtr.getOutputCurrent();
	}

	public double getRightFollowerMtrCurrent() {
		return driveRightFollowerMtr.getOutputCurrent();
	}

	public double getRightCloseLoopError() {
		return driveRightMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	// LEFT MASTER MOTOR ==============================
	
	public double getLeftSensorPosition(){
		// This value is updated every 20ms
		if(SRXDriveBaseCfg.isLeftEncoderSensorReadReversed){
		return -driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		} else {
			return driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		}
	}
	
	
	public double getLeftSensorVelocity() {
		return driveLeftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	public double getLeftEncoderPosition() {
		// This value is updated every 160ms
		return driveLeftMasterMtr.getSensorCollection().getQuadraturePosition();
	}
	
	public double getLeftEncoderVelocity(){
		// This value is updated every 160ms
		return driveLeftMasterMtr.getSensorCollection().getQuadratureVelocity();
	}
	
	public double getLeftMstrMtrCurrent() {
		return driveLeftMasterMtr.getOutputCurrent();
	}
	
	public double getLeftFollowerMtrCurrent() {
		return driveLeftFollowerMtr.getOutputCurrent();
	}
	
	public double getLeftCloseLoopError() {
		return driveLeftMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx);
	}

	public double getBusVoltage() {
		return driveLeftMasterMtr.getBusVoltage();
	}
	
	// use "setCorrectionSensor(int _CorrectionSensorSelect)" or set in variable int correctionSensorType
	public double getDriveStraightCorrection(){
		//0-none, 1-encoder, 2-Distance, 3-IMU
		switch(correctionSensorType){
				case 0:
					sensorCorrection = 0;
					break;
				case 1:
					sensorCorrection = encoderAngleCorrection(isSensorCorrectionActive);
					break;
				case 2:
					// sensorCorrection = robotDistance.getAngleCorrection();
					break;
				case 3:
					// sensorCorrection = robotAngle.getAngleCorrection();
					break;
				default:
					sensorCorrection = 0;
		}	
		return	sensorCorrection;
	}
	
	public boolean isDriveMoving() {
		if ((Math.abs(driveLeftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx)) > 0.1) ||
			(Math.abs(driveRightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx)) > 0.1))
		{
			isDriveTrainMoving = true;
		} else {
			isDriveTrainMoving = false;
		}
		return isDriveTrainMoving;
	}
	
	// ======================================================================================
	// =======================================================================================
	// STATUS-DATA METHODS
	// =======================================================================================
	// ======================================================================================
	
	private void loadSmartDashBoardParmeters() {
		SmartDashboard.putBoolean("TstBtn-StepFnc:", false);
		SmartDashboard.putBoolean("TstBtn-DrvStraightCal:", false);
		SmartDashboard.putBoolean("TstBtn-DrvStrghtAutoCal:", false);
		SmartDashboard.putBoolean("TstBtn-VelMoveToPos:", false);
		SmartDashboard.putBoolean("TstBtn-RotateToAngle:", false);
		SmartDashboard.putBoolean("TstBtn-TurnToAngle:", false);
		SmartDashboard.putBoolean("TstBtn-CascadeTest:", false);
		SmartDashboard.putBoolean("TstBtn-MotorEncoderTest:", false);
		SmartDashboard.putBoolean("TstBtn-TimeVelMove:", false);
		SmartDashboard.putBoolean("TstBtn-TimeRotate:", false);
		SmartDashboard.putBoolean("TstBtn-TimeTurn:", false);
		SmartDashboard.putBoolean("TstBtn-MagicMotionMove:", false);
		SmartDashboard.putBoolean("TstBtn-MagicMotionRotate:", false);
		SmartDashboard.putBoolean("TstBtn-MagicMotionBase:", false);
		
		SmartDashboard.putBoolean("TstBtn-EnableSRXDriveBaseConsoleDisplay:", isConsoleDataEnabled);
		
		SmartDashboard.putNumber("Left Encoder:", leftSensorPositionRead);
		SmartDashboard.putNumber("Right Encoder:", rightSensorPositionRead);
		
		SmartDashboard.putNumber("Kp_encoderHeadingPID:", Kp_encoderHeadingPID);
		SmartDashboard.putNumber("Ki_encoderHeadingPID:", Ki_encoderHeadingPID);
		SmartDashboard.putNumber("Kd_encoderHeadingPID:", Kd_encoderHeadingPID);
		
		SmartDashboard.putNumber("CAL_kDriveStraightFwdCorrection:", CAL_kDriveStraightFwdCorrection);
		SmartDashboard.putNumber("CAL_kDriveStraightRevCorrection:", CAL_kDriveStraightRevCorrection);
		
		SmartDashboard.putNumber("CAL_RightDriveCmdLevel:", CAL_RightDriveCmdLevel);
		SmartDashboard.putNumber("CAL_LeftDriveCmdLevel:", CAL_LeftDriveCmdLevel);
		
		SmartDashboard.putNumber("CAL_Throttle:", CAL_Throttle);
		SmartDashboard.putNumber("CAL_turn:", CAL_turn);
		
		SmartDashboard.putNumber("Val_encoderHeadingDeg:", encoderHeadingDeg);
		SmartDashboard.putNumber("Val_encoderPIDCorrection:", encoderPIDCorrection);
	}
	
	// Reads encoder, velocity, current, error, and displays on smartdashboard
	public void updateSRXDriveDataDisplay() {

		// Display SRX module values
		SmartDashboard.putNumber("BaseDrive-Right Bus Voltage", driveRightMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("BaseDrive-Right Output Voltage", driveRightMasterMtr.getMotorOutputVoltage());
		SmartDashboard.putNumber("BaseDrive- Right Master Current", driveRightMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive- Right Follower Current", driveRightFollowerMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive-Left Bus Voltage", driveLeftMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("BaseDrive-Left Output Voltage", driveLeftMasterMtr.getMotorOutputVoltage());
		SmartDashboard.putNumber("BaseDrive- Left Master Current", driveLeftMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive- Left Follower Current", driveRightFollowerMtr.getOutputCurrent());

		if (SRXDriveBaseCfg.isMasterEncodersPresent) {
			SmartDashboard.putNumber("BaseDrive-Right Position", driveRightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Right Velocity ", driveRightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Left Position", driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Left Velocity ", driveLeftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx));
		}

		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			SmartDashboard.putNumber("BaseDrive-Speed Right ClosedLoopErr",	driveRightMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Speed Left ClosedLoopErr", driveLeftMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx));
		}

	}

	public void logSRXDriveData(){
		if (!islogSRXDriveActive){
			islogSRXDriveActive = true;
			logSRXDriveString = ",Right Bus Voltage,Right Output Voltage,Right Master Current,Right Encoder Count,Right Follower Current,Left Bus Voltage,Left Output Voltage,Left Master Current,Left Encoder Count,Left Follower Current";
			// Log data
			DebugLogger.data(logSRXDriveString);
		} else {
			logSRXDriveString = String.format(",%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f", 
									driveRightMasterMtr.getBusVoltage(), 
									driveRightMasterMtr.getMotorOutputVoltage(),
									driveRightMasterMtr.getOutputCurrent(),
									getRightSensorVelocity(),
									driveRightFollowerMtr.getOutputCurrent(),
									driveLeftMasterMtr.getBusVoltage(),
									driveLeftMasterMtr.getMotorOutputVoltage(),
									driveLeftMasterMtr.getOutputCurrent(),
									getLeftSensorVelocity(),
									driveLeftFollowerMtr.getOutputCurrent());
			// Log data
			DebugLogger.data(logSRXDriveString);
		}
	} 
	
	private void msg(String _msgString){
		if (_msgString != lastMsgString){
			System.out.println(_msgString);
			lastMsgString = _msgString;}
		}
		
	// =======================================================================================
	// =======================================================================================
	// TELEOP METHODS
	// =======================================================================================
	// =======================================================================================
	
	
	// Note: left drive is master drive axis for the robot - the right drive
	// will be modified for driving straight
	//
	// NOTE: Motion command with open loop reflect power levels (-1 to 1) * (the motor bus voltage). 
	// Motion command with closed loop reflect speed level => (-1 to 1) * (top motor RPM)
	//
	
	public void SetDriveTrainCmdLevel(double _rightCMDLevel, double _leftCMDLevel) {
		rightCmdLevel = _rightCMDLevel;
		leftCmdLevel = _leftCMDLevel;
		
		driveRightFollowerMtr.set(ControlMode.Follower, driveRightMasterMtr.getDeviceID());
		driveLeftFollowerMtr.set(ControlMode.Follower, driveLeftMasterMtr.getDeviceID());
		
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			// Output commands to SRX modules set as [% from (-1 to 1)] x MaxVel_VelNativeUnits
			driveRightMasterMtr.set(ControlMode.Velocity, (rightCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
			driveLeftMasterMtr.set(ControlMode.Velocity, (leftCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
		} else {
			driveRightMasterMtr.set(ControlMode.PercentOutput,rightCmdLevel);
			driveLeftMasterMtr.set(ControlMode.PercentOutput,leftCmdLevel);
		}
	}

	//
	//  WPI throttle and turn commands This method uses WPI library methods to
	//  drive the robot with a throttle and turn input. Drives were set up by:
	//  driveStyle = new RobotDrive(driveRightMasterMtr, driveLeftMasterMtr); The
	//  throttle would be the game controller Y-axis(joystick fwd/rev) and turn
	//  would be game controller X-axis(joystick left/right)
	// 
	//  NOTE: WPILib throttleValue and turnValue are open loop power levels (-1
	//  to 1) * (the motor bus voltage). The speed is determined by this power
	//  level and the load to the motor.
	// 
	 //======================== THIS HAS NOT BEEN TESTED ==========================
	public void WPISetThrottleTurn(double _WPIThrottleValue, double _WPITurnValue) {
		// parms: move, turn, squared inputs
		driveStyle.arcadeDrive(_WPIThrottleValue, _WPITurnValue, false);	
	}

	// ======================================
	// SET THROTTLE-TURN
	// ======================================
	//setthrottleturn is both open loop and closed loop control with drive straight correction
	
	public void setThrottleTurn(double _throttleValue, double _turnValue) {
			// Calculate cmd level in terms of PercentVbus; range (-1 to 1)
			leftCmdLevel = _throttleValue + (_turnValue/2);
			rightCmdLevel = ((_throttleValue* SRXDriveBaseCfg.kDriveStraightFwdCorrection) - (_turnValue/2));
			
			
		// Determine drive straight correction if enabled
		if (SRXDriveBaseCfg.isDriveStraightAssistEnabled && Math.abs(_turnValue) < SRXDriveBaseCfg.kSpeedDeadBand){
			
			isSensorCorrectionActive = true;
			driveStraightDirCorrection = getDriveStraightCorrection();
			rightCmdLevel += driveStraightDirCorrection;
		} else{
			isSensorCorrectionActive = false;
			driveStraightDirCorrection = getDriveStraightCorrection();
		}
		
		// Output commands to SRX modules set as [% from (-1 to 1)] x MaxVel_VelNativeUnits for feedback control
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		//++++++++++++++++++++++++++++++++++++++
		// Display data
		if (isConsoleDataEnabled){
			System.out.printf("LftCmd:%-4.3f=RgtCmd:%-4.3f=LftVel:%-5.2f=RgtVel:%-5.2f=LftCur:%-5.2f=RgtCur:%-5.2f=Hd:%-4.3f=Cor:%-4.3f%n", 
									leftCmdLevel, 
									rightCmdLevel,
									getLeftSensorVelocity(),
									getRightSensorVelocity(),
									getLeftMstrMtrCurrent(),
									getRightMstrMtrCurrent(),
									encoderHeadingDeg,
									driveStraightDirCorrection);
		}
	}
	
	// ======================================================================================
	// =======================================================================================
	// AUTONOMOUS METHODS
	// =======================================================================================
	// ======================================================================================
	
	//==============================================
	// VELOCITY MOVE TO POSITION
	//==============================================
	
	// This method moves the robot with a predetermined power level and stops at
	// the specified position value. The move will be in brake mode to stop
	// method will check that robot is stopped and set brake mode back to coast and respond
	// that move is done
	public boolean velMoveToPosition(double _moveToPositionIn, double _moveToPositionPwrLevel, boolean _isDirectionRev, boolean _isCascadeMove) {
		if(!SRXDriveBaseCfg.isMotionMagicEnabled){
			// Read encoders
			leftSensorPositionRead = getLeftSensorPosition();
			rightSensorPositionRead = getRightSensorPosition();
			
			if (!isVelMoveToPositionActive) {
				isVelMoveToPositionActive = true;
				methodStartTime = Timer.getFPGATimestamp();
				msg("START VELOCITY MOVE===========================");
		
				// Move calculations
				// Check if move is zero and set to a long distance (inches)
				if(Math.abs(_moveToPositionIn) == 0){
					//Note short hand "if" (booleanCondition ? executeThisPartIfBooleanConditionIsTrue : executeThisPartIfBooleanConditionIsFalse) 
					_moveToPositionIn = ((_isDirectionRev)?  -1000 : 1000);
				}
				// Move from where the sensor position is
				moveStopCount = leftSensorPositionRead + ((_isDirectionRev)? -_moveToPositionIn : _moveToPositionIn) * SRXDriveBaseCfg.kLeftEncoderCountsPerIn;
				
				if(!_isCascadeMove){
					moveStopCount = moveStopCount + ((_isDirectionRev)? SRXDriveBaseCfg.kAutoMoveCoastToStopCounts : -SRXDriveBaseCfg.kAutoMoveCoastToStopCounts);
				}
				// Direction is determined by sign of _MoveToPositionIn				
				leftCmdLevel = ((_isDirectionRev)? -_moveToPositionPwrLevel : _moveToPositionPwrLevel);
				if(_isDirectionRev){
					rightCmdLevel = -_moveToPositionPwrLevel * SRXDriveBaseCfg.kDriveStraightFwdCorrection;
				} else {
					rightCmdLevel = _moveToPositionPwrLevel * SRXDriveBaseCfg.kDriveStraightRevCorrection;
				}
				
			} else {
				// Determine drive straight correction if enabled
				if (SRXDriveBaseCfg.isDriveStraightAssistEnabled && isVelMoveToPositionActive) {
							
					isSensorCorrectionActive = true;
					driveStraightDirCorrection = getDriveStraightCorrection();
					rightCmdLevel = rightCmdLevel + ((_isDirectionRev)? -driveStraightDirCorrection : driveStraightDirCorrection) ;
				} else{
					// Reset Sensor Correction Active
					isSensorCorrectionActive = false;
					driveStraightDirCorrection = getDriveStraightCorrection();
				}
				
				// Check for sensor stop
				if(_moveToPositionIn == 0){
					//isSensorStopReached = robotDistance.getCheckSensorStop();
				} else {
					isSensorStopReached = false;
				}
				
				if((isSensorStopReached) 
					|| ((_isDirectionRev)? (leftSensorPositionRead <= moveStopCount) : (leftSensorPositionRead >= moveStopCount))){
				
					msg("AT VELOCITY MOVE STOP==================================");
					if (_isCascadeMove) {
						msg("VEL MOVE TO POSITION DONE IN CASCADE MODE================");
						isVelMoveToPositionActive = false;
						// Reset Sensor Correction Active
						isSensorCorrectionActive = false;
						driveStraightDirCorrection = getDriveStraightCorrection();
					} else {
							// No ramp at end of move
							setDriveTrainRamp(0);
							
							// Apply power level in opposite direction to brake to reduce coast
							rightCmdLevel = ((_isDirectionRev)? SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue : -SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue);
							leftCmdLevel = ((_isDirectionRev)? SRXDriveBaseCfg.kAutoLeftMoveStopBrakeValue : -SRXDriveBaseCfg.kAutoLeftMoveStopBrakeValue);
							
						// Delay in sec
						if (!delay(1)) {
							isVelMoveToPositionActive = false;
							// Reset Sensor Correction Active
							isSensorCorrectionActive = false;
							driveStraightDirCorrection = getDriveStraightCorrection();
							//Stop motors
							rightCmdLevel = 0;
							leftCmdLevel = 0;
							methodTime = Timer.getFPGATimestamp() - methodStartTime;
							msg("Vel move to position Time(Sec) = " + methodTime);
							msg("VELOCITY MOVE COMPLETE=======================");
						}
					}
				}
				
			}
			
			SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
			
			//+++++++++++++++++++++++
			// LOGGING AND DISPLAY
			if (isConsoleDataEnabled){
				System.out.printf("StopCnt:%-8.0f ==LftPos:%-8.2f ==RgtPos:%-8.2f ==Angle:%-8.3f ==Correct:%-8.3f%n",
										moveStopCount,
										leftSensorPositionRead, 
										rightSensorPositionRead,
										encoderHeadingDeg,
										driveStraightDirCorrection);		
			}
		
		// Run magic move method
		} else {
			if (!isVelMoveToPositionActive) {
			isVelMoveToPositionActive = true;
			methodStartTime = Timer.getFPGATimestamp();
			msg("START VELOCITY MOTION MAGIC MOVE===========================");
			magicMoveDistance = ((_isDirectionRev)? -_moveToPositionIn : _moveToPositionIn);
			
			} else if(!SRXMoveMagic(magicMoveDistance, _moveToPositionPwrLevel)){
				msg("AT VELOCITY MOTION MAGIC MOVE STOP==================================");
				isVelMoveToPositionActive = false;
				methodTime = Timer.getFPGATimestamp() - methodStartTime;
				msg("Vel move to position Time(Sec) = " + methodTime);
				msg("VELOCITY MOTION MAGIC MOVE COMPLETE=======================");
			}
			
		}
		return isVelMoveToPositionActive;
	}

	//====================================
	// ROTATE TO ANGLE
	//====================================
	public boolean rotateToAngle(double _rotateToAngle, double _rotatePowerLevel) {
		if(!SRXDriveBaseCfg.isMotionMagicEnabled){
			leftSensorPositionRead = getLeftSensorPosition();
			rightSensorPositionRead = getRightSensorPosition();
			
			if (!isRotateToAngleActive) {
				isRotateToAngleActive = true;
				methodStartTime = Timer.getFPGATimestamp();
				msg("ROTATE TO ANGLE IS ACTIVE===========================");
				
				leftCmdLevel = Math.signum(_rotateToAngle) * _rotatePowerLevel;
				if(_rotateToAngle >= 0){
					rightCmdLevel = -_rotatePowerLevel * SRXDriveBaseCfg.kRotateCCWDriveStraightCorrection;
				} else{
					rightCmdLevel = _rotatePowerLevel * SRXDriveBaseCfg.kRotateCWDriveStraightCorrection;
				}
				
				// rotationEncoderStopCount = C(=>PI*D) * (angle as a fraction of C)
				rotationEncoderStopCount = (Math.PI*(SRXDriveBaseCfg.kTrackWidthIn) * SRXDriveBaseCfg.kEncoderCountsPerIn * (Math.abs(_rotateToAngle) / 360))
				                                 - SRXDriveBaseCfg.kAutoRotateCoastToStopCounts;
				
				// use left encoder to mark rotation distance
			} else if ((Math.abs(leftSensorPositionRead) >= rotationEncoderStopCount) || (Math.abs(rightSensorPositionRead) >= rotationEncoderStopCount)){
					msg("ROTATE TO ANGLE IS AT STOP===========================");
					
					setDriveTrainRamp(0);
					// Apply power level in opposite direction to brake
					rightCmdLevel = (Math.signum(_rotateToAngle)*SRXDriveBaseCfg.kAutoRightRotateStopBrakeValue);
					leftCmdLevel = -(Math.signum(_rotateToAngle)*SRXDriveBaseCfg.kAutoLeftRotateStopBrakeValue);
				if (!delay(1)) {
					isRotateToAngleActive = false;
					rightCmdLevel = 0;
					leftCmdLevel = 0;
					methodTime = Timer.getFPGATimestamp() - methodStartTime;
					msg("Rotate to angle Time(Sec) = " + methodTime);
					msg("ROTATE TO ANGLE IS DONE=========================");
				}		
			}
			
			SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
			
			//++++++++++++++++++++++++++++++++++++++++++++++
			// Display data
			if (isConsoleDataEnabled){
				System.out.printf("StopCnt:%-8.2f ===LftPos:%-8.2f ===RgtPos:%-8.2f%n",
										rotationEncoderStopCount,
										leftSensorPositionRead, 
										rightSensorPositionRead);
			}
			
		// Run magic rotate method
		} else {
			if (!isRotateToAngleActive) {
			isRotateToAngleActive = true;
			methodStartTime = Timer.getFPGATimestamp();
			msg("ROTATE TO ANGLE MOTION MAGIC IS ACTIVE===========================");
			
			} else if (!SRXRotateMagic(_rotateToAngle,  _rotatePowerLevel)) {
				msg("ROTATE TO ANGLE IS AT MOTION MAGIC STOP===========================");
				isRotateToAngleActive = false;
				methodTime = Timer.getFPGATimestamp() - methodStartTime;
				msg("Rotate to angle Time(Sec) = " + methodTime);
				msg("ROTATE TO ANGLE IS MAGIC MOTION DONE=========================");
			}		
		}
		return isRotateToAngleActive;
	} 
	//===================================
	// TURN BY ENCODER TO ANGLE
	//===================================
	public boolean turnByEncoderToAngle(double _turnAngleDeg, double _turnRadiusIn, double _turnPowerLevel, boolean _isTurnDirectionRev, boolean _isCascadeTurn ) {
		// read encoders
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		
		if (!isTurnToAngleActive) {
			isTurnToAngleActive = true;
			isAutoTurnMoveDone = false;
			methodStartTime = Timer.getFPGATimestamp();
			msg("TURN TO ANGLE ACTIVE===========================");
			
			// Check turn radius add 4 inches is less than 1/2 wheelbase
			if(_turnRadiusIn < wheelToCenterDistanceIn){
				_turnRadiusIn = wheelToCenterDistanceIn + 4;
			}
			
			//radius is the distance from the center of the robot to a point outside the robot
			//speedRatio =(_turnRadiusIn + wheelToCenterDistanceIn) / (_turnRadiusIn - wheelToCenterDistanceIn);
			
			// Determine which wheel has to speed up to turn
			// Turn right
			if (_turnAngleDeg > 0) {
				if(_isTurnDirectionRev){
					rightCmdLevel = -_turnPowerLevel * (1 + (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn)));
					leftCmdLevel = -_turnPowerLevel * (1 - (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn)));
				} else {
					rightCmdLevel = _turnPowerLevel * (1 - (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn)));
					leftCmdLevel = _turnPowerLevel * (1 + (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn))) * SRXDriveBaseCfg.kTurnRightDriveStraightCorrection;
				}
				
			// Turn Left
			} else {
				if(_isTurnDirectionRev){
					rightCmdLevel = -_turnPowerLevel * (1 - (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn)));
					leftCmdLevel = -_turnPowerLevel * (1 + (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn)));
				} else {
					rightCmdLevel = _turnPowerLevel * (1 + (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn))) * SRXDriveBaseCfg.kTurnLeftDriveStraightCorrection;
					leftCmdLevel = _turnPowerLevel * (1 - (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn)));
				}
				
			}
			
			// Calculations
			wheelToCenterDistanceIn = (SRXDriveBaseCfg.kTrackWidthIn / 2);
			
			// Convert turn distance in inches to encoder counts(2*PI*Radius)*(turnAngledeg/360deg)*(cnts/in)
			// Turn right
			if (_turnAngleDeg >= 0) {
				if(_isTurnDirectionRev){
					outerDistanceStopCnt = rightSensorPositionRead - (2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) * SRXDriveBaseCfg.kRightEncoderCountsPerIn);
				} else {
					outerDistanceStopCnt = leftSensorPositionRead + (2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) *	SRXDriveBaseCfg.kLeftEncoderCountsPerIn);
				}
			// Turn Left	
			} else {
				if(_isTurnDirectionRev){
					outerDistanceStopCnt = leftSensorPositionRead - (2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) *	SRXDriveBaseCfg.kLeftEncoderCountsPerIn);
				} else {
					outerDistanceStopCnt = rightSensorPositionRead + (2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) * SRXDriveBaseCfg.kRightEncoderCountsPerIn);
				}
				
			}
			if(!_isCascadeTurn){
					outerDistanceStopCnt = outerDistanceStopCnt + ((_isTurnDirectionRev)? -SRXDriveBaseCfg.kAutoTurnCoastToStopCounts : SRXDriveBaseCfg.kAutoTurnCoastToStopCounts);
				}
		// Active state -  check for end of encoder count
		} else if(_turnAngleDeg >= 0){
				if(_isTurnDirectionRev){
					if(rightSensorPositionRead <= outerDistanceStopCnt) {
						isAutoTurnMoveDone = true;
					}
				} else {
					if (leftSensorPositionRead >= outerDistanceStopCnt){
						isAutoTurnMoveDone = true;
					}
				}
			
			} else {
				if(_isTurnDirectionRev){
					if (leftSensorPositionRead <= outerDistanceStopCnt){
						isAutoTurnMoveDone = true;
					}
				} else {
					if(rightSensorPositionRead >+ outerDistanceStopCnt) {
						isAutoTurnMoveDone = true;
					}
				}
			}
		if(isAutoTurnMoveDone){
			msg("TURN TO ANGLE IS AT STOP===========================");
			if (_isCascadeTurn) {
				isTurnToAngleActive = false;
				
			} else {
				setDriveTrainRamp(0);	
				// Apply power level in opposite direction for 1 second to brake
				rightCmdLevel = ((_isTurnDirectionRev)? SRXDriveBaseCfg.kAutoRightTurnStopBrakeValue : -SRXDriveBaseCfg.kAutoRightTurnStopBrakeValue);
				leftCmdLevel = ((_isTurnDirectionRev)? SRXDriveBaseCfg.kAutoLeftTurnStopBrakeValue : -SRXDriveBaseCfg.kAutoLeftTurnStopBrakeValue);
				
				if (!delay(1)) {
					isTurnToAngleActive = false;
					rightCmdLevel = 0;
					leftCmdLevel = 0;
					methodTime = Timer.getFPGATimestamp() - methodStartTime;
					msg("Turn to angle Time(Sec) = " + methodTime);
					msg("TURN TO ANGLE DONE===========================");
				}
			}	
		}
		
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		// +++++++++++++++++++++++++++++++++++++++
		// Display data
		if (isConsoleDataEnabled){
			System.out.printf("RgtCmd: %-4.3f ===LftCmd: %-4.3f ===StopCnt: %-8.0f ===LftPos: %-8.2f ===RgtPos: %-8.2f%n",
									rightCmdLevel,
									leftCmdLevel,
									outerDistanceStopCnt,
									leftSensorPositionRead, 
									rightSensorPositionRead);
		}
		return isTurnToAngleActive;
	}
	

	//==========================================
	// ENCODER HEADING PID CORRECTION
	//==========================================
	//
	// the rightCmdLevel is a global variable
	public double encoderAngleCorrection(boolean _isPIDActive){
		if(_isPIDActive){
			
			
			// Determine present heading
			encoderHeadingDeg = (leftSensorPositionRead - rightSensorPositionRead) / SRXDriveBaseCfg.kTrackWidthIn;
			
			//headingRate = ((currentHeading - previousHeading) / timeElapsed[this is taken care of in Kd];
			EncoderHeadingRate = encoderHeadingDeg - previousEncoderHeadingDeg;
			
			//headingError = heading set point(=0) - currentHeading, however we need to increase right drive to go left;
			encoderPIDCorrection = Kp_encoderHeadingPID * encoderHeadingDeg - (Kd_encoderHeadingPID * EncoderHeadingRate);
			// Cap encoderPIDCorrection output
			if(encoderPIDCorrection > kCapPIDCorrectionValue){
				encoderPIDCorrection = kCapPIDCorrectionValue;
			}
			if(encoderPIDCorrection < -kCapPIDCorrectionValue){
				encoderPIDCorrection = -kCapPIDCorrectionValue;
			}
		} else {
			encoderHeadingDeg = 0;
			previousEncoderHeadingDeg = 0;
			encoderPIDCorrection = 0;
		}
		if (isConsoleDataEnabled){
			SmartDashboard.putNumber("Val_encoderHeadingDeg:", encoderHeadingDeg);
			SmartDashboard.putNumber("Val_encoderPIDCorrection:", encoderPIDCorrection);
		}
		return encoderPIDCorrection;
	}
	
	
	// ==============================================
	// ===============================================
	// TIMER AUTONOMOUS METHODS - Forward only
	// ===============================================
	// ==============================================
	
	//==============================================
	// TIME VELOCITY MOVE TO POSITION (Forward only)
	//==============================================
	// This method moves the robot with a predetermined power level and stops at
	// the specified time value. The move will be in brake mode to stop
	// method will check that robot is stopped and set brake mode back to coast and respond
	// that move is done
	public boolean timeVelMoveToPosition(double _MoveToPositionSec, double _MoveToPositionPwrLevel, boolean _isCascadeMove) {
		
		// Read Time
		methodAutoTime = Timer.getFPGATimestamp();
		
		if (!isTimeVelMoveToPositionActive) {
			isTimeVelMoveToPositionActive = true;
			// robotAngle.zeroYaw();
			methodStartTime = methodAutoTime;
			msg("START TIMED VELOCITY MOVE===========================");
			
			// Move calculations
			// Check if move is zero and set to a long time
			if(Math.abs(_MoveToPositionSec) == 0){
				_MoveToPositionSec = 120;
			}
			moveStopTime = methodAutoTime + _MoveToPositionSec;
							
			leftCmdLevel = (Math.signum(_MoveToPositionSec) * _MoveToPositionPwrLevel);
			rightCmdLevel = (Math.signum(_MoveToPositionSec) * _MoveToPositionPwrLevel) * SRXDriveBaseCfg.kDriveStraightFwdCorrection;
			
		} else {
			// Determine drive straight correction if enabled
			if (SRXDriveBaseCfg.isDriveStraightAssistEnabled && isTimeVelMoveToPositionActive) {	
				isSensorCorrectionActive = true;
				driveStraightDirCorrection = getDriveStraightCorrection();
				rightCmdLevel += driveStraightDirCorrection;
			} else{
				// Reset Sensor Correction Active
				isSensorCorrectionActive = false;
				driveStraightDirCorrection = getDriveStraightCorrection();
			}
			
			// Check for sensor stop
			if(_MoveToPositionSec == 0){
				//isSensorStopReached = robotDistance.getCheckSensorStop();
			} else {
				isSensorStopReached = false;
			}
			if((isSensorStopReached) || (methodAutoTime >= moveStopTime)){
				msg("AT TIMED VELOCITY MOVE STOP==================================");
				if (_isCascadeMove) {
					msg("TIMED VEL MOVE TO POSITION DONE IN CASCADE MODE================");
					isTimeVelMoveToPositionActive = false;	
					
				} else {
						// No ramp at end of move
						setDriveTrainRamp(0);
						
						// Apply power level in opposite direction to brake to reduce coast
						rightCmdLevel = -(Math.signum(_MoveToPositionSec) * SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue);
						leftCmdLevel = -(Math.signum(_MoveToPositionSec) * SRXDriveBaseCfg.kAutoLeftMoveStopBrakeValue);
						
					// Delay in sec
					if (!delay(1)) {
						isTimeVelMoveToPositionActive = false;
						// Reset Sensor Correction Active
						isSensorCorrectionActive = false;
						driveStraightDirCorrection = getDriveStraightCorrection();
						//Stop motors
						rightCmdLevel = 0;
						leftCmdLevel = 0;
						
						msg("Timed Vel move to position Time(Sec) = " + (methodAutoTime - methodStartTime));
						msg("TIMED VELOCITY MOVE COMPLETE=======================");
					}
				}
			}
			
		}
		
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		//+++++++++++++++++++++++
		// LOGGING AND DISPLAY
		if (isConsoleDataEnabled){
			System.out.printf("StopTm:%-4.0f ===StartTm:%-4.2f ===MoveTm:%-4.2f ===Correct:%-8.3f%n",
									moveStopTime,
									methodStartTime,
									methodAutoTime,
									driveStraightDirCorrection);		
		}
		return isVelMoveToPositionActive;
		
	}

	//====================================
	// TIME ROTATE TO ANGLE
	//====================================
	public boolean timeRotateToAngle(double _rotateTimeToAngleSec, double _rotatePowerLevel) {
		
		// Read Time
		methodAutoTime = Timer.getFPGATimestamp();
		
		if (!isTimeRotateToAngleActive) {
			isTimeRotateToAngleActive = true;
			methodStartTime = methodAutoTime;
			msg("TIMED ROTATE TO ANGLE IS ACTIVE===========================");
			
			leftCmdLevel = Math.signum(_rotateTimeToAngleSec) * _rotatePowerLevel;
			rightCmdLevel = -Math.signum(_rotateTimeToAngleSec) * _rotatePowerLevel * SRXDriveBaseCfg.kRotateCWDriveStraightCorrection; 
			
			moveStopTime = methodStartTime + _rotateTimeToAngleSec;
			
		
		} else if (methodAutoTime >= moveStopTime){
				msg("TIMED ROTATE TO ANGLE IS AT STOP===========================");
				
				setDriveTrainRamp(0);
				// Apply power level in opposite direction to brake
				rightCmdLevel = (Math.signum(_rotateTimeToAngleSec)*SRXDriveBaseCfg.kAutoRightRotateStopBrakeValue);
				leftCmdLevel = -(Math.signum(_rotateTimeToAngleSec)*SRXDriveBaseCfg.kAutoLeftRotateStopBrakeValue);
			if (!delay(1)) {
				isTimeRotateToAngleActive = false;
				rightCmdLevel = 0;
				leftCmdLevel = 0;
				
				msg("Rotate to angle Time(Sec) = " + (methodAutoTime - methodStartTime));
				msg("TIMED ROTATE TO ANGLE IS DONE=========================");
			}		
		}
		
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		//++++++++++++++++++++++++++++++++++++++++++++++
		// Display data
		if (isConsoleDataEnabled){
			System.out.printf("StopTm:%-8.2f ===RotateTm:%-8.2f%n",
									moveStopTime,
									methodAutoTime);
		}
		return isRotateToAngleActive;
	} 
	//===================================
	// TIME TURN BY TIME TO ANGLE
	//===================================
	public boolean timeTurnToAngle(double _turnAngleSec, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn ) {
		// read time
		methodTime = Timer.getFPGATimestamp();
		
		if (!isTimeTurnToAngleActive) {
			isTimeTurnToAngleActive = true;
			methodStartTime = methodTime;
			msg("TIME TURN TO ANGLE ACTIVE===========================");
			
			// Check turn radius add 4 inches is less than 1/2 wheelbase
			if(_turnRadiusIn < wheelToCenterDistanceIn){
				_turnRadiusIn = wheelToCenterDistanceIn + 4;
			}
			
			//radius is the distance from the center of the robot to a point outside the robot
			//speedRatio =(_turnRadiusIn + wheelToCenterDistanceIn) / (_turnRadiusIn - wheelToCenterDistanceIn);
			
			// Determine which wheel has to speed up to turn
			if (_turnAngleSec > 0) {
				rightCmdLevel = _turnPowerLevel * (1 - (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn)));
				leftCmdLevel = _turnPowerLevel * (1 + (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn)));
		
			} else {
				rightCmdLevel = _turnPowerLevel * (1 + (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn)));
				leftCmdLevel = _turnPowerLevel * (1 - (SRXDriveBaseCfg.kTrackWidthIn / (2 * _turnRadiusIn)));
			}
			
				moveStopTime = methodStartTime + _turnAngleSec;
			
		// Active state -  check for end of time
		} else if (methodTime > moveStopTime) {
				msg("TIME TURN TO ANGLE IS AT STOP===========================");
				if (_isCascadeTurn) {
					isTimeTurnToAngleActive = false;
				} else {
					setDriveTrainRamp(0);	
					// Apply power level in opposite direction for 1 second to brake
					rightCmdLevel = -(Math.signum(_turnAngleSec) * SRXDriveBaseCfg.kAutoRightTurnStopBrakeValue);
					leftCmdLevel = -(Math.signum(_turnAngleSec) * SRXDriveBaseCfg.kAutoLeftTurnStopBrakeValue);
					if (!delay(1)) {
						isTimeTurnToAngleActive = false;
						rightCmdLevel = 0;
						leftCmdLevel = 0;
						msg("Time Turn to angle Time(Sec) = " + (methodTime - methodStartTime));
						msg("TIME TURN TO ANGLE DONE===========================");
				}
			}
		}
		
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		// +++++++++++++++++++++++++++++++++++++++
		// Display data
		if (isConsoleDataEnabled){
			System.out.printf("RgtCmd: %-8.3f ===LftCmd: %-8.3f ===StopTm: %-4.2f ===MethodTm: %-4.2f%n",
									rightCmdLevel,
									leftCmdLevel,
									moveStopTime,
									methodTime);
		}
		return isTurnToAngleActive;
	}
	
	//
	// =======================================================================================
	// SRXDriveBase TEST METHODS
	// =======================================================================================
	//
	
	//==============================
	// TEST METHOD SELECTION
	//==============================
	public void testMethodSelection(){
		
		if(!isTestMethodSelectionActive){
			isTestMethodSelectionActive = true;
			
			Kp_encoderHeadingPID = SmartDashboard.getNumber("Kp_encoderHeadingPID:", Kp_encoderHeadingPID);
			//Ki_encoderHeadingPID = SmartDashboard.getNumber("Ki_encoderHeadingPID:", Ki_encoderHeadingPID);
			//Kd_encoderHeadingPID = SmartDashboard.getNumber("Kd_encoderHeadingPID:", Kd_encoderHeadingPID);
			
		} else {
			
			if (SmartDashboard.getBoolean("TstBtn-MotorEncoderTest:", true)){
				if(!isMtrTestBtnActive){
					msg("START MOTOR ENCODER TEST=============");
					methodStartTime = Timer.getFPGATimestamp();
					isMtrTestBtnActive = true;	
					setRightEncPositionToZero();
					setRightSensorPositionToZero();
					setLeftEncPositionToZero();
					setLeftSensorPositionToZero();
						
				} else if(Timer.getFPGATimestamp() - methodStartTime > .2){
				
					CAL_RightDriveCmdLevel = SmartDashboard.getNumber("CAL_RightDriveCmdLevel:", CAL_RightDriveCmdLevel);
					CAL_LeftDriveCmdLevel = SmartDashboard.getNumber("CAL_LeftDriveCmdLevel:", CAL_LeftDriveCmdLevel);
					
					// checkTestMotorEncoderSetUp()
					checkTestMotorEncoderSetUp();
				}
			} else if(isMtrTestBtnActive) {
				msg("END MOTOR ENCODER TEST=============");
				isTestMethodSelectionActive = false;
				isMtrTestBtnActive = false;
				setStopMotors();
			}
			
			if(SmartDashboard.getBoolean("TstBtn-StepFnc:", false)){
				if(!isTestBtnActive){
					msg("SHUFFLE START TEST STEP FUNCTION=============");
					isTestBtnActive = true;
					if (isLoggingDataEnabled){
						DebugLogger.fopencsv("/home/lvuser/log/SRXDrvStepFnc-" + loggingDataIncrement);
					}
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				// testStepFunction(double _stepFunctionPower, double _stepFunctionTimeSec, boolean _isTestForRightDrive)
				if(!testStepFunction(.3, 2, true)){
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					if(isLoggingDataEnabled){
						isLoggingActive = false;
						loggingDataIncrement += 1;;
						DebugLogger.closecsv();
					}
					SmartDashboard.putBoolean("TstBtn-StepFnc:", false);
					msg("SHUFFLE END TEST STEP FUNCTION=============");
				}	
			}
			
			if(SmartDashboard.getBoolean("TstBtn-DrvStraightCal:", false)){
				
				CAL_kDriveStraightFwdCorrection = SmartDashboard.getNumber("CAL_kDriveStraightFwdCorrection:", CAL_kDriveStraightFwdCorrection);
				if(!isTestBtnActive){
					msg("SHUFFLE START DRIVE STRAIGHT CAL=============");
					isTestBtnActive = true;
					if(isLoggingDataEnabled){
						DebugLogger.fopencsv("/home/lvuser/log/SRXDrvDrvStght-" + loggingDataIncrement);
					}
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				
				// testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel, boolean isDirectionRev)
				if(!testDriveStraightCalibration(50.0, .3, false)){
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					if(isLoggingDataEnabled){
						isLoggingActive = false;
						loggingDataIncrement += 1;
						DebugLogger.closecsv();
					}
					SmartDashboard.putBoolean("TstBtn-DrvStraightCal:", false);
					msg("SHUFFLE END DRIVE STRAIGHT CAL============");
				}	
			}
			
			if(SmartDashboard.getBoolean("TstBtn-VelMoveToPos:", false)){
				if(!isTestBtnActive){
					msg("SHUFFLE START VEL MOVE TO POSITION=============");
					isTestBtnActive = true;
					if(isLoggingDataEnabled){
						DebugLogger.fopencsv("/home/lvuser/log/SRXDrvVelMov-" + loggingDataIncrement);
					}
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				// velMoveToPosition(double _MoveToPositionIn, double _MoveToPositionPwrLevel, boolean _isDirectionRev, boolean _isCascadeMove)
				if(!velMoveToPosition(60, .2, false, false)) {
					if(isLoggingDataEnabled){
						isLoggingActive = false;
						loggingDataIncrement += 1;
						DebugLogger.closecsv();
					}
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					SmartDashboard.putBoolean("TstBtn-VelMoveToPos:", false);
					msg("SHUFFLE END VEL MOVE TO POSITION=============");
				}	
			}
			
				if(SmartDashboard.getBoolean("TstBtn-TimeVelMove:", false)){
					if(!isTestBtnActive){
						msg("SHUFFLE START TIME VEL MOVE TO POSITION=============");
						isTestBtnActive = true;
						setDriveTrainRamp(2);
					}
					// timeVelMoveToPosition(double _MoveToPositionSec, double _MoveToPositionPwrLevel, boolean _isCascadeMove)
					if(!timeVelMoveToPosition(3, .3, false)) {
						isTestMethodSelectionActive = false;
						isTestBtnActive = false;
						SmartDashboard.putBoolean("TstBtn-TimeVelMove:", false);
						msg("SHUFFLE END TIME VEL MOVE TO POSITION=============");
					}	
			}
			
			if(SmartDashboard.getBoolean("TstBtn-RotateToAngle:", false)){
				if(!isTestBtnActive){
					msg("SHUFFLE START ROTATE TO ANGLE=============");
					isTestBtnActive = true;
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				// rotateToAngle(double _rotateToAngle, double _rotatePowerLevel)
				if(!rotateToAngle(-90, .3)) {
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					SmartDashboard.putBoolean("TstBtn-RotateToAngle:", false);
					msg("SHUFFLE END ROTATE TO ANGLE=============");
				}	
			}
			
			if(SmartDashboard.getBoolean("TstBtn-TimeRotate:", false)){
				if(!isTestBtnActive){
					msg("SHUFFLE START TIME ROTATE TO ANGLE=============");
					isTestBtnActive = true;
					setDriveTrainRamp(2);
				}
				// timeRotateToAngle(double _rotateTimeToAngleSec, double _rotatePowerLevel) 
				if(!timeRotateToAngle(1, .3)) {
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					SmartDashboard.putBoolean("TstBtn-TimeRotate:", false);
					msg("SHUFFLE END TIME ROTATE TO ANGLE=============");
				}	
			}
			
			if(SmartDashboard.getBoolean("TstBtn-TurnToAngle:", false)){
				if(!isTestBtnActive){
					msg("SHUFFLE START TURN TO ANGLE=============");
					isTestBtnActive = true;
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				// turnByEncoderToAngle(double _turnAngleDeg, double _turnRadiusIn, double _turnPowerLevel, boolean _isTurnDirectionRev, boolean _isCascadeTurn )
				if(!turnByEncoderToAngle(-90, 25, .30, false, false)) {
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					SmartDashboard.putBoolean("TstBtn-TurnToAngle:", false);
					msg("SHUFFLE END TURN TO ANGLE=============");
				}	
			}
			
			if(SmartDashboard.getBoolean("TstBtn-TimeTurn:", false)){
				if(!isTestBtnActive){
					msg("SHUFFLE START TIME TURN TO ANGLE=============");
					isTestBtnActive = true;
					setDriveTrainRamp(2);
				}
				// timeTurnToAngle(double _turnAngleSec, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn )
				if(!timeTurnToAngle(1, 25, .3, false, false)) {
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					SmartDashboard.putBoolean("TstBtn-TimeTurn:", false);
					msg("SHUFFLE END TIME TURN TO ANGLE=============");
				}	
			}
			if (SmartDashboard.getBoolean("TstBtn-DriveCmdLevel:", false)){
				// SetDriveTrainCmdLevel(double _rightCMDLevel, double _leftCMDLevel)
				SetDriveTrainCmdLevel
				(SmartDashboard.getNumber("CAL_RightDriveCmdLevel:", CAL_RightDriveCmdLevel), SmartDashboard.getNumber("CAL_LeftDriveCmdLevel:", CAL_LeftDriveCmdLevel)); 
			} else {
				isTestMethodSelectionActive = false;
			}
			
			if (SmartDashboard.getBoolean("TstBtn-MagicMotionMove:", false)){
				if(!isTestBtnActive){
					msg("START MAGIC MOTION MOVE TEST=============");
					isTestBtnActive = true;
				
			  //SRXMoveMagic(double _SRXMoveDistanceIn, double _SRXMovePercentVel)
			  } else if(!SRXMoveMagic(60, .3)){
		      
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					SmartDashboard.putBoolean("TstBtn-MagicMotionMove:", false);
					msg("END MAGIC MOTION MOVE TEST======================");
				}
			}
			
			if (SmartDashboard.getBoolean("TstBtn-MagicMotionRotate:", false)){
				if(!isTestBtnActive){
					msg("START MAGIC MOTION ROTATETEST=============");
					isTestBtnActive = true;
			 
			  //SRXRotateMagic(double _SRXRotateAngleDeg, double _SRXRotatePercentVel)
		      } else if(!SRXRotateMagic(90, .3)){	
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					SmartDashboard.putBoolean("TstBtn-MagicMotionRotate:", false);
					msg("END MAGIC MOTION ROTATE TEST======================");
				}
			}
			
			if (SmartDashboard.getBoolean("TstBtn-MagicMotionBase:", false)){
				if(!isTestBtnActive){
					msg("START MAGIC MOTION BASE TEST=============");
					isTestBtnActive = true;
			  // magicMove(int _rightCruiseVel, int _rightAccel, double _rightDistance, int _leftCruiseVel,	int _leftAccel, double _leftDistance) {
		      } else if(!magicMove(190, 190, 4348, 190, 190, 4348)){	
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					SmartDashboard.putBoolean("TstBtn-MagicMotionBase:", false);
					msg("END MAGIC MOTION BASE TEST======================");
				}
			}
			
			if(SmartDashboard.getBoolean("TstBtn-CascadeTest:", false)){
				
				if(!isTestBtnActive){
					msg("SHUFFLE START CASCADE TEST=============");
					isTestBtnActive = true;
					autoCmdSequence = 1;
					//setRightSensorPositionToZero();
					//setLeftSensorPositionToZero();
					//setDriveTrainRamp(0);
					//Timer.delay(0.2);
				}
				
				switch(autoCmdSequence){
					case 1:
						// move 10 inches
						msg("case 1");
						// SRXMoveMagic(double _SRXMoveDistanceIn, double _SRXMoveTimeSec)
						if (!SRXMoveMagic(10, 1)) {
							autoCmdSequence = 2;
							msg("VelMoveDone - next case 2");
						};
						break;
					case 2:
						// turn right 90 deg
						// SRXRotateMagic(double _SRXRotateAngleDeg, double _SRXRotateTimeSec)
						if(!SRXRotateMagic(90, 1)){
							autoCmdSequence = 3;
							msg("TurnByEncoderDone - next case 3");
						};
						break;
					case 3:
						// move 54 in
						// SRXMoveMagic(double _SRXMoveDistanceIn, double _SRXMoveTimeSec)
						if (!SRXMoveMagic(54, 3)) {
							autoCmdSequence = 4;
							msg("VelMoveToPositionDOne - next case 4");
						};
						break;
					case 4:
						// turn left 90 deg
						// SRXRotateMagic(double _SRXRotateAngleDeg, double _SRXRotateTimeSec)
						if(!SRXRotateMagic(-90, 1)){
							autoCmdSequence = 5;
							msg("VelMoveToPositionDOne - next case 5");
							
						};
						break;
					case 5:
						// move 87 in
						// SRXMoveMagic(double _SRXMoveDistanceIn, double _SRXMoveTimeSec)
						if (!SRXMoveMagic(87, 3)) {
							isTestMethodSelectionActive = false;
							SmartDashboard.putBoolean("TstBtn-CascadeTest:", false);
							msg("SHUFFLE END CASCADE TEST=============");
							isTestBtnActive = false;
						};
						break;	
					default:
						isTestMethodSelectionActive = false;
						SmartDashboard.putBoolean("TstBtn-CascadeTest:", false);
						msg("SHUFFLE END CASCADE TEST=============");
						isTestBtnActive = false;
				}
			}
		
		}
	}
	//===============================
	// TEST STEP FUNCTION
	//===============================
	// This provides a pulse(low-High-low) and stays in lowpower. Need to stop motors or call constantly to produce a square wave
	public boolean testStepFunction(double _stepFunctionPower, double _stepFunctionTimeSec, boolean _isTestForRightDrive) {
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			cycleCount += 1;
			// initialize and start at low speed
			if (!isTestStepFunctionActive) {
				isTestStepFunctionActive = true;
				methodStartTime = Timer.getFPGATimestamp();
				msg("START TEST STEP FUNCTION ===============================");
				cycleCount = 0;
				stepFunctionSpeed = _stepFunctionPower * SRXDriveBaseCfg.MaxVel_VelNativeUnits;
				// (sec / 20ms) [iterative robot scan time]
				stepFunctionStopCount = (int)(_stepFunctionTimeSec / 0.02);
			} else if(cycleCount > stepFunctionStopCount) {
				msg("TEST STEP FUNCTION AT TIME STOP=======================");
				stepFunctionSpeed = 0;
				
				// Delay for a specified time to have motion stopped
				if(!delay(3)){
					isTestStepFunctionActive = false;
					msg("TEST STEP FUNCTION DONE=======================");
					methodTime = Timer.getFPGATimestamp() - methodStartTime;
					msg("Step Function Time(Sec) = " + methodTime);
				}
			}
			
			driveRightFollowerMtr.set(ControlMode.Follower, driveRightMasterMtr.getDeviceID());
			driveLeftFollowerMtr.set(ControlMode.Follower, driveLeftMasterMtr.getDeviceID());
			
			if (_isTestForRightDrive){
				driveRightMasterMtr.set(ControlMode.Velocity, stepFunctionSpeed);
				driveLeftMasterMtr.set(ControlMode.Velocity, 0);
			} else {
				driveLeftMasterMtr.set(ControlMode.Velocity, stepFunctionSpeed);
				driveRightMasterMtr.set(ControlMode.Velocity, 0);
			}
				
			// +++++++++++++++++++++++++++++++++++++
			// Display data
			if (isConsoleDataEnabled){
				System.out.printf("StepVel:%-8.3f==RightVel:%-8.2f==RightErr:%-8.2f==LeftVel:%-8.2f==LeftErr:%-8.2f%n",
						stepFunctionSpeed,
						getRightSensorVelocity(),
						getRightCloseLoopError(),
						getLeftSensorVelocity(),
						getLeftCloseLoopError());
			}
		} 
		return isTestStepFunctionActive;
	}
	
		//===================================
		// TEST - MOTOR-ENCODER SETUP TEST
		//===================================
		public void checkTestMotorEncoderSetUp(){ 
			
			//rightEncoderPosition = driveRightMasterMtr.getSensorCollection().getQuadraturePosition();
			rightSensorPositionValue = driveRightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
			rightSensorPositionRead = getRightSensorPosition();
			
			//leftEncoderPosition = driveLeftMasterMtr.getSensorCollection().getQuadraturePosition();
			leftSensorPositionValue = driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
			leftSensorPositionRead = getLeftSensorPosition();
		
			SetDriveTrainCmdLevel(CAL_RightDriveCmdLevel, CAL_LeftDriveCmdLevel);
			
			// +++++++++++++++++++++++++++++++++
			// DATA DISPLAY
			if (isConsoleDataEnabled){
				System.out.printf("LeftPos:%-4.0f =LeftPosRead:%-4.0f =RightPos:%-4.0f =RightPosRead:%-4.0f%n",
									leftSensorPositionValue,
									leftSensorPositionRead,
									rightSensorPositionValue, 
									rightSensorPositionRead);
			}
		}
	
	//===================================
	// TEST DRIVE STRAIGHT CALIBRATION
	//===================================
	public boolean testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel, boolean _isDirectionRev){
		
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		
		if (!isTestMoveForStraightCalActive){
			msg("START CALIBARTION======================================");
			methodStartTime = Timer.getFPGATimestamp();
			isTestMoveForStraightCalActive = true;
			CAL_kDriveStraightFwdCorrection = SmartDashboard.getNumber("CAL_kDriveStraightFwdCorrection:", CAL_kDriveStraightFwdCorrection);
			CAL_kDriveStraightRevCorrection = SmartDashboard.getNumber("CAL_kDriveStraightRevCorrection:", CAL_kDriveStraightRevCorrection);
			
			if(_isDirectionRev){
				leftCmdLevel = -_pwrLevel;
				rightCmdLevel = (-_pwrLevel * CAL_kDriveStraightRevCorrection); 
				leftEncoderStopCount = -(_testDistanceIn / SRXDriveBaseCfg.kLeftInchesPerCount);
			}
			else{
				leftCmdLevel = _pwrLevel;
				rightCmdLevel = (_pwrLevel * CAL_kDriveStraightFwdCorrection); 
				leftEncoderStopCount = (_testDistanceIn / SRXDriveBaseCfg.kLeftInchesPerCount);
			}
			
			if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.0f+++LftEnc:%-8.0f +++RgtEnc:%-8.0f+++LftCmd:%-8.4f+++RgtCmd:%-8.4f%n", 
								leftEncoderStopCount, 
								leftSensorStartPositionRead, 
								rightSensorStartPositionRead,
								leftCmdLevel,
								rightCmdLevel);
			}
			
	
		// Test for stopping movement
		} else {
			isCalAtStop = ((_isDirectionRev)? (leftSensorPositionRead <= leftEncoderStopCount) : (leftSensorPositionRead >= leftEncoderStopCount));	
		}
		
		if(isCalAtStop)	{
			msg("CALIBRATION AT STOP ===========================================");
			
			setDriveTrainRamp(0);
			// Apply power level in opposite direction for 1 second to brake
			rightCmdLevel = -SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue;
			leftCmdLevel = -SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue;
			if (!delay(1)) {
				msg("CALIBRATION END ==================================");
				isTestMoveForStraightCalActive = false;
				isCalAtStop = false;
				rightCmdLevel = 0;
				leftCmdLevel = 0;
				methodTime = Timer.getFPGATimestamp() - methodStartTime;
				msg("Drive Straight Calibration Time(Sec) = " + methodTime);
			}	
		}
		
		calCorrectionFactor = leftSensorPositionRead / rightSensorPositionRead;
		headingDeg = (leftSensorPositionRead - rightSensorPositionRead) / SRXDriveBaseCfg.kTrackWidthIn;
		
		// Output to SRX drive modules
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		//++++++++++++++++++++++++++++++++
		// LOGGING AND DISPLAY
		if (isLoggingDataEnabled) {
			if(!isLoggingActive){
				logString = ",leftEncStopCnt, leftPos, RightPos, corFactor, headingDeg";
				DebugLogger.data(logString);	
			} else {
				logString = String.format(",%8.0f,%8.0f,%8.0f,%8.4f,%8.4f", 
											leftEncoderStopCount, 
											leftSensorPositionRead, 
											rightSensorPositionRead,
											calCorrectionFactor,
											headingDeg);
				DebugLogger.data(logString);
			}
		}
		//Print on console data
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.0f===LftEnc:%-8.0f ===RgtEnc:%-8.0f===Correction:%-8.4f==Heading:%-8.2f%n", 
								leftEncoderStopCount, 
								leftSensorPositionRead, 
								rightSensorPositionRead,
								calCorrectionFactor,
								headingDeg);
		}
		return isTestMoveForStraightCalActive;
		
	}	
	
	//===================
	// DELAY
	//===================
	// This delay is looked at each scan so delay = seconds + scan(~20ms)
	public boolean delay(double _seconds){
		if (!isDelayActive) {
			isDelayActive = true;
			delayStartTime = Timer.getFPGATimestamp();
		} else if (Timer.getFPGATimestamp() >= (delayStartTime + _seconds)){
			isDelayActive = false;
		}
		return isDelayActive;
	}
	
	
	/**
	* =======================================================================================
	* INDEX AND PROFILE COMMANDS
	* =======================================================================================
	*/
	
	public boolean SRXCalStdTrapezoidMoveMagic(double _SRXRightDistanceIn, double _SRXRightTimeSec, double _SRXLeftDistanceIn, double _SRXLeftTimeSec) {
		
		if(!isSRXCalStdTrapezoidMoveMagicActive){
			msg("START MOTION MAGIC CALCULATIONS ==================================");
			methodStartTime = Timer.getFPGATimestamp();
			isSRXCalStdTrapezoidMoveMagicActive = true;
			mgmvLeftDistance = (int)(_SRXLeftDistanceIn / SRXDriveBaseCfg.kInchesPerCount);
			mgmvLeftCruiseVel = (int)(1.5 * (mgmvLeftDistance/(_SRXLeftTimeSec / SRXDriveBaseCfg.kvelMeasurePeriodSec)));
			mgmvLeftAccel = (int)((4.5 * mgmvLeftDistance) / ((_SRXLeftTimeSec / SRXDriveBaseCfg.kvelMeasurePeriodSec) * (_SRXLeftTimeSec / SRXDriveBaseCfg.kvelMeasurePeriodSec)));
			
			mgmvRightDistance = (int)( _SRXRightDistanceIn / SRXDriveBaseCfg.kInchesPerCount);
			mgmvRightCruiseVel = (int)(1.5 * (mgmvRightDistance/(_SRXRightTimeSec / SRXDriveBaseCfg.kvelMeasurePeriodSec)));
			mgmvRightAccel = (int)((4.5 * mgmvRightDistance) / ((_SRXRightTimeSec / SRXDriveBaseCfg.kvelMeasurePeriodSec) * (_SRXRightTimeSec / SRXDriveBaseCfg.kvelMeasurePeriodSec)));
			
			if (isConsoleDataEnabled){
				System.out.printf("RgtD:%-8.2f ++RgtV:%-8d ++RgtA:%-8d ++LftD:%-8.2f ++LftV:%-8d ++LftA:%-8d%n", 
						mgmvRightDistance, 
						mgmvRightCruiseVel,
						mgmvRightAccel,
						mgmvLeftDistance,
						mgmvLeftCruiseVel,
						mgmvLeftAccel);
			}
		} else if(!magicMove(mgmvRightCruiseVel, mgmvRightAccel, mgmvRightDistance, mgmvLeftCruiseVel,	mgmvLeftAccel, mgmvLeftDistance)){
			isSRXCalStdTrapezoidMoveMagicActive = false;
			methodTime = Timer.getFPGATimestamp() - methodStartTime;
			msg("SRXCal Std Trap Move Magic(Sec) = " + methodTime);
			msg("END MOTION MAGIC CALCULATIONS AND MOVE================");
		}
			
		return isSRXCalStdTrapezoidMoveMagicActive;
	}
	
	public boolean SRXMoveMagic(double _SRXMoveDistanceIn, double _moveMagicToPositionPwrLevel) {
		if(!isSRXMoveMagicActive){
			msg("START MOTION MAGIC CALCULATIONS ==================================");
			methodStartTime = Timer.getFPGATimestamp();
			isSRXMoveMagicActive = true;
			mgmvLeftDistance = (_SRXMoveDistanceIn / SRXDriveBaseCfg.kInchesPerCount);
			mgmvLeftCruiseVel = (int)(_moveMagicToPositionPwrLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits) ;			
			mgmvLeftAccel = ((int)mgmvLeftCruiseVel*2);
			
			mgmvRightDistance = mgmvLeftDistance;
			mgmvRightCruiseVel = mgmvLeftCruiseVel;
			mgmvRightAccel = mgmvLeftAccel;
			
			if (isConsoleDataEnabled){
				System.out.printf("RgtD:%-8.2f ++RgtV:%-8d ++RgtA:%-8d ++LftD:%-8.2f ++LftV:%-8d ++LftA:%-8d%n", 
						mgmvRightDistance, 
						mgmvRightCruiseVel,
						mgmvRightAccel,
						mgmvLeftDistance,
						mgmvLeftCruiseVel,
						mgmvLeftAccel);
			}
		} else if(!magicMove(mgmvRightCruiseVel, mgmvRightAccel, mgmvRightDistance, mgmvLeftCruiseVel,	mgmvLeftAccel, mgmvLeftDistance)){
			isSRXMoveMagicActive = false;
			setStopMotors();
			methodTime = Timer.getFPGATimestamp() - methodStartTime;
			msg("SRXMove Magic(Sec) = " + methodTime);
			msg("END MOTION MAGIC CALCULATIONS AND MOVE================");
		}
			
		return isSRXMoveMagicActive;
	}
	public boolean SRXRotateMagic(double _SRXRotateAngleDeg, double _rotateMagicPowerLevel) {
		if(!isSRXRotateMagicActive){
			msg("START ROTATE MOTION MAGIC CALCULATIONS ==================================");
			isSRXRotateMagicActive = true;
			methodStartTime = Timer.getFPGATimestamp();
			
			// rotationEncoderStopCount = C(=>PI*D) * (angle as a fraction of C)			                                
			mgmvLeftDistance = (int)(Math.PI*(SRXDriveBaseCfg.kTrackWidthIn) * SRXDriveBaseCfg.kEncoderCountsPerIn * (_SRXRotateAngleDeg / 360));;
			mgmvLeftCruiseVel = (int)(_rotateMagicPowerLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits) ;	
			mgmvLeftAccel = ((int)mgmvLeftCruiseVel*2);
			
			
			mgmvRightDistance = -mgmvLeftDistance;
			mgmvRightCruiseVel = -mgmvLeftCruiseVel;
			mgmvRightAccel = -mgmvLeftAccel;
			
			if (isConsoleDataEnabled){
				System.out.printf("RgtD:%-8.2f ++RgtV:%-8d ++RgtA:%-8d ++LftD:%-8.2f ++LftV:%-8d ++LftA:%-8d%n", 
						mgmvRightDistance, 
						mgmvRightCruiseVel,
						mgmvRightAccel,
						mgmvLeftDistance,
						mgmvLeftCruiseVel,
						mgmvLeftAccel);
			}
		} else if(!magicMove(mgmvRightCruiseVel, mgmvRightAccel, mgmvRightDistance, mgmvLeftCruiseVel,	mgmvLeftAccel, mgmvLeftDistance)){
			isSRXRotateMagicActive = false;
			setStopMotors();
			methodTime = Timer.getFPGATimestamp() - methodStartTime;
			msg("SRXRotate Magic(Sec) = " + methodTime);
			msg("END ROTATE MOTION MAGIC CALCULATIONS AND MOVE================");
		}
			
		return isSRXRotateMagicActive;
	}
	
	// This method performs a SRX magic motion command from user calculated values
	// User should note that the right drive distance needs to be corrected by kDriveStraightFwdCorrection
	public boolean magicMove(int _rightCruiseVel, int _rightAccel, double _rightDistance, int _leftCruiseVel,
			int _leftAccel, double _leftDistance) {
		
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		
		if (!isSRXMagicMoveActive) {
			isSRXMagicMoveActive = true;
			msg("START MOTION MAGIC ==================================");
			methodStartTime = Timer.getFPGATimestamp();
			/* Set relevant frame periods to be at least as fast as periodic rate*/
			driveRightMasterMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, SRXTimeoutValueMs);
			driveRightMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			driveRightMasterMtr.setSelectedSensorPosition(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx, SRXTimeoutValueMs);
			driveRightMasterMtr.configMotionCruiseVelocity(_rightCruiseVel, SRXTimeoutValueMs);
			driveRightMasterMtr.configMotionAcceleration(_rightAccel, SRXTimeoutValueMs);
			
			driveLeftMasterMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, SRXTimeoutValueMs);
			driveLeftMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);	
			driveLeftMasterMtr.setSelectedSensorPosition(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx, SRXTimeoutValueMs);
			driveLeftMasterMtr.configMotionCruiseVelocity(_leftCruiseVel, SRXTimeoutValueMs);
			driveLeftMasterMtr.configMotionAcceleration(_leftAccel, SRXTimeoutValueMs);

			//_rightDistance = (int)(_rightDistance * SRXDriveBaseCfg.kDriveStraightFwdCorrection);
			//_leftDistance = (int)(_leftDistance);
		} else {
			motionMagicLeftPos = driveLeftMasterMtr.getActiveTrajectoryPosition();
			motionMagicLeftVel = driveLeftMasterMtr.getActiveTrajectoryVelocity();
			motionMagicRightPos = driveRightMasterMtr.getActiveTrajectoryPosition();
			motionMagicRightVel = driveRightMasterMtr.getActiveTrajectoryVelocity();
			
			if ((Math.abs(motionMagicLeftPos) >= (Math.abs(_leftDistance)-10)) && (Math.abs(motionMagicRightPos) >= (Math.abs(_rightDistance)-10))) {
				//driveRightMasterMtr.set(ControlMode.MotionMagic, 0); 
				//driveLeftMasterMtr.set(ControlMode.MotionMagic, 0);
				setStopMotors();
				isSRXMagicMoveActive = false;
				methodTime = Timer.getFPGATimestamp() - methodStartTime;
				msg("Motion Magic(Sec) = " + methodTime);
				msg("END MOTION MAGIC ========================");
			}
		}
		// Output commands to SRX's
		driveRightFollowerMtr.set(ControlMode.Follower, driveRightMasterMtr.getDeviceID());
		driveLeftFollowerMtr.set(ControlMode.Follower, driveLeftMasterMtr.getDeviceID());
		
		driveRightMasterMtr.set(ControlMode.MotionMagic, _rightDistance); 
		driveLeftMasterMtr.set(ControlMode.MotionMagic, _leftDistance);
		
		if (isConsoleDataEnabled){
			System.out.printf("LftEnc:%-8.0f ==RgtEnc:%-8.0f ==LftSRXVel:%-8.2f ==LftSRXPos:%-8.2f ==RgtSRXVel:%-8.2f ==RgtSRXPos:%-8.2f %n", 
								leftSensorPositionRead, 
								rightSensorPositionRead,
								motionMagicLeftVel,
								motionMagicLeftPos,
								motionMagicRightVel,
								motionMagicRightPos);
		}
		return isSRXMagicMoveActive;
	}
}

