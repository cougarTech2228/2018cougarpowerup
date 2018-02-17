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
//import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRXDriveBase {
	// The following is for the addition of a Navx and ultrasonic sensors
	// public class SRXDriveBase( AngleIF _angle, DistanceIF _distance)
	// AngleIF robotAngle = _angle;
	// DistanceIF robotDistance = _distance;
	
	
	// DifferentialDrive or tank motors
	private WPI_TalonSRX driveRightMasterMtr;
	private WPI_TalonSRX driveRightFollowerMtr;
	private WPI_TalonSRX driveLeftMasterMtr;
	private WPI_TalonSRX driveLeftFollowerMtr;

	private DifferentialDrive driveStyle;
	
	private int cycleCount = 1;
	private int SRXTimeoutValueMs = 10;
	private int state = 0;
	private int CorrectionSensorType = 0;
	private int stepFunctionStopCount = 0;
	private int autoCmdSequence = 1;
	
	//TODO
	//private DrvStraightCorSensor correctionSensor = 0;
	
	private double leftEncoderStopCount = 0;
	//private double rightDrvTrainTargetPosSetPt;
	//private double leftDrvTrainTargetPosSetPt;
	
	private double leftCmdLevel = 0;
	//private double lastLeftCmdLevel = 0;
	private double rightCmdLevel = 0;
	private double rotationEncoderStopCount = 0;
	private double driveStraightDirCorrection = 0;
	private double speedRatio = 0;
	private double wheelToCenterDistanceIn = 0;
	private double outerDistanceStopCnt = 0;
	private double headingDeg =0;
	//private double drivePerpendicularDirCorrection = 0;
	private double calCorrectionFactor = 0;
	private double moveStopCount = 0;
	private double pulSqStartTimeSec = 0;
	private double sqWaveVel = 0;
	private double delayStartTime = 0;
	
	//private double driveStraightEncoderError = 0;
	private double rightSensorStartPositionRead = 0;
	private double rightSensorPositionRead = 0;
	private double leftSensorStartPositionRead = 0;
	private double leftSensorPositionRead = 0;
	
	//private double EncoderHeadingRate = 0;
	private double Kp_encoderHeadingPID = 0.1;
	private double Ki_encoderHeadingPID = 0;
	private double Kd_encoderHeadingPID = 0;
	private double encoderHeading_IAccumMax = 10;
	private double Ki_PIDHeadingAccum = 0;
	private double encoderPIDCorrection =0;
	
	private double previousEncoderHeadingDeg = 0;
	private double encoderHeadingDeg = 0;
	private double EncoderHeadingCorrection = 0;
	private double EncoderHeadingRate = 0;
	private double sensorCorrection = 0;
	private double tuneCorrectionFactor =0;
	private double lastCorrectionFactor = 0;
	private double maxSensorCorrection = 0;
	private double correction = 0.93;
	private double stepFunctionSpeed = 0;
	
	//  Program flow switches
	private boolean isConsoleDataEnabled = true;
	private boolean isLoggingDataEnabled = false;
	private boolean islogSRXDriveActive = false;
	
	private boolean isVelMoveToPositionActive = false;
	private boolean isRotateToAngleActive = false;
	private boolean isTurnToAngleActive = false;
	private boolean isSRXMagicMoveActive = false;
	private boolean isLowTimeActive = false;
	private boolean isPulse_SqWaveFnctStartActive = false;
	private boolean isMovePerpendicularActive = false;
	private boolean isTestMoveForStraightCalActive = false;
	private boolean isDelayActive = false;
	private boolean isDriveTrainMoving = false;
	private boolean isSensorStopReached = false;
	private boolean isTestMethodSelectionActive = false;
	private boolean isSensorCorrectionActive = false;
	private boolean isAutoTuneCorrectionFactorActive = false;
	private boolean isTuneDirectionPos = false;
	private boolean isLastTuneDirectionPos = false;
	private boolean isTestStepFunctionActive = false;
	private boolean isEcoderReset = false;
	
	private String logSRXDriveString = " ";
	private String lastMsgString = " ";
	
	
	// SRXDriveBase Class Constructor
	public SRXDriveBase() {
	
		
		// Create CAN SRX motor controller objects
		driveRightMasterMtr = new WPI_TalonSRX(RobotMap.CAN_ID_1);
		driveRightFollowerMtr = new WPI_TalonSRX(RobotMap.CAN_ID_2);
		driveLeftMasterMtr = new WPI_TalonSRX(RobotMap.CAN_ID_3);
		driveLeftFollowerMtr = new WPI_TalonSRX(RobotMap.CAN_ID_4);

		/*
		 * Set right/left masters and right/left followers
		 */
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
		
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			driveRightMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			driveRightMasterMtr.configAllowableClosedloopError(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kClosedLoopErr, SRXTimeoutValueMs);
			driveRightMasterMtr.config_kF(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrFeedForwardGain, SRXTimeoutValueMs);
			driveRightMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrProportionalGain, SRXTimeoutValueMs);
			driveRightMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrIntegralGain, SRXTimeoutValueMs); 
			driveRightMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrDerivativeGain, SRXTimeoutValueMs);
			driveRightMasterMtr.config_IntegralZone(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrIzone, SRXTimeoutValueMs);
		}
		driveRightMasterMtr.clearStickyFaults(SRXTimeoutValueMs);
		driveRightMasterMtr.getSensorCollection().setQuadraturePosition(0, SRXTimeoutValueMs);
		// Set Right master to percentVbus mode
		driveRightMasterMtr.set(ControlMode.PercentOutput,0);
		driveRightMasterMtr.set(ControlMode.Velocity, 0);
		
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
		
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			driveLeftMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			driveLeftMasterMtr.configAllowableClosedloopError(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kClosedLoopErr, SRXTimeoutValueMs);
			driveLeftMasterMtr.config_kF(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrFeedForwardGain, SRXTimeoutValueMs);
			driveLeftMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrProportionalGain, SRXTimeoutValueMs);
			driveLeftMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrIntegralGain, SRXTimeoutValueMs); 
			driveLeftMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrDerivativeGain, SRXTimeoutValueMs);
			driveLeftMasterMtr.config_IntegralZone(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveleftMstrIzone, SRXTimeoutValueMs);
		}
		driveLeftMasterMtr.clearStickyFaults(SRXTimeoutValueMs);
		driveLeftMasterMtr.getSensorCollection().setQuadraturePosition(0, SRXTimeoutValueMs);
		driveLeftMasterMtr.set(ControlMode.PercentOutput,0);
		driveLeftMasterMtr.set(ControlMode.Velocity, 0);
		
		// SET UP LEFT FOLLOWER =======================
		driveLeftFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveLeftFollowerMtrReversed);
		driveLeftFollowerMtr.clearStickyFaults(SRXTimeoutValueMs);
		driveLeftFollowerMtr.set(ControlMode.Follower, driveLeftMasterMtr.getDeviceID());
		
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
	/**
	* =======================================================================================
	* SRXBaseDrive SET/CONFIG METHODS
	* =======================================================================================
	*/
	
	public void setRightEncPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		driveRightMasterMtr.getSensorCollection().setQuadraturePosition(0, 15);
	}
	
	public void setRightSensorPositionToZero() {
		//previousEMAAccelFltrThrottleValue;/ SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		driveRightMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx, 25);
	}
	
	public void setLeftEncPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		driveLeftMasterMtr.getSensorCollection().setQuadraturePosition(0, 25);
	}

	public void setLeftSensorPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		driveLeftMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx, 25);
	}
	public void setCorrectionSensor(int _CorrectionSensorSelect){
		//0-none, 1-encoder, 2-Distance, 3-IMU
		CorrectionSensorType = _CorrectionSensorSelect;
	}
	
	public void setBrakeMode(boolean _isBrakeEnabled) {
		driveRightMasterMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		driveRightFollowerMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		driveLeftMasterMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		driveLeftFollowerMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
	}
	
	public void setStopMotors(){
		if(SRXDriveBaseCfg.isSRXClosedLoopEnabled){
			driveRightMasterMtr.set(ControlMode.Velocity, 0);
			driveLeftMasterMtr.set(ControlMode.Velocity, 0);
		} else {
			driveRightMasterMtr.set(ControlMode.PercentOutput, 0);
			driveLeftMasterMtr.set(ControlMode.PercentOutput, 0);
		}
	}
	
	public void setEnableConsoleData(boolean _consoleData){
		isConsoleDataEnabled = _consoleData;
	}
	
	public void setEnableLoggingData(boolean _loggingData){
		isLoggingDataEnabled = _loggingData;
	}
	
	public void setSRXDriveBaseInit() {
		isVelMoveToPositionActive = false;
		isRotateToAngleActive = false;
		isTurnToAngleActive = false;
		isSRXMagicMoveActive = false;
		isLowTimeActive = false;
		isPulse_SqWaveFnctStartActive = false;
		isMovePerpendicularActive = false;
		isTestMoveForStraightCalActive = false;
		isDelayActive = false;
		isSensorStopReached = false;
		isTestMethodSelectionActive = false;
		isSensorCorrectionActive = false;
		isAutoTuneCorrectionFactorActive = false;
		isTestStepFunctionActive = false;
		isEcoderReset = false;
		
		loadShuffleBoardParmeters();
		setDriveTrainRamp(0);
		setRightSensorPositionToZero();
		setLeftSensorPositionToZero();
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
	
	/**
	* =======================================================================================
	* SRXBaseDrive GET METHODS
	* =======================================================================================
	*/
	
	// ============== RIGHT MASTER MOTOR
	
	public double getRightEncoderPosition() {
		// This value is updated every 160ms
		if (SRXDriveBaseCfg.isRightEncoderSensorReversed){
			return -driveRightMasterMtr.getSensorCollection().getQuadraturePosition();
		} else{
			return driveRightMasterMtr.getSensorCollection().getQuadraturePosition();
		}
	}
	
	public double getRightEncoderVelocity(){
		// This value is updated every 160ms
		return driveRightMasterMtr.getSensorCollection().getQuadratureVelocity();
	}
	
	public double getRightSensorPosition(){
		// This value is updated every 20ms
		return driveRightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	public double getRightMstrMtrCurrent() {
		return driveRightMasterMtr.getOutputCurrent();
	}

	public double getRightFollowerMtrCurrent() {
		return driveRightFollowerMtr.getOutputCurrent();
	}

	public double getRightSensorVelocity() {
		return driveRightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx);
	}

	public double getRightCloseLoopError() {
		return driveRightMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	// ==============================LEFT MASTER MOTOR
	
	public double getLeftEncoderPosition() {
		// This value is updated every 160ms
		if (SRXDriveBaseCfg.isLeftEncoderSensorReversed){
			return -driveLeftMasterMtr.getSensorCollection().getQuadraturePosition();
		} else{
			return driveLeftMasterMtr.getSensorCollection().getQuadraturePosition();
		}
	}
	
	public double getLeftEncoderVelocity(){
		// This value is updated every 160ms
		return driveLeftMasterMtr.getSensorCollection().getQuadratureVelocity();
	}
	
	public double getLeftSensorPosition(){
		// This value is updated every 20ms
		if (SRXDriveBaseCfg.isLeftEncoderSensorReversed){
			return -driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		} else{
			return driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		}
	}
	
	public double getLeftMstrMtrCurrent() {
		return driveLeftMasterMtr.getOutputCurrent();
	}
	
	public double getLeftFollowerMtrCurrent() {
		return driveLeftFollowerMtr.getOutputCurrent();
	}
	
	public double getLeftSensorVelocity() {
		return driveLeftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx);
	}

	public double getLeftCloseLoopError() {
		return driveLeftMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx);
	}

	public double getBusVoltage() {
		return driveLeftMasterMtr.getBusVoltage();
	}
	// use "setCorrectionSensor(int _CorrectionSensorSelect)" or set in variable int
	public double getDriveStraightCorrection(){
		//0-none, 1-encoder, 2-Distance, 3-IMU
		switch(CorrectionSensorType){
				case 0:
					sensorCorrection = 0;
					break;
				case 1:
					// left/right position and isSensorCorrectionActive are global variables
					sensorCorrection = capCorrection(encoderAngleCorrection(leftSensorPositionRead, rightSensorPositionRead, isSensorCorrectionActive));
					break;
				case 2:
					// sensorCorrection = capCorrection(robotDistanceSensor.getAngleCorrection());
					break;
				case 3:
					// sensorCorrection = capCorrection(robotAngle.getAngleCorrection());
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
	
	/**
	* =======================================================================================
	* STATUS METHODS
	* =======================================================================================
	*/	
	public void loadShuffleBoardParmeters() {
		SmartDashboard.putBoolean("TestStepFunction:", false);
		SmartDashboard.putBoolean("DriveStraightCal:", false);
		SmartDashboard.putBoolean("DriveStraightAutoCal:", false);
		SmartDashboard.putBoolean("VelMoveToPosition:", false);
		SmartDashboard.putBoolean("RotateToAngle:", false);
		SmartDashboard.putBoolean("TurnToAngle:", false);
		SmartDashboard.putBoolean("CascadeTest:", false);
		
		SmartDashboard.putNumber("Kp_encoderHeadingPID:", Kp_encoderHeadingPID);
		SmartDashboard.putNumber("Ki_encoderHeadingPID:", Ki_encoderHeadingPID);
		SmartDashboard.putNumber("Kd_encoderHeadingPID:", Kd_encoderHeadingPID);
	}
	
	// Reads encoder, velocity, current, error, and displays on smartdashboard
	public void UpdateSRXDriveDataDisplay() {

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
		if (isLoggingDataEnabled){
			if (!islogSRXDriveActive){
				islogSRXDriveActive = true;
				logSRXDriveString = "Right Bus Voltage,Right Output Voltage,Right Master Current,Right Encoder Count,Right Follower Current,Left Bus Voltage,Left Output Voltage,Left Master Current,Left Encoder Count,Left Follower Current";
				// Log data
				DebugLogger.data(logSRXDriveString);
			} else {
				logSRXDriveString = String.format("%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f", 
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
		} else {
			islogSRXDriveActive = false;
		}
	} 
	
	private void msg(String _msgString){
		if (_msgString != lastMsgString){
			System.out.println(_msgString);
			lastMsgString = _msgString;}
		}
	/**
	* =======================================================================================
	* TELEOP METHODS
	* =======================================================================================
	*/
	/*
	 * Note: left drive is master drive axis for the robot - the right drive
	 * will be modified for driving straight
	 *
	 * NOTE: Motion command with open loop reflect power levels (-1 to 1) * (the motor bus voltage). 
	 * Motion command with closed loop reflect speed level => (-1 to 1) * (top motor RPM)
	 */
	public void SetDriveTrainCmdLevel(double _rightCMDLevel, double _leftCMDLevel) {
		rightCmdLevel = _rightCMDLevel;
		leftCmdLevel = _leftCMDLevel;
		
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			// Output commands to SRX modules set as [% from (-1 to 1)] x MaxVel_VelNativeUnits
			driveRightMasterMtr.set(ControlMode.Velocity, (rightCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
			driveLeftMasterMtr.set(ControlMode.Velocity, (leftCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
		} else {
			driveRightMasterMtr.set(ControlMode.PercentOutput,rightCmdLevel);
			driveLeftMasterMtr.set(ControlMode.PercentOutput,leftCmdLevel);
		}
	}

	/*
	 * WPI throttle and turn commands This method uses WPI library methods to
	 * drive the robot with a throttle and turn input. Drives were set up by:
	 * driveStyle = new RobotDrive(driveRightMasterMtr, driveLeftMasterMtr); The
	 * throttle would be the game controller Y-axis(joystick fwd/rev) and turn
	 * would be game controller X-axis(joystick left/right)
	 *
	 * NOTE: WPILib throttleValue and turnValue are open loop power levels (-1
	 * to 1) * (the motor bus voltage). The speed is determined by this power
	 * level and the load to the motor.
	 */
	 //======================== THIS HAS NOT BEEN TESTED ==========================
	public void WPISetThrottleTurn(double _WPIThrottleValue, double _WPITurnValue) {
		// parms: move, turn, squared inputs
		driveStyle.arcadeDrive(_WPIThrottleValue, _WPITurnValue, false);	
	}

	/*
	 * setThrottleTurn is both open loop and closed loop control with drive
	 * straight/drive perpendicular correction
	 */
	public void setThrottleTurn(double _throttleValue, double _turnValue, boolean _isDrivingPerpendicular) {
		if (SRXDriveBaseCfg.isDriveStraightAssistEnabled && _turnValue == 0) {
			driveStraightDirCorrection = getDriveStraightCorrection();
			
			// Calculate cmd level in terms of PercentVbus; range (-1 to 1)
			leftCmdLevel = _throttleValue + _turnValue + driveStraightDirCorrection;
			rightCmdLevel = ((_throttleValue* SRXDriveBaseCfg.kDriveStraightCorrection) - _turnValue) - driveStraightDirCorrection;
		} else {
			leftCmdLevel = _throttleValue  +_turnValue;
			rightCmdLevel = ((_throttleValue * SRXDriveBaseCfg.kDriveStraightCorrection) - _turnValue);
		}
		//msg("TopRPM" + SRXDriveBaseCfg.kTopRPM + "Cycles Per Rev" + SRXDriveBaseCfg.kDriveEncoderCyclesPerRev + "CNT-PER-REV" + //SRXDriveBaseCfg.kCountsPerRevolution + "MaxRPM" +SRXDriveBaseCfg.MaxVel_VelNativeUnits);
		
		// Output commands to SRX modules set as [% from (-1 to 1)] x MaxVel_VelNativeUnits for feedback control
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		//++++++++++++++++++++++++++++++++++++++
		// Display data
		if (isConsoleDataEnabled){
			System.out.printf("LftCmd:%-8.3f==RgtCmd:%-8.3f==LftVel:%-8.3f==RgtVel:%-8.3f==LftCur:%-8.3f==RgtCur:%-8.3f%n", 
									leftCmdLevel, 
									rightCmdLevel,
									getLeftSensorVelocity(),
									getRightSensorVelocity(),
									getLeftMstrMtrCurrent(),
									getRightMstrMtrCurrent());
		}
	}
	
	/**
	* =======================================================================================
	* AUTONOMOUS METHODS
	* =======================================================================================
	*/
	//==============================================
	// VELOCITY MOVE TO POSITION
	//==============================================
	// This method moves the robot with a predetermined power level and stops at
	// the specified position value. The move will be in brake mode to stop
	// method will check that robot is stopped and set brake mode back to coast and respond
	// that move is done
	public boolean velMoveToPosition(double _MoveToPositionIn, double _MoveToPositionPwrLevel, boolean _isCascadeMove) {
		
		// Read encoders
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		
		if (!isVelMoveToPositionActive) {
			msg("START VELOCITY MOVE===========================");
			isVelMoveToPositionActive = true;
			
			// Move calculations
			// Check if move is zero and set to a long distance
			if(Math.abs(_MoveToPositionIn) == 0){
				_MoveToPositionIn = 1000;
			}
			moveStopCount = ((Math.abs(_MoveToPositionIn) * SRXDriveBaseCfg.kLeftEncoderCountsPerIn) 
							- SRXDriveBaseCfg.kAutoMoveCoastToStopCounts) 
							+ leftSensorPositionRead;
							
			leftCmdLevel = (Math.signum(_MoveToPositionIn) * _MoveToPositionPwrLevel);
			rightCmdLevel = (Math.signum(_MoveToPositionIn) * _MoveToPositionPwrLevel) * SRXDriveBaseCfg.kDriveStraightCorrection;
			
		} else {
			// Check for drive straight correction
			if (SRXDriveBaseCfg.isDriveStraightAssistEnabled){
				rightCmdLevel += getDriveStraightCorrection();
			}
			
			// Check for sensor stop
			if(_MoveToPositionIn == 0){
				//isSensorStopReached = DistanceSensor.getCheckSensorStop();
			} else {
				isSensorStopReached = false;
			}
			if((isSensorStopReached) || (leftSensorPositionRead >= moveStopCount)){
				msg("AT VELOCITY MOVE STOP==================================");
				if (_isCascadeMove) {
					msg("VEL MOVE TO POSITION DONE IN CASCADE MODE================");
					isVelMoveToPositionActive = false;	
					
				} else {
						// No ramp at end of move
						setDriveTrainRamp(0);
						
						// Apply power level in opposite direction to brake to reduce coast
						rightCmdLevel = -(Math.signum(_MoveToPositionIn) * SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue);
						leftCmdLevel = -(Math.signum(_MoveToPositionIn) * SRXDriveBaseCfg.kAutoLeftMoveStopBrakeValue);
						
					// Delay in sec
					if (!delay(1)) {
						msg("VELOCITY MOVE COMPLETE=======================");
						isVelMoveToPositionActive = false;
						//Stop motors
						rightCmdLevel = 0;
						leftCmdLevel = 0;
						
					}
				}
			}
			
		}
		
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		//+++++++++++++++++++++++
		// LOGGING AND DISPLAY
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt: %-8.0f ===LftPos:%-8.2f ===RgtPos:%-8.2f%n",
									moveStopCount,
									leftSensorPositionRead, 
									rightSensorPositionRead);		
		}
		return isVelMoveToPositionActive;
		
	}

	//====================================
	// ROTATE TO ANGLE
	//====================================
	public boolean rotateToAngle(double _rotateToAngle, double _rotatePowerLevel) {
		
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		
		if (!isRotateToAngleActive) {
			isRotateToAngleActive = true;
			msg("ROTATE TO ANGLE IS ACTIVE===========================");
			
			leftCmdLevel = Math.signum(_rotateToAngle) * _rotatePowerLevel;
			rightCmdLevel = -Math.signum(_rotateToAngle) * _rotatePowerLevel * SRXDriveBaseCfg.kRotateDriveStraightCorrection; 
			
			
			// rotationEncoderStopCount = C(=>PI*D) * (angle as a fraction of C)
			rotationEncoderStopCount = (Math.PI*(SRXDriveBaseCfg.kTrackWidthIn) * SRXDriveBaseCfg.kLeftEncoderCountsPerIn * (_rotateToAngle / 360))
			                                 - SRXDriveBaseCfg.kAutoRotateCoastToStopCounts;
			
		// use left encoder to mark rotation distance
		} else if (leftSensorPositionRead >= rotationEncoderStopCount){
				msg("ROTATE TO ANGLE IS AT STOP===========================");
				
				setDriveTrainRamp(0);
				// Apply power level in opposite direction to brake
				rightCmdLevel = (Math.signum(_rotateToAngle)*SRXDriveBaseCfg.kAutoRightRotateStopBrakeValue);
				leftCmdLevel = -(Math.signum(_rotateToAngle)*SRXDriveBaseCfg.kAutoLeftRotateStopBrakeValue);
			if (!delay(1)) {
				isRotateToAngleActive = false;
				msg("ROTATE TO ANGLE IS DONE=========================");
				rightCmdLevel = 0;
				leftCmdLevel = 0;
			}		
		}
		
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		//++++++++++++++++++++++++++++++++++++++++++++++
		// Display data
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.2f ===LftEnc:%-8.2f ===RgtEnc:%-8.2f%n",
									rotationEncoderStopCount,
									leftSensorPositionRead, 
									rightSensorPositionRead);
		}
		return isRotateToAngleActive;
	} 
	//===================================
	// TURN BY ENCODER TO ANGLE
	//===================================
	public boolean turnByEncoderToAngle(double _turnAngleDeg, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn ) {
		// read encoders
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		
		if (!isTurnToAngleActive) {
			isTurnToAngleActive = true;
			msg("TURN TO ANGLE ACTIVE===========================");
			
			// Calculations
			wheelToCenterDistanceIn = (SRXDriveBaseCfg.kTrackWidthIn / 2);
			
			// Check turn radius add 4 inches is less than 1/2 wheelbase
			if(_turnRadiusIn < wheelToCenterDistanceIn){
				_turnRadiusIn = wheelToCenterDistanceIn +4;
			}
			
			//radius is the distance from the center of the robot to a point outside the robot
			speedRatio =(_turnRadiusIn + wheelToCenterDistanceIn) / (_turnRadiusIn - wheelToCenterDistanceIn);
			
			// Determine which wheel has to speed up to turn
			if (_turnAngleDeg > 0) {
				rightCmdLevel = (_turnPowerLevel);
				leftCmdLevel = (_turnPowerLevel * speedRatio);
		
			} else {
				rightCmdLevel = (_turnPowerLevel * speedRatio);
				leftCmdLevel = (_turnPowerLevel);
			}
			
			// Convert turn distance in inches to encoder counts(2*PI*Radius)*(deg/360deg)*(cnts/in)
			if (_turnAngleDeg >= 0) {
				outerDistanceStopCnt = (2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) *	SRXDriveBaseCfg.kLeftEncoderCountsPerIn) + leftSensorPositionRead - SRXDriveBaseCfg.kAutoTurnCoastToStopCounts;
			} else {
				outerDistanceStopCnt = (2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) * SRXDriveBaseCfg.kRightEncoderCountsPerIn) + rightSensorPositionRead - SRXDriveBaseCfg.kAutoTurnCoastToStopCounts;
			}
			
		// Active state -  check for end of encoder count
		} else if (((_turnAngleDeg >= 0) && (leftSensorPositionRead > outerDistanceStopCnt))
					|| ((_turnAngleDeg <= 0) && (rightSensorPositionRead > outerDistanceStopCnt))) {
				msg("TURN TO ANGLE IS AT STOP===========================");
				if (_isCascadeTurn) {
					isTurnToAngleActive = false;
					
				} else {
					
				setDriveTrainRamp(0);	
				// Apply power level in opposite direction for 1 second to brake
				rightCmdLevel = -(Math.signum(_turnAngleDeg) * SRXDriveBaseCfg.kAutoRightTurnStopBrakeValue);
				leftCmdLevel = -(Math.signum(_turnAngleDeg) * SRXDriveBaseCfg.kAutoLeftTurnStopBrakeValue);
				if (!delay(1)) {
					isTurnToAngleActive = false;
					rightCmdLevel = 0;
					leftCmdLevel = 0;
				}
			}
		}
		
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		// +++++++++++++++++++++++++++++++++++++++
		// Display data
		if (isConsoleDataEnabled){
			System.out.printf("SpeedRatio: %-8.3f ===StopCnt: %-8.2f ===LftEnc: %-8.2f ===RgtEnc: %-8.2f%n",
									speedRatio,
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
	public double encoderAngleCorrection(double _leftPosition, double _rightPosition, boolean _isPIDActive){
		if(_isPIDActive){
			
			//headingRate = ((currentHeading - previousHeading) / timeElapsed[this is taken care of in Kd];
			EncoderHeadingRate = encoderHeadingDeg - previousEncoderHeadingDeg;
			
			// Determine present heading
			encoderHeadingDeg = (leftSensorPositionRead - rightSensorPositionRead) / SRXDriveBaseCfg.kTrackWidthIn;
			
			//headingError = heading set point(=0) - currentHeading, however we need to increase right drive to go left;
			encoderPIDCorrection = (Kp_encoderHeadingPID * encoderHeadingDeg) - (Kd_encoderHeadingPID * EncoderHeadingRate);
		} else {
			encoderHeadingDeg = 0;
			previousEncoderHeadingDeg = 0;
			encoderPIDCorrection = 0;
		}
		return encoderPIDCorrection;
	}
	
	public double capCorrection(double _sensorCorrection){
		
		// Determine a max correction based on throttle power level 
		// leftCmdLevel is a global variable and main throttle (TURN = 0 going straight
		maxSensorCorrection = Math.abs(leftCmdLevel) * SRXDriveBaseCfg.kThrottlePowerRatio;
		
		// Cap correction
		if(Math.abs(_sensorCorrection) > maxSensorCorrection) {
			_sensorCorrection = Math.signum(_sensorCorrection) * maxSensorCorrection;
		}
		return _sensorCorrection;
	}
	/**
	* =======================================================================================
	* SRXDriveBase TEST METHODS
	* =======================================================================================
	*/
	
	//==============================
	// TEST METHOD SELECTION
	//==============================
	public void testMethodSelection(){
		
		if(!isTestMethodSelectionActive){
			isTestMethodSelectionActive = true;
			
			
			Kp_encoderHeadingPID = SmartDashboard.getNumber("Kp_encoderHeadingPID:", Kp_encoderHeadingPID);
			Ki_encoderHeadingPID = SmartDashboard.getNumber("Ki_encoderHeadingPID:", Ki_encoderHeadingPID);
			Kd_encoderHeadingPID = SmartDashboard.getNumber("Kd_encoderHeadingPID:", Kd_encoderHeadingPID);
			
		} else {
			if(SmartDashboard.getBoolean("TestStepFunction:", false)){
				if(!isEcoderReset){
					msg("SHUFFLE START TEST STEP FUNCTION=============");
					isEcoderReset = true;
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				// testStepFunction(double _stepFunctionPower, double _stepFunctionTimeSec, boolean _isTestForRightDrive)
				if(!testStepFunction(.3, 5, false)){
					isTestMethodSelectionActive = false;
					isEcoderReset = false;
					SmartDashboard.putBoolean("TestStepFunction:", false);
					msg("SHUFFLE END TEST STEP FUNCTION=============");
				}	
			}
			
			if(SmartDashboard.getBoolean("DriveStraightCal:", false)){
				
				//SRXDriveBaseCfg.kDriveStraightCorrection = SmartDashboard.getNumber("DriveStraightCorrection:" , SRXDriveBaseCfg.kDriveStraightCorrection);
				correction = SmartDashboard.getNumber("DriveStraightCorrection:" , correction);
				if(!isEcoderReset){
					msg("SHUFFLE START DRIVE STRAIGHT CAL=============");
					isEcoderReset = true;
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				// testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel)
				if(!testDriveStraightCalibration(50.0, .3)){
					isTestMethodSelectionActive = false;
					isEcoderReset = false;
					SmartDashboard.putBoolean("DriveStraightCal:", false);
					msg("SHUFFLE END DRIVE STRAIGHT CAL============");
				}	
			}
			
			if(SmartDashboard.getBoolean("VelMoveToPosition:", false)){
				
				if(!isEcoderReset){
					msg("SHUFFLE START VEL MOVE TO POSITION=============");
					isEcoderReset = true;
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				// velMoveToPosition(double _MoveToPositionIn, double _MoveToPositionPwrLevel, boolean _isSensorStopUsed, boolean _isCascadeMove)
				if(!velMoveToPosition(50, .3, false)) {
					isTestMethodSelectionActive = false;
					isEcoderReset = false;
					SmartDashboard.putBoolean("VelMoveToPosition:", false);
					msg("SHUFFLE END VEL MOVE TO POSITION=============");
				}	
			}
			if(SmartDashboard.getBoolean("RotateToAngle:", false)){
				
				if(!isEcoderReset){
					msg("SHUFFLE START ROTATE TO ANGLE=============");
					isEcoderReset = true;
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				// rotateToAngle(double _rotateToAngle, double _rotatePowerLevel)
				if(!rotateToAngle(90, .3)) {
					isTestMethodSelectionActive = false;
					isEcoderReset = false;
					SmartDashboard.putBoolean("RotateToAngle:", false);
					msg("SHUFFLE END ROTATE TO ANGLE=============");
				}	
			}
			if(SmartDashboard.getBoolean("TurnToAngle:", false)){
				
				if(!isEcoderReset){
					msg("SHUFFLE START TURN TO ANGLE=============");
					isEcoderReset = true;
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				//turnByEncoderToAngle(double _turnAngleDeg, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn )
				if(!turnByEncoderToAngle(90, 25, .1, false, false)) {
					isTestMethodSelectionActive = false;
					isEcoderReset = false;
					SmartDashboard.putBoolean("TurnToAngle:", false);
					msg("SHUFFLE END TURN TO ANGLE=============");
				}	
			}
			if(SmartDashboard.getBoolean("CascadeTest:", false)){
				
				if(!isEcoderReset){
					msg("SHUFFLE START CASCADE TEST=============");
					isEcoderReset = true;
					autoCmdSequence = 1;
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				
				switch(autoCmdSequence){
					case 1:
						// move 10 inches
						msg("case 1");
						if (!velMoveToPosition(10, 0.2, true)) {
							System.out.println("Velmovedone");
							autoCmdSequence = 2;
							msg("next case 2");
						};
						break;
					case 2:
						// turn right 90 deg
						
						if(!turnByEncoderToAngle(90, 25, .1, false, true )){
							autoCmdSequence = 3;
							msg("next case 3");
						};
						break;
					case 3:
						// move 10 in
						
						if (!velMoveToPosition(10, 0.2, true)) {
							autoCmdSequence = 4;
							msg("next case 4");
						};
						break;
					case 4:
						// turn left 90 deg
						
						if(!turnByEncoderToAngle(-70, 25, .1, false, false )){
							isTestMethodSelectionActive = false;
							SmartDashboard.putBoolean("CascadeTest:", false);
							msg("SHUFFLE END CASCADE TEST=============");
							isEcoderReset = false;
						};
						break;	
					default:
						isTestMethodSelectionActive = false;
						SmartDashboard.putBoolean("CascadeTest:", false);
						msg("SHUFFLE END CASCADE TEST=============");
						isEcoderReset = false;
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
				msg("START TEST STEP FUNCTION ===============================");
				cycleCount = 0;
				stepFunctionSpeed = _stepFunctionPower * SRXDriveBaseCfg.MaxVel_VelNativeUnits;
				// (sec / 20ms) [iterative robot scan time]
				stepFunctionStopCount = (int)(_stepFunctionTimeSec / 0.02);
				msg("pwr: " + _stepFunctionPower + " count: " + cycleCount +"stop count: " + stepFunctionStopCount);
			
			} else if(cycleCount > stepFunctionStopCount) {
				msg("TEST STEP FUNCTION AT TIME STOP=======================");
				stepFunctionSpeed = 0;
				
				// Delay for a specified time to have motion stopped
				if(!delay(3)){
					isTestStepFunctionActive = false;
					msg("TEST STEP FUNCTION DONE=======================");
				}
			}
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
	// TEST DRIVE STRAIGHT CALIBRATION
	//===================================
	public boolean testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel){
		
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		
		if (!isTestMoveForStraightCalActive){
			msg("START CALIBARTION======================================");
			isTestMoveForStraightCalActive = true;
			
			leftEncoderStopCount = (_testDistanceIn / SRXDriveBaseCfg.kLeftInchesPerCount);
			
			leftCmdLevel = _pwrLevel;
			rightCmdLevel = (_pwrLevel * SRXDriveBaseCfg.kDriveStraightCorrection); 
			
			
			if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.0f+++LftEnc:%-8.0f +++RgtEnc:%-8.0f+++LftCmd:%-8.4f+++RgtCmd:%-8.4f%n", 
								leftEncoderStopCount, 
								leftSensorStartPositionRead, 
								rightSensorStartPositionRead,
								leftCmdLevel,
								rightCmdLevel);
			}
		
		// Test for stopping movement
		} else if (leftSensorPositionRead >= leftEncoderStopCount) {
			msg("CALIBRATION AT STOP ===========================================");
			
			setDriveTrainRamp(0);
			// Apply power level in opposite direction for 1 second to brake
			rightCmdLevel = -SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue;
			leftCmdLevel = -SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue;
			if (!delay(1)) {
				msg("CALIBRATION END ==================================");
				isTestMoveForStraightCalActive = false;
				rightCmdLevel = 0;
				leftCmdLevel = 0;
			}	
		}
		
		calCorrectionFactor = leftSensorPositionRead / rightSensorPositionRead;
		headingDeg = (leftSensorPositionRead - rightSensorPositionRead) / SRXDriveBaseCfg.kTrackWidthIn;
		
		// Output to SRX drive modules
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		//++++++++++++++++++++++++++++++++
		// LOGGING AND DISPLAY
		if (isLoggingDataEnabled) {
			String outputString = String.format("%8.0f,%8.0f,%8.0f,%8.4f,%8.4f", 
										leftEncoderStopCount, 
										leftSensorPositionRead, 
										rightSensorPositionRead,
										calCorrectionFactor,
										headingDeg);
			//Log data
			DebugLogger.data(outputString);
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
	
//==========================================
// AUTO-TUNE CORRECTION FACTOR -this sort of works
// =========================================
public boolean autoTuneCorrectionFactor(double _autoTunepowerLevel){
	
	Timer.delay(0.050);
	leftSensorPositionRead = getLeftSensorPosition();
	rightSensorPositionRead = getRightSensorPosition();
	
	if(!isAutoTuneCorrectionFactorActive){
			isAutoTuneCorrectionFactorActive = true;
			msg("STARTING AUTO TUNE CORRECTION FACTOR==========================");
			rightCmdLevel = _autoTunepowerLevel;
			leftCmdLevel = _autoTunepowerLevel;
			int state = 1;
			
	} else {
		switch (state){
		case 1: // start after there are some counts
			if (leftSensorPositionRead > 100){
				
				// establish first sample
				if(rightSensorPositionRead > leftSensorPositionRead){
					// Right to fast -> right needs to go slower
					isLastTuneDirectionPos = false;
				} else {
					// Right to slow -> right needs to go faster
					isLastTuneDirectionPos = true;
				}
				tuneCorrectionFactor = leftSensorPositionRead / rightSensorPositionRead;
				state = 2;
			}
			break;
		
		case 2: //tune 1st decimal digit
			state = tuneDecimalDigit(2, 0.1);
			break;
		case 3: //tune 2nd decimal digit
			state = tuneDecimalDigit(3, 0.01);
			break;
		case 4: //tune 2nd decimal digit
			state = tuneDecimalDigit(4, 0.001);
			break;
		case 5: //done
			isAutoTuneCorrectionFactorActive = false;
			msg("DriveStraightCorrectionFactor: " + tuneCorrectionFactor);
			break;	
		default:
			isAutoTuneCorrectionFactorActive = false;
			break;
		}	
			
	}

	driveRightMasterMtr.set(ControlMode.PercentOutput,rightCmdLevel);
	driveLeftMasterMtr.set(ControlMode.PercentOutput,leftCmdLevel);
	if (isConsoleDataEnabled){
		System.out.printf("tuneCorrectionFactor: %-8.3f%n",	tuneCorrectionFactor);
	}
	return isAutoTuneCorrectionFactorActive;
}
// assumes left/right encoder are still valid for auto tune
private int tuneDecimalDigit(int _state, double _decimalDigit){
	
	if(rightSensorPositionRead > leftSensorPositionRead){
			// Right to fast -> right needs to go slower 
			tuneCorrectionFactor -= _decimalDigit;
			isTuneDirectionPos = false;
		} else {
			// Right to slow -> right needs to go faster
			tuneCorrectionFactor += _decimalDigit;
			isTuneDirectionPos = true;
		}
		if (isTuneDirectionPos != isLastTuneDirectionPos) {
			// went to far - backup
			isTuneDirectionPos = isLastTuneDirectionPos;
			tuneCorrectionFactor = lastCorrectionFactor;
			// Move to next state and digit
			_state += 1;
		}
		rightCmdLevel *= tuneCorrectionFactor;
		lastCorrectionFactor = tuneCorrectionFactor;
		isLastTuneDirectionPos = isTuneDirectionPos;

		return _state;
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
	
	// This method performs a SRX magic motion command from user calculated values
	// User should note that the right drive distance needs to be corrected by kDriveStraightCorrection
	public boolean magicMove(int _rightCruiseVel, int _rightAccel, int _rightDistance, int _leftCruiseVel,
			int _leftAccel, int _leftDistance) {
		
		leftSensorStartPositionRead = getLeftSensorPosition();
		rightSensorStartPositionRead = getRightSensorPosition();
		
		if (!isSRXMagicMoveActive) {
			isSRXMagicMoveActive = true;
			
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

			_rightDistance = (int)(_rightDistance * SRXDriveBaseCfg.kDriveStraightCorrection);
			_leftDistance = (int)(_leftDistance);
		} else {
			if (leftSensorStartPositionRead >= _leftDistance) {
				isSRXMagicMoveActive = false;
				_rightDistance = 0;
				_leftDistance = 0;
			}
		}
		driveRightMasterMtr.set(ControlMode.MotionMagic, _rightDistance); 
		driveLeftMasterMtr.set(ControlMode.MotionMagic, _leftDistance);
		return isSRXMagicMoveActive;
	}

}