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
	
	//private int cycleCount = 1;
	private int SRXTimeoutValueMs = 10;
	
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
	private double Kp_encoderHeadingPID = 0.07;
	private double Ki_encoderHeadingPID = 0.005;
	private double Kd_encoderHeadingPID = 0;
	private double encoderHeading_IAccumMax = 10;
	private double Ki_PIDHeadingAccum = 0;
	private double encoderPIDHeadingOut =0;
	
	private double previousEncoderHeadingDeg = 0;
	private double encoderHeadingDeg = 0;
	private double EncoderHeadingCorrection = 0;
	
	// Calibrate parameters
	// TODO for smartDashBoard
//	private double CalParm_DriveStraightCorrection = SRXDriveBaseCfg.kDriveStraightCorrection;
//	private double CalParm_RobotCoastToStopCounts = SRXDriveBaseCfg.kRobotCoastToStopCounts;
//	private double CalParm_AutoStopBrakeValue = SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue;
//	private double CalParm_AutoRotateStopBrakeValue = SRXDriveBaseCfg.kAutoRotateStopBrakeValue;
//	private double CalParm_AutoTurnStopBrakeValue = SRXDriveBaseCfg.kAutoTurnStopBrakeValue;
	
	
	
	
	//  Program flow switches
	private boolean isConsoleDataEnabled = false;
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
		driveRightMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx, 15);
		// wait for the sensor position to updated
		while (driveRightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx) != 0){
			driveRightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		}
	}
	
	public void setLeftEncPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		driveLeftMasterMtr.getSensorCollection().setQuadraturePosition(0, 15);
	}

	public void setLeftSensorPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		driveLeftMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx, 15);
		// wait for the sensor position to updated
		while (driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx) != 0){
			driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		}
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
	public void setSRXDriveBaseDefaults(){
	//hpw os ot	
	}
	
	public void setProgramStateFlagsToFalse() {
		isVelMoveToPositionActive = false;
		isRotateToAngleActive = false;
		isTurnToAngleActive = false;
		isSRXMagicMoveActive = false;
		isLowTimeActive = false;
		isPulse_SqWaveFnctStartActive = false;
		isMovePerpendicularActive = false;
		isTestMoveForStraightCalActive = false;
		isDelayActive = false;
		
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
			return driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		} else{
			return -driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
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
	public double getDriveStraightAssistCorrection(){
		// at present time do not have a angle class to get angle correction
		// robotAngle.getAngleCorrection();
		return	0;
	}
	
	public boolean isDriveMoving() {
		if ((Math.abs(driveLeftMasterMtr.getSensorCollection().getQuadratureVelocity()) > 0.1) ||
			(Math.abs(driveRightMasterMtr.getSensorCollection().getQuadratureVelocity()) > 0.1))
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
	public void DisplayChangeParmeters() {
		SmartDashboard.putNumber("Right Correction Factor", SRXDriveBaseCfg.kDriveStraightCorrection);
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
			SmartDashboard.putNumber("BaseDrive-Speed Right ClosedLoopErr",
					driveRightMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx));
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
			driveStraightDirCorrection = getDriveStraightAssistCorrection();
			
			// Calculate cmd level in terms of PercentVbus; range (-1 to 1)
			leftCmdLevel = _throttleValue + _turnValue + driveStraightDirCorrection;
			rightCmdLevel = ((_throttleValue* SRXDriveBaseCfg.kDriveStraightCorrection) - _turnValue) - driveStraightDirCorrection;
		} else {
			leftCmdLevel = _throttleValue  +_turnValue;
			rightCmdLevel = ((_throttleValue * SRXDriveBaseCfg.kDriveStraightCorrection) - _turnValue);
		}
		//msg("TopRPM" + SRXDriveBaseCfg.kTopRPM + "Cycles Per Rev" + SRXDriveBaseCfg.kDriveEncoderCyclesPerRev + "CNT-PER-REV" + //SRXDriveBaseCfg.kCountsPerRevolution + "MaxRPM" +SRXDriveBaseCfg.MaxVel_VelNativeUnits);
		
		// Output commands to SRX modules set as [% from (-1 to 1)] x MaxVel_VelNativeUnits
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
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
	public boolean velMoveToPosition(double _MoveToPositionIn, double _MoveToPositionPwrLevel, boolean _isCascadeMove) {
		// This method moves the robot with a predetermined power level and stops at
		// the specified position value. The move will be in brake mode to stop
		// method will check that robot is stopped and set brake mode back to coast and respond
		// that move is done
		if (!isVelMoveToPositionActive) {
			isVelMoveToPositionActive = true;
			
			leftSensorPositionRead = getLeftSensorPosition();
			rightSensorPositionRead = getRightSensorPosition();
			
			msg("vel move to position active");
			
			// Move calculations
			moveStopCount = ((Math.abs(_MoveToPositionIn) * SRXDriveBaseCfg.kLeftEncoderCountsPerIn) 
							- SRXDriveBaseCfg.kRobotCoastToStopCounts) 
							+ leftSensorPositionRead;
			leftCmdLevel = (Math.signum(_MoveToPositionIn) * _MoveToPositionPwrLevel);
			rightCmdLevel = (Math.signum(_MoveToPositionIn) * _MoveToPositionPwrLevel) * SRXDriveBaseCfg.kDriveStraightCorrection;
			
			if (isConsoleDataEnabled){
					SmartDashboard.putNumber("Move/rightCmdLevel:", rightCmdLevel);
					SmartDashboard.putNumber("Move/leftCmdLevel:", leftCmdLevel);
					SmartDashboard.putNumber("Move/rightSensorPositionRead:", rightSensorPositionRead);
					SmartDashboard.putNumber("Move/leftSensorPositionRead:", leftSensorPositionRead);
					SmartDashboard.putNumber("Move/moveStopCount:", moveStopCount);
			}
			
		} else {
			// Check for move stop
			if (getLeftSensorPosition() >= moveStopCount) {
				if (_isCascadeMove) {
					isVelMoveToPositionActive = false;	
					msg("vel move to position done in cascade mode");
				} else {
						// No ramp at end of move
						setDriveTrainRamp(0);
						
						// Apply power level in opposite direction to brake
						rightCmdLevel = -(Math.signum(_MoveToPositionIn) * SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue);
						leftCmdLevel = -(Math.signum(_MoveToPositionIn) * SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue);
						
					// Delay in sec
					if (!delay(1)) {
						isVelMoveToPositionActive = false;
						//Stop motors
						rightCmdLevel = 0;
						leftCmdLevel = 0;
						msg("vel move to position done");
					}
				}
			}
			
		}
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt: %-8.0f ===LftPos:%-8.2f ===RgtPos:%-8.2f%n",
									moveStopCount,
									leftSensorPositionRead, 
									rightSensorPositionRead);
		}
		return isVelMoveToPositionActive;
		
	}
/*
====================== do not have ultrasonic sensors for this at this time
	public boolean movePerpendicularToStop(double _movePerpendicularPowerLevel, double _movePerpendicularStopIn) {
		if (!isMovePerpendicularActive) {
			isMovePerpendicularActive = true;
			rightCmdLevel = _movePerpendicularPowerLevel;
			leftCmdLevel = _movePerpendicularPowerLevel;	
		} else if (robotDistance.getPerpendicularStop(_movePerpendicularStopIn)){
			rightCmdLevel=0;
			leftCmdLevel=0;
			isMovePerpendicularActive=false;
		} else{
			//drivePerpendicularDirCorrection = robotDistance.getPerpendicularCorrection();
			leftCmdLevel += drivePerpendicularDirCorrection;
			rightCmdLevel -= drivePerpendicularDirCorrection;
		}
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled)	{
			rightCmdLevel *= SRXDriveBaseCfg.SRXDriveBaseCfg.MaxVel_VelNativeUnits;
			leftCmdLevel *= SRXDriveBaseCfg.SRXDriveBaseCfg.MaxVel_VelNativeUnits;
		}
		driveRightMasterMtr.set(rightCmdLevel);
		driveLeftMasterMtr.set(leftCmdLevel);
		return isMovePerpendicularActive;
	}
*/
	public boolean rotateToAngle(double _rotateToAngle, double _rotatePowerLevel) {
		// direction(true)-rotates right, direction(false)-rotates left
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		if (!isRotateToAngleActive) {
			isRotateToAngleActive = true;
			
			leftSensorPositionRead = getLeftSensorPosition();
			rightSensorPositionRead = getRightSensorPosition();
			
			msg("++Rotate to angle is active");
			
			rightCmdLevel = -Math.signum(_rotateToAngle) * _rotatePowerLevel; 
			leftCmdLevel = Math.signum(_rotateToAngle) * _rotatePowerLevel;
			
			// rotationEncoderStopCount = C(=>PI*D) * (angle as a fraction of C)
			rotationEncoderStopCount = (Math.PI*(SRXDriveBaseCfg.kTrackWidthIn) * SRXDriveBaseCfg.kLeftEncoderCountsPerIn * (_rotateToAngle / 360)) + leftSensorPositionRead;
			
			if (isConsoleDataEnabled){
					SmartDashboard.putNumber("Rotate/rightCmdLevel:", rightCmdLevel);
					SmartDashboard.putNumber("Rotate/leftCmdLevel:", leftCmdLevel);
					SmartDashboard.putNumber("Rotate/rightSensorPositionRead:", rightSensorPositionRead);
					SmartDashboard.putNumber("Rotate/leftSensorPositionRead:", leftSensorPositionRead);
					SmartDashboard.putNumber("Rotate/moveStopCount:", rotationEncoderStopCount);
				}
			
		// use left encoder to mark rotation distance
		} else if (leftSensorPositionRead >= rotationEncoderStopCount) {
			System.out.println(leftSensorPositionRead);
				setDriveTrainRamp(0);
				// Apply power level in opposite direction to brake
				rightCmdLevel = (Math.signum(_rotateToAngle)*SRXDriveBaseCfg.kAutoRightRotateStopBrakeValue);
				leftCmdLevel = -(Math.signum(_rotateToAngle)*SRXDriveBaseCfg.kAutoLeftRotateStopBrakeValue);
			if (!delay(1)) {
				isRotateToAngleActive = false;
				msg("++Rotate to angle is done");
				rightCmdLevel = 0;
				leftCmdLevel = 0;
			}		
		}
		//leftSensorPositionRead = getLeftSensorPosition();
		//rightSensorPositionRead = getRightSensorPosition();
		
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.2f ===LftEnc:%-8.2f ===RgtEnc:%-8.2f%n",
									rotationEncoderStopCount,
									leftSensorPositionRead, 
									rightSensorPositionRead);
		}
		return isRotateToAngleActive;
	} 

	public boolean turnByEncoderToAngle(double _turnAngleDeg, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn ) {
		boolean isStopingTurn = false;
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		if (!isTurnToAngleActive) {
			isTurnToAngleActive = true;
			isStopingTurn = false;
			
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
				outerDistanceStopCnt = (2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) *	SRXDriveBaseCfg.kLeftEncoderCountsPerIn) + leftSensorPositionRead;
			} else {
				outerDistanceStopCnt = (2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) * SRXDriveBaseCfg.kRightEncoderCountsPerIn) + rightSensorPositionRead;
			}
			if (isConsoleDataEnabled){
					SmartDashboard.putNumber("Turn/rightCmdLevel:", rightCmdLevel);
					SmartDashboard.putNumber("Turn/leftCmdLevel:", leftCmdLevel);
					SmartDashboard.putNumber("Turn/speedRatio:", speedRatio);
					SmartDashboard.putNumber("Turn/outerDistanceStopCnt:", outerDistanceStopCnt);
					SmartDashboard.putNumber("Turn/rightSensorPositionRead:", rightSensorPositionRead);
					SmartDashboard.putNumber("Turn/leftSensorPositionRead:", leftSensorPositionRead);
					SmartDashboard.putNumber("Turn/moveStopCount:", rotationEncoderStopCount);
				}
			
		// Active state -  check for end of encoder count
		} else if ((_turnAngleDeg >= 0 && (leftSensorPositionRead > outerDistanceStopCnt))
					|| (_turnAngleDeg <= 0 && (rightSensorPositionRead > outerDistanceStopCnt))) {
				isStopingTurn = true;
				if (_isCascadeTurn) {
					isTurnToAngleActive = false;
					
					msg("Cascade Active flag=> isTurnToAngleActive:" + isTurnToAngleActive);
					
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
		//leftSensorPositionRead = getLeftSensorPosition();
		//rightSensorPositionRead = getRightSensorPosition();
		
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		if (isConsoleDataEnabled){
			System.out.printf("SpeedRatio: %-8.3f ===StopCnt: %-8.2f ===LftEnc: %-8.2f ===RgtEnc: %-8.2f IsStoping: %b%n",
									speedRatio,
									outerDistanceStopCnt,
									leftSensorPositionRead, 
									rightSensorPositionRead,
									isStopingTurn);
		}
		return isTurnToAngleActive;
	}
	

	
public double EncoderHeadingPID(double _encoderHeadingError, double _encoderHeadingDerivative)
{
	Ki_PIDHeadingAccum += Ki_encoderHeadingPID * _encoderHeadingError;
	if(Ki_PIDHeadingAccum > encoderHeading_IAccumMax) Ki_PIDHeadingAccum = encoderHeading_IAccumMax;
	if(Ki_PIDHeadingAccum < -encoderHeading_IAccumMax) Ki_PIDHeadingAccum = -encoderHeading_IAccumMax;
	
	encoderPIDHeadingOut = (Kp_encoderHeadingPID * _encoderHeadingError) + Ki_PIDHeadingAccum - (Kd_encoderHeadingPID * _encoderHeadingDerivative);

	return encoderPIDHeadingOut;
}
	
	/**
	* =======================================================================================
	* SRXDriveBase TEST METHODS
	* =======================================================================================
	*/
	
	
	// This provides a pulse(low-High-low) and stays in lowpower. Need to stop motors or call constantly to produce a square wave
	public boolean testMotorPulse_SquareWave(double _pulseLowPower, double _pulseHighPower, double _pulseTimeSec, boolean _isTestForRightDrive) {
		if (!SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			
			// initialize and start at low speed
			if (!isPulse_SqWaveFnctStartActive) {
				isPulse_SqWaveFnctStartActive = true;
				
				msg("++start testMotorPulse_SquareWave");
				
				isLowTimeActive = true;
				pulSqStartTimeSec = Timer.getFPGATimestamp(); // seconds

			// 20ms latter start low power output
			} else {
				if(_pulseLowPower == 0 && _pulseHighPower == 0){
					isLowTimeActive = false;
					isPulse_SqWaveFnctStartActive = false;
					driveLeftMasterMtr.set(ControlMode.Velocity,0);
					driveRightMasterMtr.set(ControlMode.Velocity,0);
				} else {
					if (isLowTimeActive) {

						// Stay at a low speed for klowSQTime ms then switch to high power level
						sqWaveVel = _pulseLowPower * SRXDriveBaseCfg.MaxVel_VelNativeUnits;
						if (_isTestForRightDrive) {
							driveRightMasterMtr.set(ControlMode.Velocity, sqWaveVel);
							driveLeftMasterMtr.set(ControlMode.Velocity, 0);
						} else {
							driveLeftMasterMtr.set(ControlMode.Velocity, sqWaveVel);
							driveRightMasterMtr.set(ControlMode.Velocity, 0);
						}
						if ((Timer.getFPGATimestamp() - pulSqStartTimeSec) > _pulseTimeSec) {
							
							msg("++Low Time: " + Timer.getFPGATimestamp());
							
							// setup for high power output
							isLowTimeActive = false;
							pulSqStartTimeSec = Timer.getFPGATimestamp();
							sqWaveVel = _pulseHighPower * SRXDriveBaseCfg.MaxVel_VelNativeUnits;
						}
					} else {

						// Stay at a high power for kHighSQTime ms then switch to low power
						if (_isTestForRightDrive) {
							driveRightMasterMtr.set(ControlMode.Velocity, sqWaveVel);
							driveLeftMasterMtr.set(ControlMode.PercentOutput,0);
						} else {
							driveLeftMasterMtr.set(ControlMode.Velocity, sqWaveVel);
							driveRightMasterMtr.set(ControlMode.PercentOutput,0);
						}
						if ((Timer.getFPGATimestamp() - pulSqStartTimeSec) > _pulseTimeSec) {
						
							msg("++High time: " + Timer.getFPGATimestamp());
							
							// Set up for low power
							isPulse_SqWaveFnctStartActive = false;
							sqWaveVel = _pulseLowPower * SRXDriveBaseCfg.MaxVel_VelNativeUnits;
							if (_isTestForRightDrive) {
								driveRightMasterMtr.set(ControlMode.Velocity, sqWaveVel);
								driveLeftMasterMtr.set(ControlMode.Velocity, 0);
							} else {
								driveLeftMasterMtr.set(ControlMode.Velocity, sqWaveVel);
								driveRightMasterMtr.set(ControlMode.Velocity, 0);
							}
						}
					}
					if (isConsoleDataEnabled){
						System.out.printf("sqWaveVel:%-8.3f==RightVel:%-8.2f==RightErr:%-8.2f==LeftVel:%-8.2f==LeftErr:%-8.2f%n",
								sqWaveVel,
								getRightSensorVelocity(),
								getRightCloseLoopError(),
								getLeftSensorVelocity(),
								getLeftCloseLoopError());
					}
				}
			}	
		} else {
			// Reset method flags for next call to motorPulse_SquareWaveTest method
			isLowTimeActive = false;
			isPulse_SqWaveFnctStartActive = false;
			driveLeftMasterMtr.set(ControlMode.Velocity,0);
			driveRightMasterMtr.set(ControlMode.Velocity,0);
		}
		return isPulse_SqWaveFnctStartActive;
	}

	//future TODO
//	public boolean testDriveStraightWithEncoderHeadingCal(double _testDistanceIn, double _pwrLevel){
//			if (!isTestMoveForStraightCalActive){
//				isTestMoveForStraightCalActive = true;
//
//				leftSensorStartPositionRead = getLeftSensorPosition();
//				rightSensorStartPositionRead = getRightSensorPosition();
//
//				previousEncoderHeadingDeg = 0;
//				Ki_PIDHeadingAccum = 0;
//
//				leftEncoderStopCount = (_testDistanceIn / SRXDriveBaseCfg.kLeftInchesPerCount) + leftSensorStartPositionRead;
//				if(_pwrLevel < 0){
//					_pwrLevel = 0;
//				}
//				//leftCmdLevel = _pwrLevel;
//				//rightCmdLevel = (_pwrLevel * SRXDriveBaseCfg.kDriveStraightCorrection); 
//				
//				if (isConsoleDataEnabled){
//					System.out.printf("StopCnt: %-8.3f++LftEncStrt: %-8.2f==RgtEncStrt: %-8.2f==LeftVel:%-8.2f==LeftErr:%-8.2f%n",
//						leftEncoderStopCount,
//						leftSensorStartPositionRead,
//						rightSensorStartPositionRead,
//						getLeftSensorVelocity(),
//						getLeftCloseLoopError());
//				}
//			} else if (getLeftSensorPosition() >= leftEncoderStopCount) {
//				
//				// Apply power level in opposite direction for 1 second to brake
//				rightCmdLevel = -SRXDriveBaseCfg.kAutoMoveStopBrakeValue;
//				leftCmdLevel = -SRXDriveBaseCfg.kAutoMoveStopBrakeValue;
//				if (!delay(1)) {
//					isTestMoveForStraightCalActive = false;
//					rightCmdLevel = 0;
//					leftCmdLevel = 0;
//					
//				}	
//			}
//
//			leftSensorPositionRead = getLeftEncoderPosition();
//			rightSensorPositionRead = getRightEncoderPosition();
//			
//			calCorrectionFactor = (leftSensorPositionRead - leftSensorPositionRead) / (rightSensorPositionRead - rightSensorPositionRead);
//
//			encoderHeadingDeg = (leftSensorPositionRead - rightSensorPositionRead) / SRXDriveBaseCfg.kTrackWidthIn;
//
//			//headingRate = ((currentHeading - previousHeading) / timeElapsed[this is taken care of in Kd);
//			EncoderHeadingRate = encoderHeadingDeg - previousEncoderHeadingDeg;
//			
//			//headingError = heading(=0) - currentHeading;
//			EncoderHeadingCorrection = EncoderHeadingPID(-encoderHeadingDeg, EncoderHeadingRate);
//
//			if(isTestMoveForStraightCalActive){
//				leftCmdLevel =_pwrLevel;

//				// (+)heading needs RgtSpeed(+) and (-) Heading needs RgtSpeed(-)
//				rightCmdLevel = _pwrLevel - EncoderHeadingCorrection;
//				if(rightCmdLevel > 1){
//					rightCmdLevel = 1;
//				}
//				if(rightCmdLevel < 0){
//					rightCmdLevel = 0;
//				}
//			}		
//          SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
//
//			previousEncoderHeadingDeg = encoderHeadingDeg;
//			
//			if (isLoggingDataEnabled) {
//				String outputString = String.format("%8.0f,%8.4f,%8.0f,%8.0f,%8.4f", 
//											leftEncoderStopCount,
//											leftSensorPositionRead,
//											rightSensorPositionRead,
//											encoderHeadingDeg,
//											EncoderHeadingRate,
//											calCorrectionFactor);
//				//Log data
//				DebugLogger.data(outputString);
//			}
//			
//			//Print on console data
//			if (isConsoleDataEnabled){
//				System.out.printf("StopCnt: %-8.0f==LftPos: %-8.3f==RgtPos: %-8.0f==Heading: %-8.0f===HeadRate: %-8.3f==HeadCor: %-8.3f==CorFact:%-8.3f%n", 
//									leftEncoderStopCount,
//									leftSensorPositionRead,
//									rightSensorPositionRead,
//									encoderHeadingDeg,
//									EncoderHeadingRate,
//									encoderHeadingCorrection,
//									calCorrectionFactor);
//			}
//			return isTestMoveForStraightCalActive;
//			
//	}
		public boolean testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel){
			if (!isTestMoveForStraightCalActive){
				
				isTestMoveForStraightCalActive = true;
				
				msg("++start testDriveStraightCalibration");
				
				// Read position so reading is not cascaded to minimize delays
				leftSensorStartPositionRead = getLeftSensorPosition();
				rightSensorStartPositionRead = getRightSensorPosition();
				
				leftEncoderStopCount = (_testDistanceIn / SRXDriveBaseCfg.kLeftInchesPerCount);
				
				leftCmdLevel = _pwrLevel;
				rightCmdLevel = (_pwrLevel * SRXDriveBaseCfg.kDriveStraightCorrection); 
				
				//if (isConsoleDataEnabled){
					SmartDashboard.putNumber("Straight Cal/rightCmdLevel:", rightCmdLevel);
					SmartDashboard.putNumber("Straight Cal/leftCmdLevel:", leftCmdLevel);
					SmartDashboard.putNumber("Straight Cal/rightSensorPositionRead:", rightSensorStartPositionRead);
					SmartDashboard.putNumber("Straight Cal/leftSensorPositionRead:", leftSensorStartPositionRead);
					SmartDashboard.putNumber("Straight Cal/leftEncoderStopCount:", leftEncoderStopCount);
				//}
				if (isConsoleDataEnabled){
				System.out.printf("StopCnt:%-8.0f+++LftEnc:%-8.0f +++RgtEnc:%-8.0f+++LftCmd:%-8.4f+++RgtCmd:%-8.2f%n", 
									leftEncoderStopCount, 
									leftSensorStartPositionRead, 
									rightSensorStartPositionRead,
									leftCmdLevel,
									leftCmdLevel);
				}
			
			// Test for stopping movement
			} else if (getLeftSensorPosition() >= leftEncoderStopCount) {
				
				msg("++stop testDriveStraightCalibration");
				
				setDriveTrainRamp(0);
				// Apply power level in opposite direction for 1 second to brake
				rightCmdLevel = -SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue;
				leftCmdLevel = -SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue;
				if (!delay(1)) {
					
					msg("++end testDriveStraightCalibration");
					
					isTestMoveForStraightCalActive = false;
					rightCmdLevel = 0;
					leftCmdLevel = 0;
				}	
			}
			leftSensorPositionRead = getLeftSensorPosition();
			rightSensorPositionRead = getRightSensorPosition();
			
			calCorrectionFactor = leftSensorPositionRead / rightSensorPositionRead;
			headingDeg = (leftSensorPositionRead - rightSensorPositionRead) / SRXDriveBaseCfg.kTrackWidthIn;
			
			// Output to SRX drive modules
			SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
			
			
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
	public boolean magicMove(int _rightCruiseVel, int _rightAccel, int _rightDistance, int _leftCruiseVel,
			int _leftAccel, int _leftDistance) {
		// This method performs a SRX magic motion command from user calculated values
		// User should note that the right drive distance needs to be corrected by kDriveStraightCorrection
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
			if (getRightEncoderPosition() >= _leftDistance) {
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