package org.usfirst.frc.team2228.robot;
/**
* Class SRXBaseDrive
* RELEASE: 2, RevA 180117 
* Team 2228 / RJV
*
*
*/
/* ===================================
 * REVISIONS:
 * Release 1
 * RevA: original
 */

//Carrying over the classes from other libraries
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
//import com.ctre.phoenix.motorcontrol.*;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRXDriveBase {
	// public class SRXDriveBase( AngleIF _angle, DistanceIF _distance)
	// AngleIF robotAngle = _angle;
	// DistanceIF robotDistance = _distance;
	
	private String VersionString = "Release 1, RevC 180108";

	// cheesy or tank motors
	
	private WPI_TalonSRX driveRightMasterMtr;
	private WPI_TalonSRX driveRightFollowerMtr;
	private WPI_TalonSRX driveLeftMasterMtr;
	private WPI_TalonSRX driveLeftFollowerMtr;

	private DifferentialDrive driveStyle;
	
	private int cycleCount = 1;
	
	private double leftEncoderCounts = 0;
	private double rightDrvTrainTargetPosSetPt;
	private double leftDrvTrainTargetPosSetPt;
	private double leftCmdLevel = 0;
	private double rightCmdLevel = 0;
	private double rotationEncoderCount = 0;
	private double driveStraightDirCorrection = 0;
	private double speedRatio = 0;
	private double wheelToCenterDistanceIn = 0;
	private double outerDistanceCnts = 0;
	private double drivePerpendicularDirCorrection = 0;
	//private double integral = 0;
	//private double previousError = 0;
	//private double previousTime = 0;
	//private double previousTimeSec =0;
	private double calCorrectionFactor = 0;
	//private double startStallTimerSec = 0;
	private double moveCounts = 0;
	private double pulSqStartTimeSec = 0;
	private double delayStartTime = 0;
	private double driveStraightEncoderError = 0;
	
	//  Program flow switches
	private boolean isStallTimerActive = false;
	private boolean isStallTimerTimedOut = false;
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
	private boolean isConsoleDataEnabled = false;
	private boolean isLoggingDataEnabled = false;
	private boolean islogSRXDriveActive = false;
	private boolean isAntiSkidBrakeToggle = false;
	private boolean isTestMoveForStraightActive = false;
	
	private String logSRXDriveString = " ";
	private String lastMsgString = " ";
	
	// SRXDriveBase Class Constructor
	public SRXDriveBase() {
	
		
		// Create CAN SRX motor controller objects
		driveRightMasterMtr = new WPI_TalonSRX(RobotMap.CAN_ID_1);
		driveRightFollowerMtr = new WPI_TalonSRX(RobotMap.CAN_ID_2);
		driveLeftMasterMtr = new WPI_TalonSRX(RobotMap.CAN_ID_3);
		driveLeftFollowerMtr = new WPI_TalonSRX(RobotMap.CAN_ID_4);

		msg("Created Drives");
//
//		LiveWindow.addActuator("rtM", "RightMaster", driveRightMasterMtr);
//		LiveWindow.addActuator("rtF", "RightFollower", driveRightFollowerMtr);
//		LiveWindow.addActuator("lftM", "LeftMaster", driveLeftMasterMtr);
//		LiveWindow.addActuator("lftF", "LeftFollower", driveLeftFollowerMtr);


		/*
		 * Set right/left masters and right/left followers
		 */
		// Set Right master to percentVbus mode
		driveRightMasterMtr.set(ControlMode.PercentOutput,0);

		// Set up right follower
		driveRightFollowerMtr.set(ControlMode.Follower, driveRightMasterMtr.getDeviceID());
		//driveRightFollowerMtr.enableControl();

		// Set left master to percentVbus modeSRXDrive
		driveLeftMasterMtr.set(ControlMode.PercentOutput,0);

		// Set up left follower
		driveLeftFollowerMtr.set(ControlMode.Follower, driveLeftMasterMtr.getDeviceID());
	
		// Invert SRX output to motors if necessary
		driveRightMasterMtr.setInverted(SRXDriveBaseCfg.isDriveRightMasterMtrReversed);
		driveRightFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveRightFollowerMtrReversed);
		
		driveLeftMasterMtr.setInverted(SRXDriveBaseCfg.isDriveLeftMasterMtrReversed);
		driveLeftFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveLeftFollowerMtrReversed);

		// Set peak and nominal output voltage levels of motor controllers
		driveRightMasterMtr.configNominalOutputForward(0.0, SRXDriveBaseCfg.kTimeoutMs);
		driveRightMasterMtr.configNominalOutputReverse(0.0, SRXDriveBaseCfg.kTimeoutMs);
		
		driveRightMasterMtr.configPeakOutputForward(1, SRXDriveBaseCfg.kTimeoutMs);
		driveRightMasterMtr.configPeakOutputReverse(-1, SRXDriveBaseCfg.kTimeoutMs);
		
		driveRightFollowerMtr.configNominalOutputForward(0.0, SRXDriveBaseCfg.kTimeoutMs);
		driveRightFollowerMtr.configNominalOutputReverse(0.0, SRXDriveBaseCfg.kTimeoutMs);
		
		driveRightFollowerMtr.configPeakOutputForward(1, SRXDriveBaseCfg.kTimeoutMs);
		driveRightFollowerMtr.configPeakOutputReverse(-1, SRXDriveBaseCfg.kTimeoutMs);
	
	    driveLeftMasterMtr.configNominalOutputForward(0.0, SRXDriveBaseCfg.kTimeoutMs);
		driveLeftMasterMtr.configNominalOutputReverse(0.0, SRXDriveBaseCfg.kTimeoutMs);
	
	    driveLeftMasterMtr.configPeakOutputForward(1, SRXDriveBaseCfg.kTimeoutMs);
		driveLeftMasterMtr.configPeakOutputReverse(-1, SRXDriveBaseCfg.kTimeoutMs);
		
		driveLeftFollowerMtr.configNominalOutputForward(0.0, SRXDriveBaseCfg.kTimeoutMs);
		driveLeftFollowerMtr.configNominalOutputReverse(0.0, SRXDriveBaseCfg.kTimeoutMs);
		
		driveLeftFollowerMtr.configPeakOutputForward(1, SRXDriveBaseCfg.kTimeoutMs);
		driveLeftFollowerMtr.configPeakOutputReverse(-1, SRXDriveBaseCfg.kTimeoutMs);
		
		msg("Set voltage levels");
		/*
		 * Set Brake-Coast mode to coast
		 */
		setBrakeMode(SRXDriveBaseCfg.isBrakeEnabled);

		/*
		 * Setup encoder feedback if there are encoders on master motors
		 */
		if (SRXDriveBaseCfg.isMasterEncodersPresent) {
			driveRightMasterMtr.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kTimeoutMs);
			driveRightMasterMtr.setSensorPhase(SRXDriveBaseCfg.isRightEncoderSensorReversed);

			driveLeftMasterMtr.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kTimeoutMs);
			driveLeftMasterMtr.setSensorPhase(SRXDriveBaseCfg.isLeftEncoderSensorReversed);
		}

		/*
		 * Setup closed-loop velocity and Setup PID values if enabled
		 */
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			setSRXSpeedModeWithFeedback();
		}
		
		/*
		* configure "full" output will scale to 11 volts
		*/
		driveRightMasterMtr.configVoltageCompSaturation(11.0, SRXDriveBaseCfg.kTimeoutMs);
		driveRightMasterMtr.enableVoltageCompensation(true); // turn on the feature
		// tweak the voltage bus measurement filter, default is 32 cells in rolling average (1ms per sample)
		driveRightMasterMtr.configVoltageMeasurementFilter(32, SRXDriveBaseCfg.kTimeoutMs);
		
		driveLeftMasterMtr.configVoltageCompSaturation(11.0, SRXDriveBaseCfg.kTimeoutMs);
		driveLeftMasterMtr.enableVoltageCompensation(true); // turn on the feature
		// tweak the voltage bus measurement filter, default is 32 cells in rolling average (1ms per sample)
		driveLeftMasterMtr.configVoltageMeasurementFilter(32, SRXDriveBaseCfg.kTimeoutMs);
		
		/*
		* set output zero (neutral) deadband at 4%
		*/
		driveLeftMasterMtr.configNeutralDeadband(0.04, SRXDriveBaseCfg.kTimeoutMs);
		driveRightMasterMtr.configNeutralDeadband(0.04, SRXDriveBaseCfg.kTimeoutMs);
		
		/*
		 * Clear all sticky faults in drive controllers
		 */
		driveRightMasterMtr.clearStickyFaults(SRXDriveBaseCfg.kTimeoutMs);
		driveRightFollowerMtr.clearStickyFaults(SRXDriveBaseCfg.kTimeoutMs);
		driveLeftMasterMtr.clearStickyFaults(SRXDriveBaseCfg.kTimeoutMs);
		driveLeftFollowerMtr.clearStickyFaults(SRXDriveBaseCfg.kTimeoutMs);
		
		/*
		* Set up stall conditions in SRX for the drive train
		*/
		driveRightMasterMtr.configPeakCurrentLimit(SRXDriveBaseCfg.kStallCurrentPeakAmps, SRXDriveBaseCfg.kTimeoutMs);
		driveRightMasterMtr.configPeakCurrentDuration(SRXDriveBaseCfg.kStallTimeMs, SRXDriveBaseCfg.kTimeoutMs);
		driveRightMasterMtr.configContinuousCurrentLimit(SRXDriveBaseCfg.kStallCurrentContinuousAmps, SRXDriveBaseCfg.kTimeoutMs);
		driveRightMasterMtr.enableCurrentLimit(true);
	
		driveLeftMasterMtr.configPeakCurrentLimit(SRXDriveBaseCfg.kStallCurrentPeakAmps, SRXDriveBaseCfg.kTimeoutMs);
		driveLeftMasterMtr.configPeakCurrentDuration(SRXDriveBaseCfg.kStallTimeMs, SRXDriveBaseCfg.kTimeoutMs);
		driveLeftMasterMtr.configContinuousCurrentLimit(SRXDriveBaseCfg.kStallCurrentContinuousAmps, SRXDriveBaseCfg.kTimeoutMs);
		driveLeftMasterMtr.enableCurrentLimit(true);
		
		/*
		* Configure the velocity measurement period and sample window
		* Sample period in ms from supported sample periods-default 100ms period/64 sample window
		*/
		driveRightMasterMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms,SRXDriveBaseCfg.kTimeoutMs);
		driveLeftMasterMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms,SRXDriveBaseCfg.kTimeoutMs);

		// Number if samples in a rolling average
		driveRightMasterMtr.configVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample,SRXDriveBaseCfg.kTimeoutMs);
		driveLeftMasterMtr.configVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample,SRXDriveBaseCfg.kTimeoutMs);

		// Activate open-Loop velocity
		driveRightMasterMtr.set(ControlMode.PercentOutput,0);
		driveLeftMasterMtr.set(ControlMode.PercentOutput,0);
		msg("Configured drives");
		/*
		*  Create drive for WPI arcade usage
		*/
		driveStyle = new DifferentialDrive(driveLeftMasterMtr, driveRightMasterMtr) ;
		driveStyle.setSafetyEnabled(false);
	
	}
	/**
	* =======================================================================================
	* SRXBaseDrive SET/CONFIG METHODS
	* =======================================================================================
	*/
	
	public void setSRXPercentVbusMode() {
		// Set Right master to percentVbus mode
		driveRightMasterMtr.set(ControlMode.PercentOutput, 0);
		//driveRightMasterMtr.enableControl();
		driveRightMasterMtr.set(ControlMode.PercentOutput, 0);

		// Set left master to percentVbus mode
		driveLeftMasterMtr.set(ControlMode.PercentOutput, 0);
		//driveLeftMasterMtr.enableControl();
		driveLeftMasterMtr.set(ControlMode.PercentOutput,0);
	}

	public void setSRXSpeedModeWithFeedback() {
		driveRightMasterMtr.config_kF(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrFeedForwardGain, SRXDriveBaseCfg.kTimeoutMs);
        driveRightMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrProportionalGain, SRXDriveBaseCfg.kTimeoutMs);
        driveRightMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrIntegralGain, SRXDriveBaseCfg.kTimeoutMs); 
        driveRightMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrDerivativeGain, SRXDriveBaseCfg.kTimeoutMs);
		driveRightMasterMtr.config_IntegralZone(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrIzone, SRXDriveBaseCfg.kTimeoutMs);
		
		driveLeftMasterMtr.config_kF(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrFeedForwardGain, SRXDriveBaseCfg.kTimeoutMs);
        driveLeftMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrProportionalGain, SRXDriveBaseCfg.kTimeoutMs);
        driveLeftMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrIntegralGain, SRXDriveBaseCfg.kTimeoutMs); 
        driveLeftMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrDerivativeGain, SRXDriveBaseCfg.kTimeoutMs);
		driveLeftMasterMtr.config_IntegralZone(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveleftMstrIzone, SRXDriveBaseCfg.kTimeoutMs);
		
		// The following compensates for battery voltage - 50% output would be
		// %50 of 11 volts
		// driveRightMasterMtr.setNominalClosedLoopVoltage(11.0);
		// cnts / 4096 cnts/rev(magnetic encoder); 100/4096= 2.4%; 8.7 degrees
		//should be configure allowable closed loop error not get
		driveRightMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kClosedLoopErr);
		driveLeftMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kClosedLoopErr);
		
		// Sample period in ms from supported sample periods-default 100ms
		// period/64 sample window
		driveRightMasterMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms, 0);
		driveLeftMasterMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms, 0);

		// Number if samples in a rolling average
		driveRightMasterMtr.configVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample, 0);
		driveLeftMasterMtr.configVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample, 0);

		// set controlMode to speed with feedback
		// Note: velocity is in counts per VelocityMeasurementPeriod  (default is 100ms)
			driveRightMasterMtr.set(ControlMode.Velocity, 0);
			driveLeftMasterMtr.set(ControlMode.Velocity, 0);	
	}

	public void setRightEncPositionToZero() {
		driveRightMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kTimeoutMs);
	}

	public void setLeftEncPositionToZero() {
		driveLeftMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx,SRXDriveBaseCfg.kTimeoutMs);

	}
//	
//	public void setRightPositionToZero() {
//		driveRightMasterMtr.setPosition(0);
//	}
//
//	public void setLeftPositionToZero() {
//		driveLeftMasterMtr.setPosition(0);
//
//	}

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
	
	public void setClearActionFlags() {
		isStallTimerActive = false;
		isStallTimerTimedOut = false;
		isVelMoveToPositionActive = false;
		isRotateToAngleActive = false;
		isTurnToAngleActive = false;
		isSRXMagicMoveActive = false;
		isLowTimeActive = false;
		isPulse_SqWaveFnctStartActive = false;
		isMovePerpendicularActive = false;
		isTestMoveForStraightCalActive = false;
		isDelayActive = false;
		isDriveTrainMoving = false;
		islogSRXDriveActive = false;
		isAntiSkidBrakeToggle = false;
		isTestMoveForStraightActive = false;
	}
	public void setDriveTrainOpenLoopRamping (double _SecToMaxPower, boolean _isOpenLoopRampingEnabled){
		if(_isOpenLoopRampingEnabled){
			// Configure open loop ramp rate of 0.5 seconds to max power
			driveRightMasterMtr.configOpenloopRamp(_SecToMaxPower, SRXDriveBaseCfg.kTimeoutMs);
			driveLeftMasterMtr.configOpenloopRamp(_SecToMaxPower, SRXDriveBaseCfg.kTimeoutMs);
		} else {
			driveRightMasterMtr.configOpenloopRamp(0.0, SRXDriveBaseCfg.kTimeoutMs);
			driveLeftMasterMtr.configOpenloopRamp(0.0, SRXDriveBaseCfg.kTimeoutMs);
		}
	}
	


	/**
	* =======================================================================================
	* SRXBaseDrive GET METHODS
	* =======================================================================================
	*/
	
	// public double getMotorOutputVoltage()
	
	public double getRawRightEncoder(){
		return driveRightMasterMtr.getSensorCollection().getQuadraturePosition();
	}
	
	public double getRightEncoderPosition() {
		if (SRXDriveBaseCfg.isRightEncoderSensorReversed){
			return -driveRightMasterMtr.getSensorCollection().getQuadraturePosition();
		} else{
			return driveRightMasterMtr.getSensorCollection().getQuadraturePosition();
		}
	}
	
	public double getRightEncoderVelocity(){
		return driveRightMasterMtr.getSensorCollection().getQuadratureVelocity();
	}
	
	public double getRightClosedLoopPosition(){
		return driveRightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	public double getRightMstrMtrCurrent() {
		return driveRightMasterMtr.getOutputCurrent();
	}

	public double getRightFollowerMtrCurrent() {
		return driveRightFollowerMtr.getOutputCurrent();
	}

	public double getRightClosedLoopVelocity() {
		return driveRightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx);
	}

	public double getRightCloseLoopError() {
		return driveRightMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	// ==============================Left master motor
	public double getRawLeftEncoder() {
		return driveLeftMasterMtr.getSensorCollection().getQuadraturePosition();
	}
	
	public double getLeftEncoderVelocity(){
		return driveLeftMasterMtr.getSensorCollection().getQuadratureVelocity();
	}
	
	public double getLeftEncoderPosition() {
		if (SRXDriveBaseCfg.isLeftEncoderSensorReversed){
			return -driveLeftMasterMtr.getSensorCollection().getQuadraturePosition();
		} else{
			return driveLeftMasterMtr.getSensorCollection().getQuadraturePosition();
		}
	}
	
	public double getLeftClosedLoopPosition(){
		return driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	public double getLeftMstrMtrCurrent() {
		return driveLeftMasterMtr.getOutputCurrent();
	}
	
	public double getLeftFollowerMtrCurrent() {
		return driveLeftFollowerMtr.getOutputCurrent();
	}
	
	public double getLeftClosedLoopVelocity() {
		return driveLeftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx);
	}

	public double getLeftCloseLoopError() {
		return driveLeftMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx);
	}

	public double getBusVoltage() {
		return driveLeftMasterMtr.getBusVoltage();
	}
	
	public boolean isDriveMoving() {
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

		// Display SRXBaseDrive version
		SmartDashboard.putString("SRXBaseDrive-Version", VersionString);
		// Display SRX module values
		SmartDashboard.putNumber("BaseDrive-Right Bus Voltage", driveRightMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("BaseDrive-Right Output Voltage", driveRightMasterMtr.getMotorOutputVoltage());
		SmartDashboard.putNumber("BaseDrive-Current Right Master", driveRightMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive-Current Right Follower", driveRightFollowerMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive-Left Bus Voltage", driveLeftMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("BaseDrive-Left Output Voltage", driveLeftMasterMtr.getMotorOutputVoltage());
		SmartDashboard.putNumber("BaseDrive-Current Left Master", driveLeftMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive-Current Left Follower", driveRightFollowerMtr.getOutputCurrent());

		if (SRXDriveBaseCfg.isMasterEncodersPresent) {
			SmartDashboard.putNumber("BaseDrive-Right Encoder Count", driveRightMasterMtr.getSensorCollection().getQuadraturePosition());
			SmartDashboard.putNumber("BaseDrive-Right Position", driveRightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Right Speed ", driveRightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Left Encoder Count", driveLeftMasterMtr.getSensorCollection().getQuadraturePosition());
			SmartDashboard.putNumber("BaseDrive-Left Position", driveLeftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Left Speed ", driveLeftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx));
			
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
				logSRXDriveString = "Right Bus Voltage,Right Output Voltage,Right Master Current,Right Follower Current,Left Bus Voltage,Left Output Voltage,Left Master Current,Left Follower Current,Right Encoder Count,Left Encoder Count";
				// Log data
				DebugLogger.data(logSRXDriveString);
			} else {
				logSRXDriveString = String.format("%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f", 
										driveRightMasterMtr.getBusVoltage(), 
										driveRightMasterMtr.getMotorOutputVoltage(),
										driveRightMasterMtr.getOutputCurrent(),
										driveRightFollowerMtr.getOutputCurrent(),
										driveRightFollowerMtr.getOutputCurrent(),
										driveLeftMasterMtr.getBusVoltage(),
										driveLeftMasterMtr.getMotorOutputVoltage(),
										driveLeftMasterMtr.getOutputCurrent(),
										driveRightMasterMtr.getSensorCollection().getQuadraturePosition(),
										driveLeftMasterMtr.getSensorCollection().getQuadraturePosition() );
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
	 * NOTE: Motion command with open loop reflect power levels (-1 to 1) * (the
	 * motor bus voltage). Motion command with closed loop reflect speed level
	 * (-1 to 1) * (top motor RPM)
	 */
	public void SetDriveTrainCmdLevel(double _rightCMDLevel, double _leftCMDLevel) {
		rightCmdLevel = _rightCMDLevel;
		leftCmdLevel = _leftCMDLevel;
		isDriveTrainMoving =((rightCmdLevel > 0) || (leftCmdLevel > 0))? true : false;
		rightCmdLevel *= SRXDriveBaseCfg.kDriveStraightCorrection;
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			driveRightMasterMtr.set(ControlMode.Velocity, (rightCmdLevel*SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
			driveLeftMasterMtr.set(ControlMode.Velocity, (leftCmdLevel*SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
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
		isDriveTrainMoving =((_WPIThrottleValue > 0) || (_WPITurnValue > 0))? true : false;
		// deadband joystick in hands off condition
		_WPIThrottleValue = (Math.abs(_WPIThrottleValue) < SRXDriveBaseCfg.kNeutralDeadBand)? 0: _WPIThrottleValue;
		_WPITurnValue = (Math.abs(_WPITurnValue) < SRXDriveBaseCfg.kNeutralDeadBand)? 0: _WPITurnValue;
		// parms: move, turn, squared inputs
		driveStyle.arcadeDrive(_WPIThrottleValue, _WPITurnValue, false);	
	}

	/*
	 * setThrottleTurn is both open loop and closed loop control with drive
	 * straight/drive perpendicular correction
	 */
	public void setThrottleTurn(double _throttleValue, double _turnValue, boolean _isDrivingPerpendicular) {
		// declare drive moving if joysticks are moved
		isDriveTrainMoving =((_throttleValue > 0) || (_turnValue > 0))? true : false;
		// deadband joystick in hands off condition
		_throttleValue = (Math.abs(_throttleValue) < SRXDriveBaseCfg.kNeutralDeadBand)? 0: _throttleValue;
		_turnValue = (Math.abs(_turnValue) < SRXDriveBaseCfg.kNeutralDeadBand)? 0: _turnValue;

		if (SRXDriveBaseCfg.isDriveStraightAssistEnabled && _turnValue == 0) {

			// at present time do not have a angle class to get angle correction
			// driveStraightDirCorrection = robotAngle.getAngleCorrection();
			driveStraightDirCorrection = 0;
			
			// Calculate cmd level in terms of PercentVbus; range (-1 to 1)
			leftCmdLevel = _throttleValue + _turnValue + driveStraightDirCorrection;
			rightCmdLevel = ((_throttleValue* SRXDriveBaseCfg.kDriveStraightCorrection) - _turnValue) - driveStraightDirCorrection;

		} else {
			leftCmdLevel = _throttleValue  +_turnValue;
			rightCmdLevel = ((_throttleValue * SRXDriveBaseCfg.kDriveStraightCorrection) - _turnValue);
		}
//		if (Math.abs(_throttleValue) < .3 && Math.abs(_throttleValue) > .1){
//			if(isAntiSkidBrakeToggle){
//				rightCmdLevel = -(Math.signum(_throttleValue)*(1- Math.abs(_throttleValue)) * SRXDriveBaseCfg.kStopBrakeValue);
//				leftCmdLevel = -(Math.signum(_throttleValue)*(1- Math.abs(_throttleValue)) * SRXDriveBaseCfg.kStopBrakeValue);
//			}
//		}
//		isAntiSkidBrakeToggle = !isAntiSkidBrakeToggle;
//		
		// Output commands to SRX modules
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			driveRightMasterMtr.set(ControlMode.Velocity, (rightCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
			driveLeftMasterMtr.set(ControlMode.Velocity, (leftCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
		} else {
			driveRightMasterMtr.set(ControlMode.PercentOutput, rightCmdLevel);
			driveLeftMasterMtr.set(ControlMode.PercentOutput, leftCmdLevel);
		}
		if (isConsoleDataEnabled){
			System.out.printf("Throttle:%-8.3f===Turn:%-8.3f===RightCmd:%-8.3f===LeftCmd:%-8.3f===RightCurrent:%-8.3f===LeftCurrent:%-8.3f%n", 
									_throttleValue, 
									_turnValue,
									rightCmdLevel,
									leftCmdLevel,
									getRightMstrMtrCurrent(),
									getLeftMstrMtrCurrent());
		}
	}
	
	/*
	 * If robot blocked by another robot Fast turn turns robot 180 deg to escape
	 */
	public void setFastTurn(boolean _isFastTurnRight) {
		// future TODO
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
			isDriveTrainMoving = true;
			setRightEncPositionToZero();
			setLeftEncPositionToZero();
			setBrakeMode(true);
			msg("vel move to position active");
			moveCounts = (Math.abs(_MoveToPositionIn) * SRXDriveBaseCfg.kLeftEncoderCountsPerIn)
							- SRXDriveBaseCfg.kRobotCoastToStopCounts;
			leftCmdLevel = (Math.signum(_MoveToPositionIn)*_MoveToPositionPwrLevel);
			rightCmdLevel = (Math.signum(_MoveToPositionIn)*_MoveToPositionPwrLevel) * SRXDriveBaseCfg.kDriveStraightCorrection;
			msg("velMoveToPosition=> Move Counts:" + moveCounts +  " left/right Command:" + leftCmdLevel + " " + rightCmdLevel);
		} else {
			if (getLeftEncoderPosition() >= moveCounts) {
				if (_isCascadeMove) {
					isVelMoveToPositionActive = false;	
					msg("vel move to position done in cascade mode");
				} else {
						// Apply power level in opposite direction to brake
						rightCmdLevel = -(Math.signum(_MoveToPositionIn) * SRXDriveBaseCfg.kStopBrakeValue);
						leftCmdLevel = -(Math.signum(_MoveToPositionIn) * SRXDriveBaseCfg.kStopBrakeValue);
					if (!delay(1)) {
						isVelMoveToPositionActive = false;
						isDriveTrainMoving = false;
						setBrakeMode(false);
						rightCmdLevel = 0;
						leftCmdLevel = 0;
						msg("vel move to position done");
					}
				}
			}
			
		}
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			driveRightMasterMtr.set(ControlMode.Velocity, (rightCmdLevel*SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
			driveLeftMasterMtr.set(ControlMode.Velocity, (leftCmdLevel*SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
		} else {
			driveRightMasterMtr.set(ControlMode.PercentOutput,rightCmdLevel);
			driveLeftMasterMtr.set(ControlMode.PercentOutput,leftCmdLevel);
		}
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.0f===LftEnc:%-8.0f ===RgtEnc:%-8.0f%n",
									moveCounts,
									getLeftEncoderPosition(), 
									getRightEncoderPosition());
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
		if (!isRotateToAngleActive) {
			isRotateToAngleActive = true;
			isDriveTrainMoving = true;
			setRightEncPositionToZero();
			setLeftEncPositionToZero();
			msg("Rotate to angle is active");
			rightCmdLevel = -Math.signum(_rotateToAngle) * _rotatePowerLevel; 
			leftCmdLevel = Math.signum(_rotateToAngle) * _rotatePowerLevel;
			// rotationEncoderCount = C(=>PI*D) * (angle as a fraction of C)
			rotationEncoderCount = Math.PI*(SRXDriveBaseCfg.kTrackWidthIn) * SRXDriveBaseCfg.kLeftEncoderCountsPerIn * (_rotateToAngle / 360);
			msg("rotation count" + rotationEncoderCount + " left/right command level:" + leftCmdLevel + " " + rightCmdLevel );
		// use left encoder to mark rotation distance
		} else if (getLeftEncoderPosition() >= rotationEncoderCount) {
	
			// Apply power level in opposite direction to brake
				rightCmdLevel = (Math.signum(_rotateToAngle)*SRXDriveBaseCfg.kStopBrakeValue);
				leftCmdLevel = -(Math.signum(_rotateToAngle)*SRXDriveBaseCfg.kStopBrakeValue);
			if (!delay(1)) {
				isRotateToAngleActive = false;
				isDriveTrainMoving = false;
				setBrakeMode(false);
				rightCmdLevel = 0;
				leftCmdLevel = 0;
				msg("rotate to angle complete after brake applied");
			}		
		}
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			driveRightMasterMtr.set(ControlMode.Velocity, (rightCmdLevel*SRXDriveBaseCfg.MaxVel_VelNativeUnits));
			driveLeftMasterMtr.set(ControlMode.Velocity, (leftCmdLevel*SRXDriveBaseCfg.MaxVel_VelNativeUnits));
		} else {
			driveRightMasterMtr.set(ControlMode.PercentOutput,rightCmdLevel);
			driveLeftMasterMtr.set(ControlMode.PercentOutput,leftCmdLevel);
		}
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.0f===LftEnc:%-8.0f ===RgtEnc:%-8.0f%n",
									rotationEncoderCount,
									getLeftEncoderPosition(), 
									getRightEncoderPosition());
		}
		return isRotateToAngleActive;
	} 

	public boolean turnByEncoderToAngle(double _turnAngleDeg, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn ) {
		if (!isTurnToAngleActive) {
			isTurnToAngleActive = true;
			isDriveTrainMoving = true;
			msg(" Turn by encoder to angle is active");
			setRightEncPositionToZero();
			setLeftEncPositionToZero();
			// Calculations
			wheelToCenterDistanceIn = (SRXDriveBaseCfg.kTrackWidthIn / 2);
			speedRatio =(_turnRadiusIn + wheelToCenterDistanceIn) / (_turnRadiusIn - wheelToCenterDistanceIn);
			msg ("Speed Ratio:" + speedRatio + " turnToAngle cmd:"+ _turnPowerLevel + " Center wheel distance:" + wheelToCenterDistanceIn);
			
			// Determine which wheel has to speed up to turn
			if (_turnAngleDeg > 0) {
				rightCmdLevel = (_turnPowerLevel);
				leftCmdLevel = (_turnPowerLevel * speedRatio);
		
			} else {
				rightCmdLevel = (_turnPowerLevel * speedRatio);
				leftCmdLevel = (_turnPowerLevel);
			}
			
			// Convert turn distance in inches to encoder counts
			if (_turnAngleDeg > 0) {
				outerDistanceCnts = 2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) *	SRXDriveBaseCfg.kLeftEncoderCountsPerIn;
			} else {
				outerDistanceCnts = 2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) * SRXDriveBaseCfg.kRightEncoderCountsPerIn;
			}
			msg("Outer Wheel Distance Counts" + outerDistanceCnts);
		// Active state -  check for end of encoder count
		} else if ((_turnAngleDeg >= 0 && (getLeftEncoderPosition() > outerDistanceCnts))
					|| (_turnAngleDeg <= 0 && (getRightEncoderPosition() > outerDistanceCnts))) {
				if (_isCascadeTurn) {
					isTurnToAngleActive = false;
					msg("Cascade Active flag=> isTurnToAngleActive:" + isTurnToAngleActive);
				} else {
				// Apply power level in opposite direction for 1 second to brake
				rightCmdLevel = -(Math.signum(_turnAngleDeg) * SRXDriveBaseCfg.kStopBrakeValue);
				leftCmdLevel = -(Math.signum(_turnAngleDeg) * SRXDriveBaseCfg.kStopBrakeValue);
				if (!delay(1)) {
					isTurnToAngleActive = false;
					isDriveTrainMoving = false;
					setBrakeMode(false);
					rightCmdLevel = 0;
					leftCmdLevel = 0;
					msg("turn to angle complete");
				}
			}
		}
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			driveRightMasterMtr.set(ControlMode.Velocity, (rightCmdLevel*SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
			driveLeftMasterMtr.set(ControlMode.Velocity, (leftCmdLevel*SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
		} else {
			driveRightMasterMtr.set(ControlMode.PercentOutput,rightCmdLevel);
			driveLeftMasterMtr.set(ControlMode.PercentOutput,leftCmdLevel);
		}
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.0f===LftEnc:%-8.0f ===RgtEnc:%-8.0f%n",
									outerDistanceCnts,
									getLeftEncoderPosition(), 
									getRightEncoderPosition());
		}
		return isTurnToAngleActive;
	}
	
	
	
	/**
	* =======================================================================================
	* SRXDriveBase TEST METHODS
	* =======================================================================================
	*/
	public void testMotorPulse_SquareWave(boolean isTestMotorPulse_SquareWaveContinuous, boolean _isTestForRightDrive) {
		if (!SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			
			// initialize and start at low speed
			if (!isPulse_SqWaveFnctStartActive) {
				isPulse_SqWaveFnctStartActive = true;
				isLowTimeActive = true;
				pulSqStartTimeSec = Timer.getFPGATimestamp(); // seconds

			// 20ms latter start low power output
			} else {
				if (isLowTimeActive) {

					// Stay at a low speed for klowSQTime ms then switch to high
					// power level
					if (_isTestForRightDrive) {
						driveRightMasterMtr.set(ControlMode.Velocity, (SRXDriveBaseCfg.kSquareWaveLowPower * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
						driveLeftMasterMtr.set(ControlMode.Velocity, 0);
					} else {
						driveLeftMasterMtr.set(ControlMode.Velocity, (SRXDriveBaseCfg.kSquareWaveLowPower * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
						driveRightMasterMtr.set(ControlMode.Velocity, 0);
					}
					if ((Timer.getFPGATimestamp() - pulSqStartTimeSec) > SRXDriveBaseCfg.kSquareWaveLowPowerTimeSec) {

						// setup for high power output
						isLowTimeActive = false;
						pulSqStartTimeSec = Timer.getFPGATimestamp();
					}
				} else {

					// Stay at a high power for kHighSQTime ms then switch to low power
					if (_isTestForRightDrive) {
						driveRightMasterMtr.set(ControlMode.Velocity, (SRXDriveBaseCfg.kSquareWaveHighPower * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
						driveLeftMasterMtr.set(ControlMode.PercentOutput,0);
					} else {
						driveLeftMasterMtr.set(ControlMode.Velocity, (SRXDriveBaseCfg.kSquareWaveHighPower * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
						driveRightMasterMtr.set(ControlMode.PercentOutput,0);
					}
					if ((Timer.getFPGATimestamp() - pulSqStartTimeSec) > SRXDriveBaseCfg.kSquareWaveHighPowerTimeSec) {
						
						// setup for low power velocity
						isLowTimeActive = true;
						pulSqStartTimeSec = Timer.getFPGATimestamp();
						
						if(!isTestMotorPulse_SquareWaveContinuous){
							// Reset method flags for next call to motorPulse_SquareWaveTest method
							isLowTimeActive = false;
							isPulse_SqWaveFnctStartActive = false;
							driveLeftMasterMtr.set(ControlMode.Velocity,0);
							driveRightMasterMtr.set(ControlMode.Velocity,0);
						}
						
					}
				}
				SmartDashboard.putNumber("Test Low Power", SRXDriveBaseCfg.kSquareWaveLowPower);
				SmartDashboard.putNumber("Test High Power", SRXDriveBaseCfg.kSquareWaveHighPower);
				if (_isTestForRightDrive) {
					SmartDashboard.putNumber("Test Right Speed(cntsPerSampleTime)", getRightClosedLoopVelocity());
					SmartDashboard.putNumber("Test Right Error", getRightCloseLoopError());
					SmartDashboard.putNumber("Test Left Speed(cntsPerSampleTime)", 0);
					SmartDashboard.putNumber("Test Left Error", 0);
				} else {
					SmartDashboard.putNumber("Test Right Speed(cntsPerSampleTime)", 0);
					SmartDashboard.putNumber("Test Right Error", 0);
					SmartDashboard.putNumber("Test Left Speed(cntsPerSampleTime)", getLeftClosedLoopVelocity());
					SmartDashboard.putNumber("Test Left Error", getLeftCloseLoopError());
				}
			}
		} else {
			// Reset method flags for next call to motorPulse_SquareWaveTest method
			isLowTimeActive = false;
			isPulse_SqWaveFnctStartActive = false;
			driveLeftMasterMtr.set(ControlMode.Velocity,0);
			driveRightMasterMtr.set(ControlMode.Velocity,0);
		}
	}

	public boolean testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel){
		if (!isTestMoveForStraightCalActive){
			isTestMoveForStraightCalActive = true;
			cycleCount = 0;
			leftEncoderCounts = _testDistanceIn / SRXDriveBaseCfg.kLftInchesPerCount;
			leftCmdLevel = _pwrLevel;
			rightCmdLevel = _pwrLevel + SmartDashboard.getNumber("Right Correction Factor", SRXDriveBaseCfg.kDriveStraightCorrection); 
			isDriveTrainMoving = true;
			setRightEncPositionToZero();
			setLeftEncPositionToZero();
		} else if (getLeftEncoderPosition() >= leftEncoderCounts) {
			
			// Apply power level in opposite direction for 1 second to brake
			rightCmdLevel = -SRXDriveBaseCfg.kStopBrakeValue;
			leftCmdLevel = -SRXDriveBaseCfg.kStopBrakeValue;
			if (!delay(1)) {
				isTestMoveForStraightCalActive = false;
				isDriveTrainMoving = false;
				setBrakeMode(false);
				rightCmdLevel = 0;
				leftCmdLevel = 0;
			}	
		}
		calCorrectionFactor = getLeftEncoderPosition() / getRightEncoderPosition();
		driveLeftMasterMtr.set(ControlMode.PercentOutput,leftCmdLevel);
		driveRightMasterMtr.set(ControlMode.PercentOutput,rightCmdLevel);
		
		if (isLoggingDataEnabled) {
			String outputString = String.format("%8.0f,%8.0f,%8.0f,%8.4f", 
										leftEncoderCounts, 
										getLeftEncoderPosition(), 
										getRightEncoderPosition(),
										calCorrectionFactor);
		}
		
		return isTestMoveForStraightCalActive;
			// Log data
			//DebugLogger.data(outputString);}
		}
		public boolean testDriveStraight(double _testDistanceIn1, double _pwrLevel1){
		if (!isTestMoveForStraightActive){
			isTestMoveForStraightActive = true;
			cycleCount = 0;
			leftEncoderCounts = _testDistanceIn1 / SRXDriveBaseCfg.kLftInchesPerCount;
			leftCmdLevel = _pwrLevel1;
			rightCmdLevel = _pwrLevel1; 
			isDriveTrainMoving = true;
			setRightEncPositionToZero();
			setLeftEncPositionToZero();
		} else if (getLeftEncoderPosition() >= leftEncoderCounts) {
			
			// Apply power level in opposite direction for 1 second to brake
			rightCmdLevel = -SRXDriveBaseCfg.kStopBrakeValue;
			leftCmdLevel = -SRXDriveBaseCfg.kStopBrakeValue;
			if (!delay(1)) {
				isTestMoveForStraightActive = false;
				isDriveTrainMoving = false;
				setBrakeMode(false);
				rightCmdLevel = 0;
				leftCmdLevel = 0;
			}
			driveStraightEncoderError = getLeftEncoderPosition() - getRightEncoderPosition();
			rightCmdLevel += (driveStraightEncoderError * .03); 
		}
		calCorrectionFactor = getLeftEncoderPosition() / getRightEncoderPosition();
		driveLeftMasterMtr.set(ControlMode.PercentOutput,leftCmdLevel);
		driveRightMasterMtr.set(ControlMode.PercentOutput,rightCmdLevel);
		
		if (isLoggingDataEnabled) {
			String outputString = String.format("%8.0f,%8.0f,%8.0f,%8.4f", 
										leftEncoderCounts, 
										getLeftEncoderPosition(), 
										getRightEncoderPosition(),
										calCorrectionFactor);
			// Log data
			DebugLogger.data(outputString);
		}
		//Print on console data
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.0f===LftEnc:%-8.0f ===RgtEnc:%-8.0f ===Correction:%-8.4f%n", 
								leftEncoderCounts, 
								getLeftEncoderPosition(), 
								getRightEncoderPosition(),
								calCorrectionFactor);
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
			driveRightMasterMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, SRXDriveBaseCfg.kTimeoutMs);
			driveRightMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			driveRightMasterMtr.setSelectedSensorPosition(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kTimeoutMs);
			driveRightMasterMtr.configMotionCruiseVelocity(_rightCruiseVel, SRXDriveBaseCfg.kTimeoutMs);
			driveRightMasterMtr.configMotionAcceleration(_rightAccel, SRXDriveBaseCfg.kTimeoutMs);
			
			driveLeftMasterMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, SRXDriveBaseCfg.kTimeoutMs);
			driveLeftMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);	
			driveLeftMasterMtr.setSelectedSensorPosition(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kTimeoutMs);
			driveLeftMasterMtr.configMotionCruiseVelocity(_leftCruiseVel, SRXDriveBaseCfg.kTimeoutMs);
			driveLeftMasterMtr.configMotionAcceleration(_leftAccel, SRXDriveBaseCfg.kTimeoutMs);

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