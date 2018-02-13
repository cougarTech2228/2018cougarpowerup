package org.usfirst.frc.team2228.robot;
/**
* Class SRXDriveBase API
* RELEASE: 2, RevA 180128
* Team 2228 
*
* The SRXDriveBase provides several levels of control for the drive base for a six wheel differential drive.
1 - No encoders
2 - With encoders with added features of driving straight and autonomous motion
3 - With heading angle and ultrasonics - added features improved driving straight, perpendicular drive movement
4 - Advance trapezoidal indexing with SRX magic motion

*=============================================
* PROGRAM CONFIGURATION
*=============================================
1 - Using Iterative Robot the Robot Init will create the SRXDriveBase Instance which is passed to autonomous and teleop
2 - In autonomousInit and teleopInit the user needs to call setClearActionFlags(). (It was found that during testing
    disabling in the middle of SRXBaseDrive method execution did not reset action flags)
3 - In teleop a teleopController class will interface with a user interface(game controller), define all teleop commands and 
    interface with the SRXDriveBase
3 - Standard input for left/right or throttle/turn is -1 to 1
4 - Standard "sign" of input values: Left/Right viewed from back of robot to front of robot, Forward/Reverse - Positive/Negative, 
     angle right - Positive, angle left - Negative

*==========================================
* CONFIGURATION/CALIBRATION OF SRXDRIVEBASE
*==========================================
1 - Wire up two motor on each side (left/Right). Encoder should be placed on one motor on each side
2 - Test that the user interface provides a left drive cmd and right drive cmd (-1 to 1)
3 - Power up RoboRio and using Internet Explorer with Silverlight bring up RoboRio WebDashboard
4 - using "self test" check CAN ID, 
5 - Using SRXDriveBase: method "SetDriveTrainCmdLevel" move left and right side to see that motor are rotating correctly and in the 
    correct direction - switch motor output if needed to have the motor on one side rotate in the correct direction
6 - Connect encoders and with robot disabled turn wheels for forward motion and change sensor direction to have encoders increase in value 
    when wheels are rotated to have robot move forward.	
7 - Using calibration method "testDriveStraightCalibration" determine drive straight calibration
8 - Enable feedback control and using method "testMotorPulse_SquareWave" determing PID values


public class SRXDriveBaseAPI {
/**
*
* =======================================================================================
* SRXBaseDrive SET METHODS
* =======================================================================================


	
	public void setRightEncPositionToZero() {
		// clear right master encoder counter to zero
	}
	public void setRightPositionToZero() {
		// this will also clear the encoder counter to zero
	}
	public void setLeftEncPositionToZero() {
		// clear the left master encoder counter to zero
	}
	public void setLeftPositionToZero() {
		// this will also clear the encoder counter to zero 
	}
	public void setBrakeMode(boolean isBrakeEnabled) {
		// this set the brake mode true-brake, false-coast
	}
	public void setStopMotors(){
		// stops motors 
	}
	public void setEnableConsoleData(boolean _consoleData){
		// true - enables console messages
	}
	public void setEnableLoggingData(boolean _loggingData){
		// true - enables data logging to a csv file - needs filezilla to get file from the roborio
	}
	public void setClearActionFlags() {
		// this clears program paths in methods - this is required in teleinit and autoinit
	}
	public void setDriveTrainRamping (double _SecToMaxPower) {
		// this sets ramp rate in SRX - defined as seconds to max power
	}
	

* =======================================================================================
* SRXBaseDrive GET METHODS
* =======================================================================================

	public double getRawRightEncoder(){
		// gets the right master raw encoder
	}
	public double getRightEncoderPosition() {
		// gets the right master encoder value with corrected sign for forward
	}
	public double getRightEncoderVelocity() {
		// gets encoder velocity - counts / 100ms
	}
	public double getRightClosedLoopPosition(){
		// gets the closed loop position 
	}
	public double getRightMstrMtrCurrent() {
		// gets the right master motor current
	}
	public double getRightFollowerMtrCurrent() {
		// gets the right follower motor current
	}
	public double getRightClosedLoopVelocity() {
			// gets the right master motor velocity - counts / 100ms
	}
	public double getRightCloseLoopError() {
		// gets the right master motor close loop error - counts / 100ms
	}
	//========================== Left Master
	public double getRawLeftEncoder() {
		// gets the raw data from the left encoder counts
	}
	public double getLeftEncoderPosition() {
		// gets the encoder count with corrected sign for forward
	}
	public double getLeftClosedLoopPosition(){
		// gets the left closed loop position
	}
	public double getLeftFollowerMtrCurrent() {
		// gets left follower motor current
	}
	public double getLeftClosedLoopVelocity() {
		// gets left master motor velocity
	}
	public double getLeftCloseLoopError() {
		// gets left master motor close loop error - counts / 100ms
	}
	public double getBusVoltage() {
		// gets the bus voltage
	}
	public boolean getIsMoving {
		// This method returns true is the robot is moving
	}
*
* =======================================================================================
* STATUS METHODS
* =======================================================================================
*
	public void DisplayChangeParmeters() {
		// displays on smartdashboard values that are changed for tuning
	}
	public void UpdateSRXDriveDataDisplay() {
		// Reads encoder, velocity, current, error, and displays on smartdashboard	
	}
	public void logSRXDriveData(){
		// logs drive data to file
	}
	private void msg(String _msgString) {
		// prints a message - if the same message is sent it will not be printed
	}
*
* =======================================================================================
* TELEOP METHODS
* =======================================================================================
*
	
	public void SetDriveTrainCmdLevel(	double _rightCMDLevel, 
										double _leftCMDLevel) {
		
		 // Note: left drive is master drive axis for the robot - the right drive
		 // will be modified for driving straight
		 
		 // NOTE: Motion command with open loop reflect power levels (-1 to 1) * (the
		 // motor bus voltage). Motion command with closed loop reflect speed level
		 // (-1 to 1) * (top motor RPM)
	}

	
	public void WPISetThrottleTurn(	double throttleValue, 
									double turnValue) {
		 // WPI throttle and turn commands This method uses WPI library methods to
		 // drive the robot with a throttle and turn input. Drives were set up by:
		 // driveStyle = new RobotDrive(driveRightMasterMtr, driveLeftMasterMtr); The
		 // throttle would be the game controller Y-axis(joystick fwd/rev) and turn
		 // would be game conctroller X-axis(joystick left/right)
		 
		 // NOTE: WPILib throttleValue and turnValue are open loop power levels (-1
		 // to 1) * (the motor bus voltage). The speed is determined by this power
		 // level and the load to the motor.
	}

	 *
	 * setThrottleTurn is both open loop and closed loop control with drive
	 * straight/drive perpendicular correction
	 *
	public void setThrottleTurn(double _throttleValue, 				// (-1 to 1) rev-neg value/fwd-pos value
								double _turnValue, 					// (-1 to 1) left-neg value/right-pos value
								boolean _isDrivingPerpendicular) {
		// 	_isDrivingPerpendicular -(true): assists drive to have robot move perpendicular into a wall						
	}
	
	public void setFastTurn(boolean _isFastTurnRight) {
		// future TODO
		// If robot blocked by another robot Fast turn turns robot 180 deg to escape
	}

*
* =======================================================================================
* AUTONOMOUG METHODS
* =======================================================================================
*
	public boolean velMoveToPosition(double _MoveToPositionIn, 
										double _MoveToPositionPwrLevel, 
										boolean _isCascadeMove) {
		// This method moves the robot with a predetermined power level and stops at
		// the specified position value. The move will be in brake mode to stop
		// method will check that robot is stopped and set brake mode back to coast and respond
		// that move is done
	}
	public boolean movePerpendicularToStop(	double _movePerpendicularPowerLevel, 
											double _movePerpendicularStopIn) {
	}
	public boolean rotateToAngle(	double _rotateToAngle, 
									double _rotatePowerLevel) {
		// direction(true)-rotates right, direction(false)-rotates left
	} 

	public boolean turnByEncoderToAngle(double _turnAngleDeg, 
										double _turnRadiusIn, 
										double _turnPowerLevel, 
										boolean _isDirectionReverse, 
										boolean _isCascadeTurn ) {
	}
	
*
* =======================================================================================
* SRXDriveBase TEST METHODS
* =======================================================================================
*
	public void testMotorPulse_SquareWave(boolean isTestMotorPulse_SquareWaveContinuous, 
											boolean _isTestForRightDrive) {
	}

	public boolean testDriveStraightCalibration(double _testDistanceIn, 
												double _pwrLevel){
	} 
	public boolean testDriveStraight(double _testDistanceIn1, double _pwrLevel1) {
		
	}
	
	public boolean delay(double _seconds){
		
	}
	
*
* =======================================================================================
* INDEX AND PROFILE COMMANDS
* =======================================================================================
*
	public boolean magicMove(	double _rightCruiseVel, 
								double _rightAccel, 
								double _rightDistance, 
								double _leftCruiseVel,
								double _leftAccel, 
								double _leftDistance) {
		// This method performs a SRX magic motion command from user calculated
		// values
		// User should note that the right drive distance needs to be corrected
		// by kDriveStraightCorrection
	}
	
	
	
	driveIndexRobot method:
	
	 
	indexRobot uses "Magic Motion" in the Talon SRX modules to index the robot. 
	"Magic Motion" SRX method needs 
		cruise velocity(cnts/100msTime),
		acceleration rate (cnts/100msTime/100msTime), and 
		distance(encoder counts)

	The following equations for a trapezoid with 1/3 time segments are
	used to determine params for "Magic Motion" move:

	Velocity = 1.5*(Distance / Time) Accel = Decel = 4.5*(Distance / Time2)

	Velocity(RPM) = (1.5*(Distance(in) / Time(sec))*60(sec/min)) / Wheel Circum(in/rev) 
	Acceleration(RPM/sec) = ((1.5*(Distance(in) / Time(sec))*60(sec/min)) / Wheel Circum(in/rev)) / Ta(sec) 
	 
	 Ta should be Time/3 
	 Distance(Encoder Counts) = Distance(in) / (in/count)
	 *
	
*/
