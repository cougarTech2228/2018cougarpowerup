package org.usfirst.frc.team2228.robot;
/**
+ Class SRXDriveBaseManual
+ RELEASE: 2, RevA 180128
+ Team 2228 
+
+ The SRXDriveBase provides several levels of control for the drive base for a six wheel differential drive.
1 - No encoders
2 - With encoders with added features of driving straight and autonomous motion
3 - With heading angle and ultrasonics - added features improved driving straight, perpendicular drive movement
4 - Advance trapezoidal indexing with SRX magic motion

+=============================================
+ PROGRAM CONFIGURATION
+=============================================
1 - Using Iterative Robot the Robot Init will create the "SRXDriveBase()" or "SRXDriveBase( AngleIF _angle, DistanceIF _distance)" 
    Instance which is passed to autonomous and teleop
2 - In autonomousInit() and teleopInit() the USER NEEDS TO CALL "setSRXDriveBaseInitialization()". (It was found that during testing
    disabling in the middle of SRXBaseDrive method execution did not reset action flags. It also sets up smartDashboard)
3 - In teleop a "TeleopController(DriverIF _driverIF, SRXDriveBase _driveBase)" class will interface with a user interface(game controller), 
    define all teleop commands and interface with the SRXDriveBase

+==========================================
+ TELEOPCONTROLLER AND SRXDRIVEBASE METHODS
+==========================================
1 - Method descriptions detailed at end of this document
	
+=================================================
+ SIGNAL LEVELS AND DIRECTION SIGN VALUES STANDARDS
+=================================================
1 - Standard input for left/right drive or throttle/turn is (-1 to 1)
2 - Left/Right viewed from back of robot to front of robot
3 - Forward(Positive)/Reverse(Negative), 
4 - Angle Right(Yaw) - Positive, Angle Left(Yaw) - Negative
5 - Encoder forward - increments up; Encoder Reverse - increments down

+=================================================
+ SHUFFLEBOARD
+=================================================
1 - shuffleboard is an FIRST application to view program variables and change calibration values
2 - In eclipse select wpilib and then shuffleboard or manually start from "c:\'users\Admin\wpilib\Tools\shuffleboard.jar"
3 - The shuffleboard screen file is located at "c:\'users\Admin\shuffleboard\shuffleboard SRXBaseDrive.json"
4 - In Autonomous or teleop call "driveBase.testMethodSelection()" to use the shuffleboard for the SRXDriveBase
5 - smartDashboard variables are located in the left panel under nettables. Right click to select type of widget to put variable in
6 - Note: variables in shuffleboard/smartDashboard are located by their "string keys". For SRXDriveBase "string Keys" start with "CAL_"
6 - If shuffleboard does stops working close down and manually start

+==========================================
+ CONFIGURATION/CALIBRATION OF SRXDRIVEBASE
+==========================================

+=====
SETUP MOTOR CAN ID, MOTOR DIRECTION, AND ENCODER DIRECTION
+=====
1 - Wire up two motor on each side (left/Right). Encoder should be placed on one motor on each side
2 - Test that the user interface provides a left drive cmd and right drive cmd (-1 to 1)
3 - Power up RoboRio and using "Internet Explorer with Silverlight" bring up "RoboRio WebDashboard"
4 - Using "self test" check with robot disabled
    a - CAN ID
    b - Encoder direction - Manually move left/right wheel in forward direction and check encoder value - if negative reverse in "SRXDriveBaseCfg"
	    Note: this will not change in Webdashboard
		
        public static boolean isRightEncoderSensorReversed = false;
		public static boolean isLeftEncoderSensorReversed = true;
		
5 - Using Shuffleboard "SetDriveTrainCmdLevel"  and change power to left or right (-1 to 1) to see that motor are rotating correctly
    a - switch motor output in "SRXDriveBaseCfg" if needed to have the motor on one side rotate in the correct direction. Download new compile.
	Note: This does not change forward in code just changes output of SRX module
	
		// SET MOTOR DIRECTION
		public static boolean isDriveRightMasterMtrReversed = false;
		public static boolean isDriveRightFollowerMtrReversed = false;
		public static boolean isDriveLeftMasterMtrReversed = true;
		public static boolean isDriveLeftFollowerMtrReversed = true;

+=====
+ TAKE ROBOT MEASUREMENTS
+=====
1 - Enter robot measurements into SRXDriveBaseCfg
	a - EncoderCyclesPerRev
		kGearRatio
		kDriveEncoderCyclesPerRev
		kTrackWidthIn
		kMeasuredRgtWheelCircum with a thin mm ruler and convert to inches
		kMeasuredLftWheelCircum with a thin mm ruler and convert to inches
	b - Back calculate diameter and do encoder calculations in SRXDriveBaseCfg
	c - enable encoders in SRXBaseDriveCfg
			"isMasterEncodersPresent = true;"
+=====
+ SHUFFLEBOARD FOR SRXDRIVEBASE TESTING
+=====
1 - Using shuffleboard, console display to calibrate SRXDriveBase methods
	a - In SRXDriveBaseCfg set:
		"isConsoleDataEnabled = true;"  
	b - Open shuffleboard application from eclipse wpilib menu 
	c - In "teleopPeriodic()" call "driveBase.testMethodSelection()" 
	
NOTE!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 - CAL_ values have been removed presently - need to change values in config files

+=====
+ TEST LEFT AND RIGHT DRIVES
+=====
1 - Put robot up on blocks with wheels off the ground
2 - Open RoboRio WebDashboard
3 - Using calibration method "CAL_SetDriveTrainCmdLevel"  to test left and right drives with power levels (-1 to 1)
	a - set	"CAL_leftPwrLevel" and "CAL_rightPwrLevel"
	b - use "CAL_isSTART_DriveCmd" and "CAL_isSTOP_DriveCmd" start and stop power level commands to left/right drives
4 - View WebDashboard to see velocity, encoder, 	
	
+=====
+ DETERMINE DRIVE STAIGHT VARIABLE, MOVE FORWARD BRAKE AND MOVE FORWARD COAST VALUES
+=====
1 - First put robot on blocks with wheels off the ground			
2 - Using calibration method "CAL_testDriveStraightCalibration"  to determine:
    a - drive straight factor for right drive (left drive is master for robot). 
	    Set "CAL_kDriveStraightCorrection" to 1.0 for first cut at drive straight factor
	b - Change "CAL_kDriveStraightCorrection" until the encoders are very close to each other after a run
	c - set brake values for forward movement. The values should not make the robot to move backwards
			"CAL_kAutoRightMoveStopBrakeValue"
			"CAL_kAutoLeftMoveStopBrakeValue"
	d - determine coast values for robot coast after braking from console display
			"CAL_kAutoMoveCoastToStopCounts"
3 - UPDATE! UPDATE! - SRXBaseDriveCfg constants
	a - CAL default values are loaded from SRXBaseDriveCfg when you reload a new compile!!!!!!!!
			"kDriveStraightCorrection"
			"kAutoMoveCoastToStopCounts"
			"kAutoRightMoveStopBrakeValue"
			"kAutoLeftMoveStopBrakeValue"
			REEEEEEEEE
+=====
+ CALIBRATE MOVE
+=====
1 - Using calibration method "CAL_VelMoveToPosition"  to determine distance, braking and coast
	a -	Set "CAL_distanceIn" and "CAL_pwrLevel" - power level (-1 to 1)
	b - Set brake values for forward movement	
		"CAL_kAutoRightMoveStopBrakeValue"
		"CAL_kAutoLeftMoveStopBrakeValue"
	c - Ramp value is set at 2 seconds to ramp to full power
	d - Determine coast value for robot coast after braking
		"CAL_kAutoMoveCoastToStopCounts"
	
2 - UPDATE! UPDATE! - SRXBaseDriveCfg constants
	a - CAL default values are loaded from SRXBaseDriveCfg when you reload a new compile!!!!!!!!
			"kAutoMoveCoastToStopCounts"
			"kAutoRightMoveStopBrakeValue"
			"kAutoLeftMoveStopBrakeValue"
			
+=====
+ CALIBRATE ROTATE
+=====
1 - Using calibration method "CAL_Rotate"  to determine rotate, braking and coast			
2 - Set "CAL_rotateAngleDeg", "CAL_pwrLevel,CAL_kAutoRightRotateStopBrakeValue", 
	"CAL_kAutoLeftRotateStopBrakeValue", "CAL_kAutoRotateCoastToStopCounts"
3 - UPDATE! UPDATE! - SRXBaseDriveCfg constants
	a - CAL default values are loaded from SRXBaseDriveCfg when you reload a new compile!!!!!!!!
			"kRoatateDriveStraightCorrection"
			"kAutoRotateCoastToStopCounts"
			"kAutoRightRotateStopBrakeValue"
			"kAutoLeftRotateStopBrakeValue"
+=====
+ CALIBRATE TURN
+=====		
1 - Using calibration method "CAL_Turn"  to determine rotate, braking and coast
2 - set "CAL_turnAngleDeg", "CAL_turnRadiusIn", "CAL_pwrLevel", "CAL_kAutoTurnCoastToStopCounts",
	"CAL_kAutoRightTurnStopBrakeValue", "CAL_kAutoLeftTurnStopBrakeValue"
3 - UPDATE! UPDATE! - SRXBaseDriveCfg constants
	a - CAL default values are loaded from SRXBaseDriveCfg when you reload a new compile!!!!!!!!	
        "kAutoTurnCoastToStopCounts"
		"kAutoRightTurnStopBrakeValue"
		"kAutoLeftTurnStopBrakeValue"
		
+=====
+ CALIBRATE FEEDBACK PID
+=====
1 - Using shuffleboard "SetDriveTrainCmdLevel" set output level to 1
2 - From RoboRio WebDashBoard => Read quad encoder velocity (native_cnts/100ms) and UPDATE SRXDriveBaseCfg 
		"MaxVel_VelNativeUnits"	
3 - Calculate top RPM: (vel(cnts/100ms) * 600) / (counts/rev)  and UPDATE "kTopRPM" 
4 - Calculate feedforward: 1023 / (Vel[cnts/100ms]) and UPDATE "kdriveRightMstrFeedForwardGain"
5 - UPDATE isSRXClosedLoopEnabled = true;
6 - Using shuffleboard start "Pulse_SquareWave" and from console display look at closed loop error
7 - Make first cut P gain value: (10% * 1023) / [closed loop error] => 102.3 / [close loop error]
8 - Now use RoboRio WebDashboard to better tune the PID loop
9 - UPDATE UPDATE SRXDriveBaseCfg
		kdriveRightMstrFeedForwardGain
		kdriveRightMstrProportionalGain
		kdriveRightMstrIntegralGain
		kdriveRightMstrDerivativeGain
		
		kdriveLeftMstrFeedForwardGain
		kdriveLeftMstrProportionalGain
		kdriveLeftMstrIntegralGain
		kdriveLeftMstrDerivativeGain
		
		

public class SRXDriveBaseAPI {

+
+ =======================================================================================
+ SRXBaseDrive SET METHODS
+ =======================================================================================
	public void setSRXDriveBaseInitialization(){
		- this clears program paths in methods - this is required in teleopInit and autonomousInit
	}

	public void setRightEncPositionToZero() {
		- clear right master encoder counter to zero
	}
	public void setRightSensorPositionToZero() {
		- this will also clear the encoder counter to zero
	}
	public void setLeftEncPositionToZero() {
		- clear the left master encoder counter to zero
	}
	public void setLeftSensorPositionToZero() {
		- this will also clear the encoder counter to zero 
	}
	public void setCorrectionSensor(int _CorrectionSensorSelect){
		-
	}
	public void setBrakeMode(boolean isBrakeEnabled) {
		- this set the brake mode true-brake, false-coast
	}
	public void setStopMotors(){
		- stops motors 
	}
	public void setEnableConsoleData(boolean _consoleData){
		- true - enables console messages
	}
	public void setEnableLoggingData(boolean _loggingData){
		- true - enables data logging to a csv file - needs filezilla to get file from the roborio
	}
	public void setSRXDriveBaseInitialization(){
		- this clears program paths in methods - this is required in teleinit and autoinit
	}
	public void setDriveTrainRamp(double _SecToMaxPower) {
		- this sets ramp rate in SRX - defined as seconds to max power
	}
	

+ =======================================================================================
+ SRXBaseDrive GET METHODS
+ =======================================================================================

	public double getRightEncoderPosition() {
		- This is updated every 160ms
		- gets the right master encoder value with corrected sign for forward
	}
	public double getRightEncoderVelocity() {
		- This is updated every 160ms
		- gets encoder velocity - counts / 100ms
	}
	public double getRightSensorPosition(){
		- This value is updated every 20ms
		- gets the sensor position 
	}
	public double getRightMstrMtrCurrent() {
		- gets the right master motor current
	}
	public double getRightFollowerMtrCurrent() {
		- gets the right follower motor current
	}
	public double getRightSensorVelocity() {
		- This value is updated every 20ms
		- gets the right master motor velocity - counts / 100ms
	}
	public double getRightCloseLoopError() {
		- gets the right master motor close loop error - counts / 100ms
	}
	+========================== Left Master
	
	public double getLeftEncoderPosition() {
		- This value is updated every 160ms
		- gets the encoder count with corrected sign for forward
	}
	public double getLeftEncoderVelocity(){
		- This value is updated every 160ms
	}
	public double getLeftSensorPosition(){
		- This value is updated every 20ms
		- gets the left closed loop position
	}
	public double getLeftMstrMtrCurrent() {
		- gets left master motor current
	}
	public double getLeftFollowerMtrCurrent() {
		- gets left follower motor current
	}
	public double getLeftSensorVelocity() {
		- gets left master motor velocity
	}
	public double getLeftCloseLoopError() {
		- gets left master motor close loop error - counts / 100ms
	}
	public double getBusVoltage() {
		- gets the bus voltage
	}
	public double getDriveStraightCorrection(){
		- gets drive straight correction from encoder, IMU, DistanceSensor
	}
	public boolean IsDriveMoving {
		- This method returns true is the robot is moving
	}
+
+ =======================================================================================
+ STATUS METHODS
+ =======================================================================================
+
	public void loadShuffleBoardParmeters() {
		- method buttons and shuffleboard values that are changed for calibrating
	}
	public void UpdateSRXDriveDataDisplay() {
		- Reads encoder, velocity, current, error, and displays on smartdashboard	
	}
	public void logSRXDriveData(){
		- logs drive data to file
	}
	private void msg(String _msgString) {
		- "if setEnableConsoleData(true)" prints a message - if the same message is sent it will not be printed
	}
+
+ =======================================================================================
+ TELEOP METHODS
+ =======================================================================================
+
	
	public void SetDriveTrainCmdLevel(	double _rightCMDLevel, double _leftCMDLevel) {
		
		 - Note: left drive is master drive axis for the robot - the right drive
		 - will be modified for driving straight
		 
		 - NOTE: Motion command with open loop reflect power levels (-1 to 1) + (the
		 - motor bus voltage). Motion command with closed loop reflect speed level
		 - (-1 to 1) + (top motor RPM)
	}

	
	public void WPISetThrottleTurn(	double throttleValue, 
									double turnValue) {
		 - WPI throttle and turn commands This method uses WPI library methods to
		 - drive the robot with a throttle and turn input. Drives were set up by:
		 - driveStyle = new RobotDrive(driveRightMasterMtr, driveLeftMasterMtr); The
		 - throttle would be the game controller Y-axis(joystick fwd/rev) and turn
		 - would be game conctroller X-axis(joystick left/right)
		 
		 - NOTE: WPILib throttleValue and turnValue are open loop power levels (-1
		 - to 1) + (the motor bus voltage). The speed is determined by this power
		 - level and the load to the motor.
	}

	 +
	 + setThrottleTurn is both open loop and closed loop control with drive
	 + straight/drive perpendicular correction
	 +
	public void setThrottleTurn(double _throttleValue, 				// (-1 to 1) rev-neg value/fwd-pos value
								double _turnValue, 					// (-1 to 1) left-neg value/right-pos value
								boolean _isDrivingPerpendicular) {
		- 	_isDrivingPerpendicular -(true): assists drive to have robot move perpendicular into a wall						
	}
	

+
+ =======================================================================================
+ SRXDriveBase AUTONOMOUG METHODS
+ =======================================================================================
+
Note!!!!!!!!!!!!!!!!!!!!
Before calling any autonomous method the encoders need to be zeroed
					setRightSensorPositionToZero();
					setLeftSensorPositionToZero();
					setDriveTrainRamp(2);
					Timer.delay(0.2);

	public boolean velMoveToPosition(double _MoveToPositionIn, 
										double _MoveToPositionPwrLevel, 
										boolean _isCascadeMove) {
		- This method moves the robot with a predetermined power level and stops at the specified position value.
		- IF _MoveToPositionIn = 0 the robot will stop on a sensor input
	}
	public boolean movePerpendicularToStop(	double _movePerpendicularPowerLevel, 
											double _movePerpendicularStopIn) {
	}
	public boolean rotateToAngle(	double _rotateToAngle, 
									double _rotatePowerLevel) {
		- direction(true)-rotates right, direction(false)-rotates left
	} 

	public boolean turnByEncoderToAngle(double _turnAngleDeg, 
										double _turnRadiusIn, 
										double _turnPowerLevel, 
										boolean _isDirectionReverse, 
										boolean _isCascadeTurn ) {
	}
	
+
+ =======================================================================================
+ SRXDriveBase TIMED AUTONOMOUG METHODS
+ =======================================================================================
+	
	
	public boolean timeVelMoveToPosition(double _MoveToPositionSec, double _MoveToPositionPwrLevel, boolean _isCascadeMove){
		
	}
	public boolean timeRotateToAngle(double _rotateTimeToAngleSecSec, double _rotatePowerLevel) {
		
	}
	public boolean timeTurnToAngle(double _turnAngleSec, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn ) {
		
	}
+
+ =======================================================================================
+ SRXDriveBase TEST SELECTION / SHUFFLE BOARD METHODS
+ =======================================================================================
+	
	-  the following needs to be added to class Robot extends IterativeRobot
	
	- This function is called once during test mode
	@Override
	public void testInit() {
		SRXDriveBase.SRXDriveBaseInit();
		loadShuffleBoardParmeters();
	}
	
	- This function is called periodically during test mode
	@Override
	public void testPeriodic() {
		SRXDriveBase.testMethodSelection();
	}
	
	- =====================================
	if(SmartDashboard.getBoolean("TstBtn-StepFnc:", false)){
		- testStepFunction(double _stepFunctionPower, double _stepFunctionTimeSec, boolean _isTestForRightDrive)
	}
	if(SmartDashboard.getBoolean("TstBtn-DrvStraightCal:", false)){
		- testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel)
		- Shuffleboard also uses CAL_kDriveStraightCorrection that is changed for the driveStraightCorrection variable
	}
	if(SmartDashboard.getBoolean("TstBtn-VelMoveToPos:", false)){
		- velMoveToPosition(double _MoveToPositionIn, double _MoveToPositionPwrLevel, boolean _isSensorStopUsed, boolean _isCascadeMove)
	}
	if(SmartDashboard.getBoolean("TstBtn-TimeVelMove:", false)){
		- timeVelMoveToPosition(double _MoveToPositionSec, double _MoveToPositionPwrLevel, boolean _isCascadeMove)
	}
	if(SmartDashboard.getBoolean("TstBtn-RotateToAngle:", false)){
		- rotateToAngle(double _rotateToAngle, double _rotatePowerLevel)
	}
	if(SmartDashboard.getBoolean("TstBtn-TimeRotate:", false)){
		- timeRotateToAngle(double _rotateTimeToAngleSec, double _rotatePowerLevel) 
	}
	if(SmartDashboard.getBoolean("TstBtn-TurnToAngle:", false)){
		- turnByEncoderToAngle(double _turnAngleDeg, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn )
	}
	if(SmartDashboard.getBoolean("TstBtn-TimeTurn:", false)){
		- timeTurnToAngle(double _turnAngleSec, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn )
	}
	if (SmartDashboard.getBoolean("TstBtn-DriveCmdLevel:", false)){
		- SetDriveTrainCmdLevel(double _rightCMDLevel, double _leftCMDLevel)
		- Shuffleboard also uses CAL_RightDriveCmdLevel, CAL_LeftDriveCmdLevel to change the power level value of the drives
		
	}
	if(SmartDashboard.getBoolean("TstBtn-CascadeTest:", false)){
		
	}
	
	
+
+ =======================================================================================
+ SRXDriveBase TEST METHODS
+ =======================================================================================
+
	public void testMethodSelection(){
		-This uses Shuffleboard to test all methods
	}
	public boolean testStepFunction(double _testStepFunctionPower, double testStepFunctionTimeSec, boolean _isTestForRightDrive) {
		-
	}

	public boolean testDriveStraightCalibration(double _testDistanceIn, 
												double _pwrLevel){
		-
	} 
	==this needs work - sort of works
	public boolean autoTuneCorrectionFactor(double _autoTunepowerLevel){
		- This will automatically sequence the correction factor decimal digits to determine right wheel correction factor 
	}
		
	public boolean delay(double _seconds){
		- This delay is looked at each scan so delay = seconds + scan(~20ms) 
	}
	
+
+ =======================================================================================
+ INDEX AND PROFILE COMMANDS
+ =======================================================================================
+
	public boolean magicMove(	double _rightCruiseVel, 
								double _rightAccel, 
								double _rightDistance, 
								double _leftCruiseVel,
								double _leftAccel, 
								double _leftDistance) {
		- This method performs a SRX magic motion command from user calculated
		- values
		- User should note that the right drive distance needs to be corrected
		- by kDriveStraightCorrection
	}
	
	
	
	driveIndexRobot method:
	
	 
	indexRobot uses "Magic Motion" in the Talon SRX modules to index the robot. 
	"Magic Motion" SRX method needs 
		cruise velocity(cnts/100msTime),
		acceleration rate (cnts/100msTime/100msTime), and 
		distance(encoder counts)

	The following equations for a trapezoid with 1/3 time segments are
	used to determine params for "Magic Motion" move:

	Velocity = 1.5+(Distance / Time) Accel = Decel = 4.5+(Distance / Time2)

	Velocity(RPM) = (1.5+(Distance(in) / Time(sec))+60(sec/min)) / Wheel Circum(in/rev) 
	Acceleration(RPM/sec) = ((1.5+(Distance(in) / Time(sec))+60(sec/min)) / Wheel Circum(in/rev)) / Ta(sec) 
	 
	 Ta should be Time/3 
	 Distance(Encoder Counts) = Distance(in) / (in/count)
	 
	
*/

import edu.wpi.first.wpilibj.Timer;
