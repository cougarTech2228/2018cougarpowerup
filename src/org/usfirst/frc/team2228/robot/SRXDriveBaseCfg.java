package org.usfirst.frc.team2228.robot;

public class SRXDriveBaseCfg {
		// configuration flags
		// ===============================================
		// SRX ESC MODULE
		//timeoutMS is recommended to be 10 milliseconds for bootup sequence according to the manual (3.1.2.1)
		//heydud
		public static int kslotIDx = 0;
		public static int kPIDLoopIDx = 0;
		//hoi
		// ==============================================================
		// SET MOTOR DIRECTION
		// THESE ARE FOR MU
//		public static boolean isDriveRightMasterMtrReversed = true;
//		public static boolean isDriveRightFollowerMtrReversed = true;
//		public static boolean isDriveLeftMasterMtrReversed = false;
//		public static boolean isDriveLeftFollowerMtrReversed = false;
//		//I created a boolean that reverses turn in TeleopController by adding a - symbol
//		public static boolean isTurnReversed = true;
		//===============================================
		// SET MOTOR DIRECTION
		// THESE ARE FOR THE TEST BOT
		public static boolean isDriveRightMasterMtrReversed = false;
		public static boolean isDriveRightFollowerMtrReversed = false;
		public static boolean isDriveLeftMasterMtrReversed = true;
		public static boolean isDriveLeftFollowerMtrReversed = true;
		//I created a boolean that reverses turn in TeleopController by adding a - symbol
		public static boolean isTurnReversed = false;
		//===============================================
		// BRAKE AND COAST PARAMETERS
		// sets SRX zero speed brake mode to brake(true) and coast(false)
		public static boolean isBrakeEnabled = true;
		

		public static double kAutoMoveCoastToStopCounts = 0;
		public static double kAutoRotateCoastToStopCounts = 0;
		public static double kAutoTurnCoastToStopCounts = 0;
		
		public static double kTeleStopBrakeValue = 0.05;
		
		
		public static double kAutoRightMoveStopBrakeValue = 0.0;
		public static double kAutoLeftMoveStopBrakeValue = 0.0;
		
		public static double kAutoRightRotateStopBrakeValue = 0.0;
		public static double kAutoLeftRotateStopBrakeValue = 0.0;
		
		public static double kAutoRightTurnStopBrakeValue = 0.0;
		public static double kAutoLeftTurnStopBrakeValue = 0.0;
		
		public static double SecToMaxPower = 2;
		// ====================================================
		// SRX CLOSE LOOP SETUP PARAMETERS
		
		public static boolean isSRXClosedLoopEnabled = false;
		
		public static double kdriveRightMstrFeedForwardGain = 1.58;
		public static double kdriveRightMstrProportionalGain = 0.05;
		public static double kdriveRightMstrIntegralGain = 0;
		public static double kdriveRightMstrDerivativeGain = 0;
		public static int kdriveRightMstrIzone = 0;
		public static int kdriveRightMstrRampRate = 0;
		public static int kdriveRightMstrProfile = 0;
		
		public static double kdriveLeftMstrFeedForwardGain = 1.58;
		public static double kdriveLeftMstrProportionalGain = 0.05;
		public static double kdriveLeftMstrIntegralGain = 0;
		public static double kdriveLeftMstrDerivativeGain = 0;
		public static int kdriveleftMstrIzone = 0;
		public static int kdriveLeftMstrRampRate = 0;
		public static int kdriveLeftMstrProfile = 0;
		
		// =============================================
		// DEADBANDS
		
		public static int kClosedLoopErr = 1000;
		// 0.001 represents 0.1% - default value is 0.04 or 4%previousEMAAccelFltrThrottleValue;
        public static double kSpeedDeadBand = 0.1;
		public static double kNeutralDeadBand = 0.08;
		
		// This sets the velocity calculation time sample
		public static int kSRXVelocitySample = 64;
		
		
		// ======================================
		// ENCODER PARAMETERS AND ENCODER CALCULATIONS
		
		// Encoder set up Parameters
		public static boolean isMasterEncodersPresent = true;
		
		// The following changes the encoder sign internal to the SRX only
		// If direct read of encoder is negative in fwd dir is----EncoderSensorReversed = true
		public static boolean isRightEncoderSensorReversed = true;
		public static boolean isLeftEncoderSensorReversed = false;
		
		// CIMcode magnetic quadrature 20 cycles per revolution
		public static int EncoderCyclesPerRev = 20;
		
		// AndyMark tough box mini (50.0/14.0)*(48.0/16.0)
		// CIMCoder and 2018 drive train gear ratio 72/11
		public static double kGearRatio = 6.5454;
		
		// = (cycles/rev) * (gearRatio) = 20*6.5454
		public static double kDriveEncoderCyclesPerRev = 130.9;
		
		// =============================================================
		// DRIVE TRAIN CALCULATIONS
		
		public static double kTrackWidthIn = 22.875;
		
		public static double kMeasuredRgtWheelDiameter = 4.035;
		public static double kMeasuredLftWheelDiameter = 4.035;

		//(kMeasuredRgtWheelDiameter + kMeasuredLftWheelDiameter)/2
		public static double kWheelDiameterIn = 4.035;

		// WheelDiameter * Math.PI
		public static double kMeasuredRgtWheelCircum = 12.678;
		public static double kMeasuredLftWheelCircum = 12.678;

		// ======================================
		// ENCODER ENCODER CALCULATIONS
		// inches per revolution / counts per revolution
		
		// cnts per rev = quadrature(4) * encoder square wave cycles per rev
		public static double kCountsPerRevolution = 523.63;
		
		// Wheel circumference / counts per revolution

		public static double kRightInchesPerCount = 0.0242; //old 0.0179
		public static double kRightEncoderCountsPerIn = 41.302; //old 55.586
		public static double kLeftInchesPerCount = 0.0242; //old 0.01799
		public static double kLeftEncoderCountsPerIn = 41.302; //old 55.586

		
		//=======================================================
		// DRIVING STRAIGHT
		
		// Driving straight set up parameters

		// Cap sensor correction to % of throttle power level
		public static double kThrottlePowerRatio = 0.2;
		public static boolean isDriveStraightAssistEnabled = true;
		
		// This value is determined by testDriveStraightCalibration method
		public static double kRotateDriveStraightCorrection = 0.93;	
		public static double kDriveStraightCorrection = .9150;
				//for test bot 1.0805;
				//for robox 0.93; 
				// Hard floor correction 0.87;
		
		//===============================================
		//MOTION METHOD PARAMETERS
		
		// See t opRPM calibration procedure for this parameter
		// From RoboRio WebDashBoard => (vel(cnts/100ms) * 600) / (counts/rev)
		public static double kTopRPM = 645.9;
		// RPM * 1/60sec * 1/(10 => 100ms samples/sec) * counts/rev = counts/100ms
		public static double MaxVel_VelNativeUnits = 563.69;
		
		//================================================
		// DRIVE TRAIN STALL PARAMETERS
		public static int kStallCurrentContinuousAmps = 10;
		public static int kStallCurrentPeakAmps = 100;
        public static int kStallTimeMs = 6000;
		
		
		
		//===============================================
		// MAGIC MOTION SETUP PARAMETERS
		
		public static double kRgtDistanceCalibration = 1.0;
		public static double kLftDistanceCalibration = 1.0;
		
}

