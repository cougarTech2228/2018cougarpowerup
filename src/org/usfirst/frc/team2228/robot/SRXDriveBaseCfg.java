package org.usfirst.frc.team2228.robot;

public class SRXDriveBaseCfg {
		//=======================================
		// SRXDRIVEBASE CONTROL FLAGS
		public static boolean isSRXClosedLoopEnabled = true;
		public static boolean isMasterEncodersPresent = true;
		public static boolean isDriveStraightAssistEnabled = false;
		public static boolean isMotionMagicEnabled = true;
		
		//!!!!!!!!!!!!!!!!!!!- MOTOR FWD -> ENCODER INCREASES COUNT AND POSITIVE -> SENSOR READ IS POSITIVE
		// =======================================
		// SET MOTOR DIRECTION
		public static boolean isDriveRightMasterMtrReversed = false;
		public static boolean isDriveRightFollowerMtrReversed = false;
		public static boolean isDriveLeftMasterMtrReversed = true;
		public static boolean isDriveLeftFollowerMtrReversed = true;
				
		// =======
		// FEEDBACK SENSOR - ENCODER DIRECTION
		
		// The following changes the encoder sign internal to the SRX only
		// If direct read of encoder is negative in fwd dir => is----EncoderSensorReversed = true
		public static boolean isRightEncoderSensorReversed = true;
		public static boolean isLeftEncoderSensorReversed = true;
		
		//========
		// PROGRAM ENCODER SENSOR DIRECTION
		
		public static boolean isRightEncoderSensorReadReversed = false;
		public static boolean isLeftEncoderSensorReadReversed = false;
		
		// ======================================
		// ROBOT MEASUREMENTS/DATA:
		
		// AndyMark tough box mini (50.0/14.0)*(48.0/16.0)
		// CIMCoder and 2018 drive train gear ratio 72/11
		public static double kGearRatio =6.5454;
				//6.5454;
		
		// CIMcode magnetic quadrature 20 cycles per revolution
		public static int EncoderCyclesPerRev = 20;
		
		// !!!!!!!!!!!!!!!!!!!!!! This is measured with a tape measure
		public static double kTrackWidthIn = 24.125;
				//22.875;
		
		// !!!!!!!!!!!!!!!!!!!!!! This is measured with a thin tape measure - use mm and convert to in
		public static double kMeasuredRgtWheelCircum = 18.8976;
				//12.678;
		public static double kMeasuredLftWheelCircum = 19.0551;
				//12.678;
		
		// =============================================================
		// DRIVE TRAIN CALCULATIONS
		
		// kDriveEncoderCyclesPerRev = (cycles/rev) * (gearRatio) = 20*8.459
				public static double kDriveEncoderCyclesPerRev = 130.9;
				
		// kCountsPerRevolution = quadrature(4) * kDriveEncoderCyclesPerRev = 4*169.18
		public static double kCountsPerRevolution = 676.72;
				//523.63;
		
		// kWheelCircumIn = (kMeasuredRgtWheelCircum + kMeasuredLftWheelCircum)/2
		public static double kWheelCircumIn = 18.97635;
		
		// kInchesPerCount = kWheelCircumIn / kCountsPerRevolution; 1/kInchesPerCount
		public static double kInchesPerCount = 0.036;
				//18.0242;
		public static double kEncoderCountsPerIn = 27.778;
				//41.302;
		
		//==========
		//INFO CALCULATIONS
		// Diameter = WheelCircum / Pi
		public static double kRgtWheelDiameter = 6.0153;
		public static double kLftWheelDiameter = 6.0654;
		
		// kRightInchesPerCount = kMeasuredRgtWheelCircum / kCountsPerRevolution; 1/kRightInchesPerCount
		public static double kRightInchesPerCount = 0.0361; 
		public static double kRightEncoderCountsPerIn = 27.855;
		
		// kLeftInchesPerCount = kMeasuredLftWheelCircum / kCountsPerRevolution; 1/kLeftInchesPerCount
		public static double kLeftInchesPerCount = 0.0359; 
		public static double kLeftEncoderCountsPerIn = 27.701;

		//===============================================
		// SRX CLOSED LOOP VELOCITY CALCULATIONS
		
		// Use RoboRio web dashboard: Run at high speed and read Output % and nativeUnit velocity - calculate max
		// MaxVel_VelNativeUnits = RPM * 1/60sec * 1/[10 => 100ms samples/sec] * kCountsPerRevolution = counts/100ms
		//From RoboRio WebDashBoard:
		public static double MaxVel_VelNativeUnits = 563.69;
	
		// kTopRPM = (vel[measureVelPeriod->100ms] * 600) / kCountsPerRevolution
		public static double kTopRPM = 645.9;
		
		
		
		
		// ===============================================
		// SRX ESC MODULE
		//timeoutMS is recommended to be 10 milliseconds for bootup sequence according to the manual (3.1.2.1)
		public static int kslotIDx = 0;
		public static int kPIDLoopIDx = 0;
		
		// =======
		// SRX CLOSE LOOP SETUP PARAMETERS
		public static double kdriveRightMstrFeedForwardGain = 1.5; //2
		public static double kdriveRightMstrProportionalGain = 2; //2
		public static double kdriveRightMstrIntegralGain = 0;
		public static double kdriveRightMstrDerivativeGain = 10; //10
		public static int    kdriveRightMstrIzone = 0;
		public static int    kdriveRightMstrRampRate = 0;
		public static int    kdriveRightMstrProfile = 0;
		
		public static double kdriveLeftMstrFeedForwardGain = 1.5; //2
		public static double kdriveLeftMstrProportionalGain = 2; //2
		public static double kdriveLeftMstrIntegralGain = 0;
		public static double kdriveLeftMstrDerivativeGain = 10; //10
		public static int    kdriveleftMstrIzone = 0;
		public static int    kdriveLeftMstrRampRate = 0;
		public static int    kdriveLeftMstrProfile = 0;
		
		// =============================================
		// DEADBANDS
		public static int kClosedLoopErr = 0;
		
		// 0.001 represents 0.1% - default value is 0.04 or 4%previousEMAAccelFltrThrottleValue;
        public static double kSpeedDeadBand = 0.1;
		public static double kNeutralDeadBand = 0.08;
		
		// This sets the velocity calculation time sample
		public static int kSRXVelocitySample = 64;
		
		//Sample period in ms from supported sample periods-default 100ms period/64 sample window
		public static double kvelMeasurePeriodSec =.1; 
		
		//===============================================
		// SRX DRIVE BASE BRAKE, POWER BRAKE, AND COAST PARAMETERS
		
		// sets SRX zero speed brake mode to brake(true) and coast(false)
		public static boolean isBrakeEnabled = true;
		
		public static double kAutoRightMoveStopBrakeValue = 0.0;
		public static double kAutoLeftMoveStopBrakeValue = 0.0;
		
		public static double kAutoRightRotateStopBrakeValue = 0.0;
		public static double kAutoLeftRotateStopBrakeValue = 0.0;
		
		public static double kAutoRightTurnStopBrakeValue = 0.0;
		public static double kAutoLeftTurnStopBrakeValue = 0.0;
		
		
		public static double kAutoMoveCoastToStopCounts = 0;
		public static double kAutoRotateCoastToStopCounts = 0;
		public static double kAutoTurnCoastToStopCounts = 0;	
		
		//================================================
		// DRIVING STRAIGHT
		
		// This value is determined by testDriveStraightCalibration method
		public static double kDriveStraightFwdCorrection = 1.02; // Hard floor correction 0.87;
		public static double kDriveStraightRevCorrection = 1.02;
		
		public static double kRotateCWDriveStraightCorrection = 0.75;// fwd-.93
		public static double kRotateCCWDriveStraightCorrection = 0.75;
		
		public static double kTurnRightDriveStraightCorrection = 1;
		public static double kTurnLeftDriveStraightCorrection = 1;
		
		// Cap sensor correction to % of throttle power level
		public static double kThrottlePowerRatio = 0.2;
	
		//================================================
		// DRIVE TRAIN STALL PARAMETERS
		public static int kStallCurrentContinuousAmps = 10;
		public static int kStallCurrentPeakAmps = 100;
        public static int kStallTimeMs = 6000;
	
}
