package org.usfirst.frc.team2228.robot;

public class SRXDriveBaseCfg {
	// configuration flags
		// ===============================================
		// SRX ESC MODULE
		
		//timeoutMS is recommended to be 10 milliseconds for bootup sequence according to the manual (3.1.2.1)
		public static int kTimeoutMs = 5;
		public static int kslotIDx = 0;
		public static int kPIDLoopIDx = 0;
		
		// ==============================================================
		// Set motor direction
		public static boolean isDriveRightMasterMtrReversed = false;
		public static boolean isDriveRightFollowerMtrReversed = false;
		public static boolean isDriveLeftMasterMtrReversed = true;
		public static boolean isDriveLeftFollowerMtrReversed = true;
		
		
		// sets SRX zero speed brake mode to brake(true) and coast(false)
		public static boolean isBrakeEnabled = false;
		
		// ====================================================
		// SRX CLOSE LOOP SETUP PARAMETERS
		
		public static boolean isSRXClosedLoopEnabled = false;
		
		public static double kdriveRightMstrFeedForwardGain = 0.025;
		public static double kdriveRightMstrProportionalGain = 0.3;
		public static double kdriveRightMstrIntegralGain = 0;
		public static double kdriveRightMstrDerivativeGain = 0;
		public static int kdriveRightMstrIzone = 0;
		public static int kdriveRightMstrRampRate = 0;
		public static int kdriveRightMstrProfile = 0;
		
		public static double kdriveLeftMstrFeedForwardGain = 0.025;
		public static double kdriveLeftMstrProportionalGain = 0.3;
		public static double kdriveLeftMstrIntegralGain = 0;
		public static double kdriveLeftMstrDerivativeGain = 0;
		public static int kdriveleftMstrIzone = 0;
		public static int kdriveLeftMstrRampRate = 0;
		public static int kdriveLeftMstrProfile = 0;
		
		// =============================================
		// DEADBANDS
		
		public static int kClosedLoopErr = 100;
		// 0.001 represents 0.1% - default value is 0.04 or 4%
        public static double kSpeedDeadBand = 0.1;
		public static double kNeutralDeadBand = 0.08;
		
		// This sets the velocity calculation time sample
		public static int kSRXVelocitySample = 64;
		
		
		// ======================================
		// ENCODER PARAMETERS AND ENCODER CALCULATIONS
		
		// Encoder setup Parameters
		public static boolean isMasterEncodersPresent = true;
		
		// The following changes the encoder sign internal to the SRX only
		// If direct read of encoder is negative in fwd dir is----EncoderSensorReversed = true
		public static boolean isRightEncoderSensorReversed = false;
		public static boolean isLeftEncoderSensorReversed = true;
		
		// CIMcode magnetic quadrature 20 cycles per revolution
		public static int EncoderCyclesPerRev = 20;
		public static double kGearRatio = (72/11);
		public static int kDriveEncoderCyclesPerRev = EncoderCyclesPerRev * (int)kGearRatio;
		// =============================================================
		// DRIVE TRAIN CALCULATIONS
		
		// AndyMark tough box mini (50.0/14.0)*(48.0/16.0)
		// 2018 vex chassis 11:72
		public static double kMeasuredRgtWheelDiameter = 4.0;
		public static double kMeasuredLftWheelDiameter = 4.0;
		public static double kWheelDiameterIn = (kMeasuredRgtWheelDiameter + kMeasuredLftWheelDiameter)/2;
		public static double kTrackWidthIn = 23;		
		public static double kMeasuredRgtWheelCircum = kMeasuredRgtWheelDiameter*Math.PI;
		public static double kMeasuredLftWheelCircum = kMeasuredLftWheelDiameter*Math.PI;

		// ======================================
		// ENCODER ENCODER CALCULATIONS
		// inches per revolution / counts per revolution
		
		// cnts per rev = quadrature(4) * encoder square wave cycles per rev
		public static double kCountsPerRevolution = 4*kDriveEncoderCyclesPerRev;
		public static double kRightInchesPerCount = kMeasuredRgtWheelCircum/kCountsPerRevolution;
		public static double kLeftInchesPerCount = kMeasuredLftWheelCircum/kCountsPerRevolution;
		public static double kLeftEncoderCountsPerIn = 1 / kLeftInchesPerCount;
		public static double kRightEncoderCountsPerIn = 1 / kRightInchesPerCount;
		
		//=======================================================
		// DRIVING STRAIGHT
		
		// Driving straight setup parameters
		public static boolean isDriveStraightAssistEnabled = false;
		
		// This value is determined by testDriveStraightCalibration method
		public static double kDriveStraightCorrection = 0.957;
		
		//===============================================
		//MOTION METHOD PARAMETERS
		
		// See topRPM calibration procedure for this parameter
		public static double kTopRPM = 1000;
		// RPM * 1/60sec * 1/(10 100ms samples/sec) * counts/rev = counts/100ms
		public static double MaxVel_VelNativeUnits = kTopRPM * (1/60) * (1/10) * kCountsPerRevolution;
		
		//================================================
		// DRIVE TRAIN STALL PARAMETERS
		public static int kStallCurrentContinuousAmps = 10;
		public static int kStallCurrentPeakAmps = 100;
        public static int kStallTimeMs = 6000;
		
		//===============================================
		// BRAKE AND COAST PARAMETERS
		public static double kRobotCoastToStopCounts = 0;
		public static double kAutoStopBrakeValue = 0.05;
		public static double kTeleStopBrakeValue = 0.05;
		
		//===============================================
		// MAGIC MOTION SETUP PARAMETERS
		
		public static double kRgtDistanceCalibration = 1.0;
		public static double kLftDistanceCalibration = 1.0;
		
}

