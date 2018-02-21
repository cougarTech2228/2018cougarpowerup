package org.usfirst.frc.team2228.robot;

public class SRXDriveBaseCfg {
		// configuration flags
		// ===============================================
		// SRX ESC MODULE
		//timeoutMS is recommended to be 10 milliseconds for bootup sequence according to the manual (3.1.2.1)
		
		public static int kslotIDx = 0;
		public static int kPIDLoopIDx = 0;
		//hoi
		// ==============================================================
		// SET MOTOR DIRECTION
		public static boolean isDriveRightMasterMtrReversed = false;
		public static boolean isDriveRightFollowerMtrReversed = false;
		public static boolean isDriveLeftMasterMtrReversed = true;
		public static boolean isDriveLeftFollowerMtrReversed = true;
		
		//===============================================
		// BRAKE AND COAST PARAMETERS
		// sets SRX zero speed brake mode to brake(true) and coast(false)
		public static boolean isBrakeEnabled = true;
		
		public static double kRobotCoastToStopCounts = 0;
		public static double kTeleStopBrakeValue = 0.05;		
		public static double kAutoMoveStopBrakeValue = 0.05;
		public static double kAutoRotateStopBrakeValue = 0.05;
		public static double kAutoTurnStopBrakeValue = 0.05;
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
		public static boolean isRightEncoderSensorReversed = false;
		public static boolean isLeftEncoderSensorReversed = true;
		
		// CIMcode magnetic quadrature 20 cycles per revolution
		public static int EncoderCyclesPerRev = 20;
		
		// AndyMark tough box mini (50.0/14.0)*(48.0/16.0)
		// CIMCoder and 2018 drive train gear ratio 72/11
		public static double kGearRatio = 6.5454;
		
		// = (cycles/rev) * (gearRatio) = 20*6.5454
		public static double kDriveEncoderCyclesPerRev = 130.9;
		
		// =============================================================
		// DRIVE TRAIN CALCULATIONS
		
		public static double kTrackWidthIn = 23;
		
		public static double kMeasuredRgtWheelDiameter = 4.0;
		public static double kMeasuredLftWheelDiameter = 4.0;
		//(kMeasuredRgtWheelDiameter + kMeasuredLftWheelDiameter)/2
		public static double kWheelDiameterIn = 4.0;
		// WheelDiameter * Math.PI
		public static double kMeasuredRgtWheelCircum = 9.425;
		public static double kMeasuredLftWheelCircum = 9.425;

		// ======================================
		// ENCODER ENCODER CALCULATIONS
		// inches per revolution / counts per revolution
		
		// cnts per rev = quadrature(4) * encoder square wave cycles per rev
		public static double kCountsPerRevolution = 523.63;
		// Wheel circumference / counts per revolution
		public static double kRightInchesPerCount = 0.023;
		//old 0.01799;
		public static double kLeftInchesPerCount = 0.023;
		//old 0.01799;
		public static double kLeftEncoderCountsPerIn = 43.07;
		//old 55.586;
		public static double kRightEncoderCountsPerIn = 43.07;
		//old 55.586;
		
		//=======================================================
		// DRIVING STRAIGHT
		
		// Driving straight set up parameters
		public static boolean isDriveStraightAssistEnabled = false;
		
		// This value is determined by testDriveStraightCalibration method
		public static double kDriveStraightCorrection = 0.96; 
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
