package org.usfirst.frc.team2228.robot;

public class TestDriveBaseCFG {
	public static final double COUNTS_PER_REV = 676.0;
	public static final double WHEEL_DIAMETER = 6.0; // inches
	
	public static final double COUNTS_INCH = COUNTS_PER_REV / (WHEEL_DIAMETER * Math.PI);
	
	//testing and calibration
	public static final double K_RIGHT_HIGH_SPEED = 0.75;
	public static final double K_LEFT_HIGH_SPEED = 0.75;
	public static final double K_RIGHT_LOW_SPEED = 0.25;
	public static final double K_LEFT_LOW_SPEED = 0.25;
	public static final double K_SQ_WAVE_HIGH_TIME = 500.0;
	public static final double K_SQ_WAVE_LOW_TIME = 500.0;
}