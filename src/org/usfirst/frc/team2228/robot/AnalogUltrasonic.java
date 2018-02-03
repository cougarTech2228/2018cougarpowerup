package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogUltrasonic {
	AnalogInput sensor = new AnalogInput(1);
	public double voltage;
	double distance;
	private double round;

	public AnalogUltrasonic() {
		
	}

	public double getDistance() {
		voltage = sensor.getVoltage();
		distance = ((voltage * 5000.0) / 4.85) / 25.4 + 4;//distance in inches
		return round(distance);
	}
	
	public void roundTo(double d) {
		round = d;
	}
	
	private double round(double d) {
		return (int)((d + 0.5) / round) * round;
	}
}