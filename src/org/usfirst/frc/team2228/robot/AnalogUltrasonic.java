package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogUltrasonic {
	AnalogInput sensor = new AnalogInput(1);
	double voltage;
	double distance;

	public AnalogUltrasonic(AnalogInput sensor) {
		
	}
	public double getDistance(){
		voltage = sensor.getVoltage();
		return voltage;
	}
}