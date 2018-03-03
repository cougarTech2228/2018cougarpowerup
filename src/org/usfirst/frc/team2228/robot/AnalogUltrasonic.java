package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;

public class AnalogUltrasonic {

	private final double ROBORIO_VOLTAGE = 4.697;
	private final double MM_TO_INCHES = 25.4;
	private final double MAX_READ_DISTANCE = 5000;// maximum distance the us
													// sensor can read in mm
	AnalogInput sensor = new AnalogInput(0);
	AnalogInput sensor2 = new AnalogInput(1);
	PIDSource pid;
	double[] distanceArray1 = new double[10];
	double[] distanceArray2 = new double[10];
	double[] ffGains = new double[10];
	double[] fbGains = new double[10];
	LinearDigitalFilter filter = new LinearDigitalFilter(pid, ffGains, fbGains);
	LinearDigitalFilter fifo = LinearDigitalFilter.movingAverage(pid, 10);
	public double voltage1, voltage2;
	double distance1, distance2;
	private double round;

	public AnalogUltrasonic() {

	}

	public double getDistance1() {
		int i = 0;
		voltage1 = sensor.getVoltage();
		distance1 = ((voltage1 * MAX_READ_DISTANCE) / ROBORIO_VOLTAGE) / MM_TO_INCHES;      // distance
																							// in
																							// inches
		distanceArray1[i] = distance1;
		i++;
		if (i == 10) {
			i = 0;
		}
		return round(distance1);

	}

	public double getDistance2() {
		int i = 0;
		voltage2 = sensor2.getVoltage();
		distance2 = ((voltage2 * MAX_READ_DISTANCE) / ROBORIO_VOLTAGE) / MM_TO_INCHES;      // distance
																							// in
																							// inches
		distanceArray2[i] = distance2;
		i++;
		if (i == 10) {
			i = 0;
		}
		return round(distance2);
	}
	
	public void addffGains(){
		int i = 0;
		ffGains[i] = filter.pidGet();
		i++;
		if(i == 10){
			i = 0;
		}
	}
	public void roundTo(double d) {
		round = d;
	}

	private double round(double d) {
		return (int) ((d + 0.5) / round) * round;
	}
}