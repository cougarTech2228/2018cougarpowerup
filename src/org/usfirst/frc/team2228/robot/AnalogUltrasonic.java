package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;

public class AnalogUltrasonic {

	private final double ROBORIO_VOLTAGE = 4.697;
	private final double MM_TO_INCHES = 25.4;
	private final double MAX_READ_DISTANCE = 5000;// maximum distance the us
													// sensor can read in mm
	AnalogInput sensor = new AnalogInput(1);
	AnalogInput sensor2 = new AnalogInput(2);
	PIDSource pid;
	double[] Sensor1 = new double[10];
	double[] Sensor2 = new double[10];
	double[] ffGains = new double[10];
	double[] fbGains = new double[10];
	int index = 0;
	LinearDigitalFilter filter = new LinearDigitalFilter(pid, ffGains, fbGains);
	LinearDigitalFilter fifo = LinearDigitalFilter.movingAverage(pid, 10);
	public double voltage1, voltage2;
	double distance1, distance2;
	private int round;

	public AnalogUltrasonic() {
		double Dis1 = ((sensor.getVoltage() * MAX_READ_DISTANCE) / ROBORIO_VOLTAGE) / MM_TO_INCHES;
		double Dis2 = ((sensor2.getVoltage() * MAX_READ_DISTANCE) / ROBORIO_VOLTAGE) / MM_TO_INCHES;
		for(int i = 0; i < Sensor1.length; i++) {
			Sensor1[i] = Dis1;
			Sensor2[i] = Dis2;
		}
	}
	public void updateSensors() {
		Sensor1[index] = ((sensor.getVoltage() * MAX_READ_DISTANCE) / ROBORIO_VOLTAGE) / MM_TO_INCHES;
		Sensor2[index] = ((sensor2.getVoltage() * MAX_READ_DISTANCE) / ROBORIO_VOLTAGE) / MM_TO_INCHES;
		index++;
		if(index >= Sensor1.length)
			index = 0;
	}
	public double getDistance1() {
		double total = 0;
		for(int i = 0; i < Sensor1.length; i++)
			total += Sensor1[i];
		return round(total / Sensor1.length);
	}

	public double getDistance2() {
		double total = 0;
		for(int i = 0; i < Sensor2.length; i++)
			total += Sensor2[i];
		return round(total / Sensor2.length);
	}
	
	public void addffGains(){
		int i = 0;
		ffGains[i] = filter.pidGet();
		i++;
		if(i == 10){
			i = 0;
		}
	}
	public void roundTo(int DecimalPlaces) {
		round = (int) Math.pow(10, DecimalPlaces);
	}

	private double round(double d) {
		return Math.round(d * round) / (double)round;
	}
}