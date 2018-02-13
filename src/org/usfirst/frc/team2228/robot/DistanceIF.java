package org.usfirst.frc.team2228.robot;


import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Ultrasonic;

public class DistanceIF extends SampleRobot{
	AnalogInput sensor = new AnalogInput(1);
	//temporary return value for the double methods
	double a;
	//PIDController to be used for getPerpindicularDistanceCorrection()
	PIDController ultraSensor;
	//creates the ultra object and assigns ultra to be an ultrasonic sensor which uses DigitalOutput 1 for the echo pulse and DigitalInput 1 for the trigger pulse
	Ultrasonic ultra = new Ultrasonic(1, 1);
	// If true, sensor is on the bot and enabled. If false, sensor is not on the bot and/or the sensor is not enabled
	boolean isSensorEnabled = false;

	public DistanceIF(AnalogInput sensor) {
		
	}
	
	public void setCalibrationNear(){
		
	}
	
	public void setCalibrationFar(){
		
	}
	
	public double getLeftDistance(){
		return a;
	}
	
	public double getLeftFilteredDistance(){
		return a;
	}
	
	public double getRightDistance(){
		return a;
	}
	
	public double getRightFilteredDistance(){
		return a;
	}
	
	public void getPerpendicularDistanceCorrection(){
		
	}
}
