package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CubeManipulator {
	boolean on = true;
	boolean off = false;
	public DriverIF driverIF;
	public Compressor c = new Compressor(RobotMap.CAN_ID_10);
	
	public Solenoid squeezies = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_0);
	public Solenoid lift = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_1);
	public Solenoid brake = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_2);
	
	public boolean pressureSwitch = c.getPressureSwitchValue();
	boolean lastButton3 = false;
	private boolean lastButton4 = false;
	private boolean triggered3 = false;
	private boolean triggered4 = false;
	private Spark left;
	private Spark right;
	private double cubeCollectionValue;
	private double cubeExpulsionValue;
	private boolean triggered = false;
	boolean triggered2 = false;
	private boolean lastButton = false;
	boolean lastButton2 = false;

	public CubeManipulator(DriverIF _driverIF) {
		driverIF = _driverIF;
		brakeSet(false);
		left = new Spark(RobotMap.PWM_PORT_0);
		right = new Spark(RobotMap.PWM_PORT_1);
		
		SmartDashboard.putNumber("CollectionValue", 0.75);
		SmartDashboard.putNumber("ExpulsionValue", -0.75);
		
		// positive is clockwise, negative is counter clockwise
		left.set(0);
		right.set(0);
		left.setInverted(true);
	}

	public void teleopPeriodic() {
		c.setClosedLoopControl(true);
		//System.out.println(brake.get());
		if (!driverIF.squeezeToggle() && lastButton3 && triggered3 == false) {
			squeezies.set(true);
			triggered3  = true;
		}
		else if (!driverIF.squeezeToggle() && lastButton3 && triggered3 == true) {
			squeezies.set(false);
			triggered3 = false;
		}
		lastButton3 = driverIF.squeezeToggle();
		//Tests to see if button is pressed, and then actuates on the release
		if (!driverIF.cubeRotateToggle() && lastButton4 && triggered4 == false) {
			lift.set(false);
			triggered4 = true;
			System.out.println("cubeRotateUp toggle active");
		}
		else if (!driverIF.cubeRotateToggle() && lastButton4 && triggered4 == true) {
			lift.set(true);
			triggered4 = false;
			System.out.println("cubeRotatedown toggle active");
		}
		lastButton4 = driverIF.cubeRotateToggle();
		cubeCollectionValue = SmartDashboard.getNumber("CollectionValue", 0.8);
		cubeExpulsionValue = SmartDashboard.getNumber("ExpulsionValue", -0.8);
		if (!driverIF.collectionToggle() && lastButton && triggered == false) {
			left.set(cubeCollectionValue);
			right.set(cubeCollectionValue);
			triggered = true;
			System.out.println("Suck in");
		}
		else if (!driverIF.collectionToggle() && lastButton && triggered == true) {
			left.set(0);
			right.set(0);
			triggered = false;
		}
		if (!driverIF.expulsion() && lastButton2 && triggered2 == false) {
			left.set(cubeExpulsionValue);
			right.set(cubeExpulsionValue);
			triggered2 = true;
		}
		else if (!driverIF.expulsion() && lastButton2 && triggered2 == true) {
			left.set(0);
			right.set(0);
			triggered2 = false;
		}
		
//		else if(driverIf.expulsion()){
//		left.set(cubeExpulsionValue);
//		right.set(cubeExpulsionValue);
//		}
		lastButton = driverIF.collectionToggle();
		lastButton2 = driverIF.expulsion();
	}
	public void liftSet(boolean state){
		lift.set(state);
	}
	public void brakeSet(boolean state){
		brake.set(state);
	}
	public void squeezeSet(boolean state) {
		squeezies.set(state);
	}
	public void squeeze(boolean _grab){
		if(_grab == true){
			squeezies.set(true);
		}
		else{
			squeezies.set(false);
		}
	}
	// beep bop boopedy beep beep

}
