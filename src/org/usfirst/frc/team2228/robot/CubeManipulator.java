package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CubeManipulator {
	boolean on = true;
	boolean off = false;
	public DriverIF driverIF;
	public Compressor c = new Compressor(RobotMap.CAN_ID_10);
	
	public Solenoid squeezies = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_0);
	public Solenoid lift = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_1);
	public Solenoid brake = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_2);
	public Solenoid push = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_3);
	
	public boolean pressureSwitch = c.getPressureSwitchValue();
	boolean lastButton3 = false;
	private Spark left;
	private Spark right;
	boolean lastButton2 = false;
	//double initTime;
	public CubeManipulator(DriverIF _driverIF) {
		driverIF = _driverIF;
		brake.set(false);
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
		if(driverIF.cubeGrapToggle()) {
			squeezies.set(true);
		}
		else squeezies.set(false);
		
		if(driverIF.cubeLiftToggle()) {
			 lift.set(true);
		}
		else lift.set(false);
		
		if(driverIF.wheelState() == -1) {
			rollerSet(-1);
		}
		else if(driverIF.wheelState() == 1) {
			rollerSet(1);
		}
		else {
			left.stopMotor();
			right.stopMotor();
		}

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
	public void rollerSet(double speed) {
		left.set(speed);
		right.set(speed);
	}
	// beep bop boopedy beep beep

}
