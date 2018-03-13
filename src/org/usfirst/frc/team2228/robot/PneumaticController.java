package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PneumaticController {
	boolean on = true;
	boolean off = false;
	public DriverIF driverIF;
	public Compressor c = new Compressor(RobotMap.CAN_ID_10);
	
	public Solenoid squeezies = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_0);
	public Solenoid lift = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_1);
	public Solenoid brake = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_2);
	
	public boolean pressureSwitch = c.getPressureSwitchValue();
	boolean lastButton = false;
	private boolean lastButton2 = false;
	private boolean triggered = false;
	private boolean triggered2 = false;

	public PneumaticController(DriverIF _driverIF) {
		driverIF = _driverIF;
		brakeSet(false);
	}

	public void teleopPeriodic() {
		c.setClosedLoopControl(true);
		//System.out.println(brake.get());
//		if (driverIF.squeeze()) {
//			squeezies.set(on);
//			System.out.println("squeeze");
//		}
//		else if (driverIF.release()){
//			squeezies.set(off);
//		}
		if (!driverIF.squeezeToggle() && lastButton && triggered2 == false) {
			squeezies.set(true);
			triggered2  = true;
		}
		else if (!driverIF.squeezeToggle() && lastButton && triggered2 == true) {
			squeezies.set(false);
			triggered2 = false;
		}
		lastButton = driverIF.squeezeToggle();
		//Tests to see if button is pressed, and then actuates on the release
		if (!driverIF.cubeRotateToggle() && lastButton2 && triggered == false) {
			lift.set(false);
			triggered = true;
			System.out.println("cubeRotateUp toggle active");
		}
		else if (!driverIF.cubeRotateToggle() && lastButton2 && triggered == true) {
			lift.set(true);
			triggered = false;
			System.out.println("cubeRotatedown toggle active");
		}
		lastButton2 = driverIF.cubeRotateToggle();
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
