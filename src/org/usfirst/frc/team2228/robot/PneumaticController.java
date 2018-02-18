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
	public Solenoid brake = new Solenoid(RobotMap.CAN_ID_10, RobotMap.PCM_PORT_3);
	
	public boolean pressureSwitch = c.getPressureSwitchValue();
	private boolean lastButton2 = false;
	private boolean triggered = false;

	public PneumaticController(DriverIF _driverIF) {
		driverIF = _driverIF;
		brakeSet(false);
	}

	public void teleopPeriodic() {
		c.setClosedLoopControl(true);
		//System.out.println(brake.get());
		if (driverIF.squeeze()) {
			squeezies.set(on);
			System.out.println("squeeze");
		}
		else if (driverIF.release()){
			squeezies.set(off);
		}
		//Tests to see if button is pressed, and then actuates on the release
		if (!driverIF.cubeRotateToggle() && lastButton2 && triggered == false) {
			lift.set(true);
			triggered = true;
			System.out.println("cubeRotateUp toggle active");
		}
		else if (!driverIF.cubeRotateToggle() && lastButton2 && triggered == true) {
			lift.set(false);
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
	// beep bop boopedy beep beep

}
