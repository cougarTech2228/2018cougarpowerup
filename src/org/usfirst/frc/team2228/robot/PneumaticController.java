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
	public boolean pressureSwitch = c.getPressureSwitchValue();

	public PneumaticController(DriverIF _driverIF) {
		driverIF = _driverIF;
	}

	public void teleopPeriodic() {
		c.setClosedLoopControl(true);

		if (driverIF.squeeze()) {
			squeezies.set(on);
			System.out.println("squeeze");
		}
		else if (driverIF.release()) {
			System.out.println("release");
			squeezies.set(off);
		}
		if(driverIF.liftCube()){
			lift.set(on);
		}
		else if (driverIF.lowerCube()){
			lift.set(off);
		}
	}
	// beep bop boopedy beep beep

}
