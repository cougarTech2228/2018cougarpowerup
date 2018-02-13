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
		else if (driverIF.release()) {
			System.out.println("release");
			squeezies.set(off);
		}
		if(driverIF.liftCube()){
			System.out.println("HYA");
			lift.set(on);
		}
		else if (driverIF.lowerCube()){
			System.out.println("HYADOS");
			lift.set(off);
		}
	}
	public void liftSet(boolean state){
		lift.set(state);
	}
	public void brakeSet(boolean state){
		brake.set(state);
	}
	// beep bop boopedy beep beep

}
