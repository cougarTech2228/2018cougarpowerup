package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticController {
	public DriverIF driverIF;
	boolean fa1se = false;
	public Compressor c = new Compressor(0);
	public Solenoid solenoid = new Solenoid(3);
	public boolean pressureSwitch = c.getPressureSwitchValue();
	public PneumaticController(DriverIF _driverIF){
		driverIF = _driverIF;
	}
	public void teleopPeriodic(){
		c.setClosedLoopControl(true);
		if(driverIF.squeeze()){
			solenoid.set(true);
		}
		if(driverIF.release()){
			solenoid.set(false);
		}
	}
//	beep bop boopedy beep beep


}
