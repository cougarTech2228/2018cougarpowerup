package org.usfirst.frc.team2228.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PneumaticController {
	boolean on = true;
	boolean off = false;
	public DriverIF driverIF;
	public Compressor c = new Compressor(10);
	public Solenoid s = new Solenoid(10, 3);
	public boolean pressureSwitch = c.getPressureSwitchValue();
	public PneumaticController(DriverIF _driverIF){
		driverIF = _driverIF;
	}
	public void teleopPeriodic(){
		c.setClosedLoopControl(true);

		if(driverIF.squeeze()){
			s.set(on);
			System.out.println("squeeze");
		}
		if(driverIF.release()){
            System.out.println("release");
			s.set(false);
		}
	}
//	beep bop boopedy beep beep


}
