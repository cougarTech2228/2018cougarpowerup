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
	private boolean lastButton = false;
	private boolean lastButton2 = false;
	private boolean triggered = false;
	private boolean triggered2 = false;
	
	private Toggler toggle = new Toggler(2);

	public PneumaticController(DriverIF _driverIF) {
		driverIF = _driverIF;
		brakeSet(false);
	}

	public void teleopPeriodic() {
		c.setClosedLoopControl(true);
		pneumaticToggle(driverIF.squeezeToggle(), true, squeezies);
		pneumaticToggle(driverIF.cubeRotateToggle(), true, lift);
	}

	public void liftSet(boolean state) {
		lift.set(state);
	}

	public void brakeSet(boolean state) {
		brake.set(state);
	}

	/**
	 * 
	 * @param button
	 *            - button you would like to toggle
	 * @param lastButton
	 *            - set to state of button after loop
	 * @param triggered
	 *            - if motor is on or off
	 * @param onDirection
	 *            - primary direction of solenoid
	 * @param solenoid
	 *            - solenoid user would like to activate
	 */
	public void pneumaticToggle(boolean button, boolean onDirection, Solenoid solenoid) {
		if(toggle.toggle(button) == 0)
			solenoid.set(onDirection);
		else
			solenoid.set(!onDirection);
		lastButton = button;
	}
	// beep bop boopedy beep beep

}
